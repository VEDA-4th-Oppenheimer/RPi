/*
 * imu_driver.c — ICM-20948 I2C 캐릭터 디바이스 드라이버
 *
 * MPU-6050 에서 ICM-20948 로 교체. 배선은 동일(SDA/SCL/GND/VCC)하지만
 * 주의: **슬레이브 주소가 0x68 -> 0x69 로 바뀐다.** GY-521 은 AD0 를 L 로 묶어
 *   0x68 이었는데 이 ICM-20948 브레이크아웃은 AD0 가 H 다. 배선이 같아서
 *   주소도 같으려니 하고 넘어가기 쉬운 지점이다(overlays/imu-overlay.dts 참조).
 *
 * ── /dev/imu 계약은 바뀌지 않는다 ──────────────────────────────────────────
 *   read(fd, buf, 6) 이 가속도 3축을 **빅엔디안 int16** 로 준다.
 *     buf[0..1]=ax, buf[2..3]=ay, buf[4..5]=az
 *   ICM-20948 도 MPU-6050 과 같은 바이트 순서/폭이고, 아래 probe 가 레인지를
 *   ±2g(16384 LSB/g)로 두므로 **스케일도 동일**하다. 그래서 데몬(imu_module.c)
 *   과 테스트앱(imu_test.c)의 파싱 코드는 손대지 않았다.
 *
 * ── MPU-6050 과 다른 점 (이식하면서 실제로 바뀐 것) ────────────────────────
 *   ① 레지스터 뱅크. ICM-20948 은 레지스터가 4개 뱅크로 나뉘어 있고,
 *      REG_BANK_SEL(0x7F) 로 골라야 한다. 이 레지스터만 모든 뱅크에서 보인다.
 *      설정은 뱅크2, 데이터/전원은 뱅크0 이라 probe 가 뱅크를 오간다.
 *      주의: 읽기 경로(imu_read)는 뱅크0 만 건드리므로, probe 는 반드시
 *        **뱅크0 으로 되돌려놓고** 끝나야 한다.
 *   ② 주소 이동. PWR_MGMT_1  0x6B -> 0x06 (뱅크0)
 *                 가속도 시작  0x3B -> 0x2D (뱅크0)
 *   ③ WHO_AM_I 가 0xEA (MPU-6050 은 0x68). 엉뚱한 칩이 꽂힌 걸 probe 에서
 *      잡을 수 있게 확인한다 — 배선이 같아 물리적으로는 아무거나 꽂히므로
 *      이 검사가 유일한 방어선이다.
 *   ④ LP_EN. 리셋 직후 PWR_MGMT_1 = 0x41 로 SLEEP 과 저전력 모드가 켜져
 *      있다. SLEEP 만 풀고 LP_EN 을 남기면 샘플이 듬성듬성 갱신돼 "값이
 *      멈춘 것처럼" 보인다. 0x01 로 둘 다 풀고 CLKSEL=1(자동)을 쓴다.
 *   ⑤ 가속도 DLPF 를 켠다(5.7Hz). MPU-6050 때는 필터를 안 걸어 대역이
 *      통째로 열려 있었는데, 이 IMU 는 스텝모터 바로 옆에 있고 마운트가
 *      한쪽만 지지된 상태라 진동이 그대로 들어온다. 수평 게이트는 1Hz 판정
 *      이라 대역폭이 남아돌므로 세게 거는 편이 이득이다.
 *
 * ── 유지한 것 (MPU-6050 판에서 그대로) ────────────────────────────────────
 *   · imu_client 를 mutex 로 보호 (probe/remove 와 read 의 레이스)
 *   · noop_llseek (스트림 디바이스)
 *   · 초기화 write 반환값 체크
 *   · devnode 콜백으로 /dev/imu 를 0666 으로
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/of.h>      /* of_device_id — DT 매칭 */

/* --- 뱅크 선택 (모든 뱅크에서 접근 가능한 유일한 레지스터) ----------------*/
#define ICM20948_REG_BANK_SEL       0x7F
#define ICM20948_BANK_0             0x00   /* 값은 bit[5:4] 라 뱅크n = n<<4 */
#define ICM20948_BANK_2             0x20

/* --- 뱅크 0 ---------------------------------------------------------------*/
#define ICM20948_REG_WHO_AM_I       0x00
#define ICM20948_WHO_AM_I_VALUE     0xEA
#define ICM20948_REG_PWR_MGMT_1     0x06
#define ICM20948_REG_PWR_MGMT_2     0x07
#define ICM20948_REG_ACCEL_XOUT_H   0x2D

#define ICM20948_PWR1_DEVICE_RESET  0x80
/* SLEEP=0, LP_EN=0, CLKSEL=1(자동 선택 — 데이터시트 권장) */
#define ICM20948_PWR1_RUN           0x01
/* 자이로/가속도 모두 인가. 지금은 가속도만 읽지만, 나중에 자이로를 쓸 때
 * 여기만 보면 되도록 명시적으로 써 둔다(리셋 기본값과 같은 값이다). */
#define ICM20948_PWR2_ALL_ON        0x00

/* --- 뱅크 2 ---------------------------------------------------------------*/
#define ICM20948_REG_ACCEL_CONFIG   0x14
/* ACCEL_CONFIG = [5:3]DLPFCFG | [2:1]FS_SEL | [0]FCHOICE
 *   DLPFCFG=6 -> 5.7Hz,  FS_SEL=0 -> ±2g(16384 LSB/g),  FCHOICE=1 -> DLPF 사용
 * 주의: FS_SEL 을 바꾸면 데몬의 IMU_ACCEL_LSB_PER_G(16384) 도 같이 고쳐야 한다.
 *   두 곳이 어긋나면 각도가 아니라 스케일만 틀려서 잘 안 드러난다. */
#define ICM20948_ACCEL_CFG_2G_5HZ   0x31

static struct class *imu_class = NULL;
static int imu_major;
static struct device *imu_dev = NULL;

/* imu_client는 probe/remove(다른 컨텍스트)와 read(파일 read 컨텍스트)에서
 * 동시에 접근될 수 있으므로 mutex로 보호한다. */
static struct i2c_client *imu_client = NULL;
static DEFINE_MUTEX(imu_lock);

static int i2c_read_bytes(struct i2c_client *client, u8 reg, u8 *buf, u8 len)
{
    struct i2c_msg msgs[2] = {
        {
            .addr = client->addr,
            .flags = 0,             /* Write */
            .len = 1,
            .buf = &reg,
        },
        {
            .addr = client->addr,
            .flags = I2C_M_RD,      /* Read */
            .len = len,
            .buf = buf,
        }
    };
    /* 주의: i2c_transfer 는 **전송한 메시지 수**를 돌려준다. 성공이면 반드시
     *   2 이고, 어댑터에 따라 0 이나 1 이 나올 수 있다(주소 ACK 는 받았는데
     *   데이터 단계에서 끊긴 경우). ret < 0 만 보면 그 부분 전송을 성공으로
     *   읽어, 채워지지 않은 버퍼의 쓰레기 값을 가속도로 쓰게 된다. 수평
     *   게이트가 그 값으로 판정하므로 조용히 틀린 답을 낸다. */
    const int ret = i2c_transfer(client->adapter, msgs, 2);

    if (ret < 0)
        return ret;
    return (ret == 2) ? 0 : -EIO;
}

/* 레지스터 뱅크 전환. ICM-20948 의 거의 모든 초기화가 이걸 거친다.
 * 실패를 그냥 넘기면 **엉뚱한 뱅크의 같은 주소**에 쓰게 되므로 반환값을
 * 반드시 확인해야 한다 — 조용히 다른 레지스터를 망가뜨리는 유형이다. */
static int icm20948_set_bank(struct i2c_client *client, u8 bank)
{
    return i2c_smbus_write_byte_data(client, ICM20948_REG_BANK_SEL, bank);
}

static ssize_t imu_read(struct file *filep, char __user *buffer, size_t len, loff_t *offset)
{
    u8 raw_buf[6] = {0};
    int ret;
    struct i2c_client *client;

    if (len < 6)
        return -EINVAL;

    /* imu_client를 락 안에서 로컬로 스냅샷 — remove()가 그 사이 NULL로
     * 바꾸더라도 이 함수 안에서는 유효한 포인터를 계속 사용하게 된다. */
    mutex_lock(&imu_lock);
    client = imu_client;
    if (!client) {
        mutex_unlock(&imu_lock);
        return -ENODEV;
    }

    /* probe 가 뱅크0 으로 되돌려놓고 끝나고, 런타임에 뱅크를 바꾸는 경로가
     * 여기 말고 없으므로 매 read 마다 뱅크를 다시 고르지 않는다.
     * 주의: 나중에 자이로/자력계 설정을 런타임에 바꾸는 코드를 추가한다면,
     *   그 경로가 반드시 뱅크0 으로 복구하거나 여기서 매번 선택해야 한다. */
    ret = i2c_read_bytes(client, ICM20948_REG_ACCEL_XOUT_H, raw_buf, 6);
    mutex_unlock(&imu_lock);

    if (ret < 0) {
        pr_err_ratelimited("IMU Driver: Read Error %d\n", ret);
        return -EIO;
    }

    /* raw byte 덤프. 데몬이 1Hz 로 계속 읽으므로 pr_info 면 dmesg 가 끝없이
     * 쌓인다 — 정작 봐야 할 커널 메시지가 밀려나므로 pr_debug 로 낮춘다.
     * 필요할 때만 켠다:
     *   echo 'file imu_driver.c +p' > /sys/kernel/debug/dynamic_debug/control
     * (CONFIG_DYNAMIC_DEBUG 가 없으면 모듈을 -DDEBUG 로 빌드) */
    pr_debug("IMU Driver: raw = %02x %02x %02x %02x %02x %02x\n",
             raw_buf[0], raw_buf[1], raw_buf[2],
             raw_buf[3], raw_buf[4], raw_buf[5]);

    if (copy_to_user(buffer, raw_buf, 6))
        return -EFAULT;

    /* offset은 건드리지 않는다 (noop_llseek 로 seek 자체를 막아 의미를 명확히 함) */
    return 6;
}

static struct file_operations imu_fops = {
    .owner   = THIS_MODULE,
    .read    = imu_read,
    .llseek  = noop_llseek, /* 스트림 디바이스: seek 무의미 (커널 6.x에서 no_llseek 제거됨) */
};

/* /dev/imu 노드 권한. devtmpfs 가 노드를 만들 때 이 콜백으로 모드를 묻는다.
 *
 * 주의: turret 은 miscdevice 라 구조체의 .mode 필드 하나로 끝나지만, 여기는
 *   class_create + device_create 방식이라 클래스에 콜백을 달아야 한다.
 *   두 드라이버의 권한 설정 방법이 다른 이유가 이 등록 방식 차이다.
 *
 * 0666 인 근거(turret 과 동일):
 *   ① 원격에서 스캐너를 움직일 수 있는 유일한 경로는 MQTT mTLS 다.
 *      파일 권한은 그 경로에 관여하지 않는다.
 *   ② Raspberry Pi OS 의 pi 계정은 패스워드 없는 sudo 를 가지므로
 *      (/etc/sudoers.d/010_pi-nopasswd) 0600 으로 잠가도 로컬 공격을
 *      막지 못한다. 즉 0666 이 새로 열어주는 공격면이 없다.
 *   → 데몬을 systemd 서비스로 옮길 때는 udev 그룹 방식으로 바꿀 것.
 *      (드라이버 재컴파일 없이 정책을 바꿀 수 있다)
 *
 * 반환값 NULL = 기본 경로(/dev/imu) 유지. 이름을 바꾸려면 여기서 문자열을
 * 돌려주지만 우리는 device_create 가 준 이름 그대로 쓴다. */
static char *imu_devnode(const struct device *dev, umode_t *mode)
{
    (void)dev;
    if (mode)
        *mode = 0666;
    return NULL;
}

/* 칩 초기화. 성공해야만 호출자가 imu_client 를 공개한다(아래 probe 주석). */
static int icm20948_setup(struct i2c_client *client)
{
    int ret;

    /* 1) 뱅크0 으로 맞추고 DEVICE_RESET. 이전 세션(커널 내장 드라이버,
     *    이전 insmod)이 남긴 설정을 배제한다. 리셋 중에는 응답이 없을 수
     *    있어 반환값은 보지 않고 충분히 대기만 한다. */
    (void)icm20948_set_bank(client, ICM20948_BANK_0);
    i2c_smbus_write_byte_data(client, ICM20948_REG_PWR_MGMT_1,
                              ICM20948_PWR1_DEVICE_RESET);
    msleep(100);

    /* 리셋은 뱅크도 0 으로 되돌리지만, 리셋이 실제로 먹었는지 여기서는 알 수
     * 없으므로(위에서 반환값을 안 봤다) 명시적으로 다시 고른다. */
    ret = icm20948_set_bank(client, ICM20948_BANK_0);
    if (ret < 0) {
        pr_err("IMU Driver: BANK_SEL write failed (%d) — I2C 배선/주소 확인\n", ret);
        return ret;
    }

    /* 2) WHO_AM_I 확인. 배선이 MPU-6050 과 같고 주소도 0x68 로 같아서,
     *    엉뚱한 칩이 꽂혀도 여기까지는 아무 증상 없이 통과한다. */
    ret = i2c_smbus_read_byte_data(client, ICM20948_REG_WHO_AM_I);
    if (ret < 0) {
        pr_err("IMU Driver: WHO_AM_I read failed (%d)\n", ret);
        return ret;
    }
    if (ret != ICM20948_WHO_AM_I_VALUE) {
        pr_err("IMU Driver: WHO_AM_I = 0x%02x (기대 0x%02x) — ICM-20948 이 아니다"
               " (MPU-6050 이면 0x68 이 나온다)\n",
               (unsigned int)ret, (unsigned int)ICM20948_WHO_AM_I_VALUE);
        return -ENODEV;
    }

    /* 3) SLEEP 과 LP_EN 을 함께 해제. LP_EN 을 남기면 값이 띄엄띄엄 갱신돼
     *    "멈춘 것처럼" 보인다. */
    ret = i2c_smbus_write_byte_data(client, ICM20948_REG_PWR_MGMT_1,
                                    ICM20948_PWR1_RUN);
    if (ret < 0) {
        pr_err("IMU Driver: PWR_MGMT_1 write failed (%d), sensor may stay asleep\n", ret);
        return ret;
    }
    msleep(50);

    /* 4) 가속도/자이로 전원 인가 */
    ret = i2c_smbus_write_byte_data(client, ICM20948_REG_PWR_MGMT_2,
                                    ICM20948_PWR2_ALL_ON);
    if (ret < 0) {
        pr_err("IMU Driver: PWR_MGMT_2 write failed (%d)\n", ret);
        return ret;
    }

    /* 5) 가속도 레인지 ±2g + DLPF 5.7Hz (뱅크2) */
    ret = icm20948_set_bank(client, ICM20948_BANK_2);
    if (ret < 0) {
        pr_err("IMU Driver: BANK_SEL(2) write failed (%d)\n", ret);
        return ret;
    }
    ret = i2c_smbus_write_byte_data(client, ICM20948_REG_ACCEL_CONFIG,
                                    ICM20948_ACCEL_CFG_2G_5HZ);
    if (ret < 0) {
        pr_err("IMU Driver: ACCEL_CONFIG write failed (%d)\n", ret);
        /* 뱅크를 2 에 남긴 채 나가면 다음 read 가 엉뚱한 레지스터를 읽는다.
         * 실패해도 복구는 시도한다. */
        (void)icm20948_set_bank(client, ICM20948_BANK_0);
        return ret;
    }

    /* 6) 핵심: 반드시 뱅크0 으로 복귀. imu_read 가 뱅크0(0x2D)을 읽는다. */
    ret = icm20948_set_bank(client, ICM20948_BANK_0);
    if (ret < 0) {
        pr_err("IMU Driver: BANK_SEL(0) 복귀 실패 (%d)\n", ret);
        return ret;
    }

    msleep(50);   /* DLPF 가 채워질 시간 */
    return 0;
}

static int icm20948_probe(struct i2c_client *client)
{
    int ret;

    /* 핵심: 초기화가 **끝난 뒤에** imu_client 를 공개한다.
     *
     *   MPU-6050 판에서는 probe 진입 직후에 공개했는데, 그러면 리셋~웨이크
     *   사이 150ms 동안 read() 가 초기화 중인 칩을 읽는다. ICM-20948 에서는
     *   그 사이 **뱅크가 2 로 가 있는 구간**까지 있어서, 그때 들어온 read 는
     *   가속도가 아니라 설정 레지스터를 읽어 간다. 순서를 뒤집어 없앤다. */
    ret = icm20948_setup(client);
    if (ret < 0)
        return ret;

    mutex_lock(&imu_lock);
    imu_client = client;
    mutex_unlock(&imu_lock);

    pr_info("IMU Driver: ICM-20948 probed successfully! (±2g, DLPF 5.7Hz)\n");
    return 0;
}

static void icm20948_remove(struct i2c_client *client)
{
    (void)client;
    mutex_lock(&imu_lock);
    imu_client = NULL;
    mutex_unlock(&imu_lock);
    pr_info("IMU Driver: I2C Client Removed\n");
}

/* sysfs new_device 로 손수 인스턴스화할 때 쓰이는 이름 매칭.
 *   echo icm20948 0x69 > /sys/bus/i2c/devices/i2c-1/new_device
 * (주소는 0x69 — 위 헤더 주석 참조. i2cdetect -y 1 로 실측 확인할 것) */
static const struct i2c_device_id icm20948_id[] = {
    { "icm20948", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, icm20948_id);

/* Device Tree 매칭 (overlays/imu-overlay.dts).
 *
 * 핵심: 이게 있어야 오버레이가 선언한 imu@68 노드에 probe 가 붙는다. id_table
 *   만으로는 DT 노드와 매칭되지 않으므로, 오버레이를 올려도 아무 일이 안
 *   일어난다. 반대로 이것만 있고 오버레이가 없으면 붙을 노드가 없다 —
 *   **둘이 짝이다.**
 *
 * 주의: compatible 에 표준 문자열("invensense,icm20948")을 쓰지 않는다. 커널
 *   내장 inv-mpu6050 계열이 ICM-20948 도 물고 있어 어느 쪽이 바인딩될지
 *   경합한다. 아래 driver.name 을 고유하게 둔 것과 같은 이유다. */
static const struct of_device_id icm20948_of_match[] = {
    { .compatible = "adts,imu-icm20948" },
    { }
};
MODULE_DEVICE_TABLE(of, icm20948_of_match);

static struct i2c_driver icm20948_driver = {
    .driver = {
        /* 커널 내장 드라이버와 등록 이름이 겹치면 i2c_add_driver() 가
         * "Driver '...' is already registered" 로 실패한다. driver.name 은
         * id_table 매칭과 무관한 순수 등록용 식별자이므로 고유한 이름을 쓴다.
         * (echo icm20948 0x68 > new_device 로 디바이스 만드는 건 그대로 동작함) */
        .name           = "imu_icm20948_custom",
        .of_match_table = icm20948_of_match,
        .owner          = THIS_MODULE,
    },
    .probe    = icm20948_probe,
    .remove   = icm20948_remove,
    .id_table = icm20948_id,
};

static int __init imu_init(void)
{
    int ret;

    imu_major = register_chrdev(0, "imu", &imu_fops);
    if (imu_major < 0)
        return imu_major;

    imu_class = class_create("imu_class");
    if (IS_ERR(imu_class)) {
        unregister_chrdev(imu_major, "imu");
        return PTR_ERR(imu_class);
    }

    /* 주의: device_create() **전에** 달아야 한다. 노드는 device_create 시점에
     *   만들어지므로, 그 뒤에 콜백을 붙이면 이미 만들어진 노드의 권한은
     *   그대로 0600 으로 남는다. */
    imu_class->devnode = imu_devnode;

    imu_dev = device_create(imu_class, NULL, MKDEV(imu_major, 0), NULL, "imu");
    if (IS_ERR(imu_dev)) {
        class_destroy(imu_class);
        unregister_chrdev(imu_major, "imu");
        return PTR_ERR(imu_dev);
    }

    ret = i2c_add_driver(&icm20948_driver);
    if (ret < 0) {
        device_destroy(imu_class, MKDEV(imu_major, 0));
        class_destroy(imu_class);
        unregister_chrdev(imu_major, "imu");
        return ret;
    }

    pr_info("IMU Driver: Initialized & /dev/imu created\n");
    return 0;
}

static void __exit imu_exit(void)
{
    i2c_del_driver(&icm20948_driver);
    if (imu_dev) {
        device_destroy(imu_class, MKDEV(imu_major, 0));
        class_destroy(imu_class);
        unregister_chrdev(imu_major, "imu");
    }
}

module_init(imu_init);
module_exit(imu_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ICM-20948 Level Detection Driver");

//# 1. 기존 모듈 제거
//sudo rmmod imu_driver
//
//# 2. 모듈 재로드
//sudo insmod imu_driver.ko
//
//# 3-A. (권장) 오버레이로 붙이기 — 재부팅해도 유지된다
//sudo dtoverlay overlays/imu-overlay.dtbo
//
//# 3-B. 오버레이 없이 손으로 붙이기 (재부팅하면 사라짐)
//echo icm20948 0x69 | sudo tee /sys/bus/i2c/devices/i2c-1/new_device
//
//# 4. probe() 성공 로그 확인 — WHO_AM_I 불일치면 여기서 잡힌다
//dmesg | tail -n 10
//
//# 5. 칩 주소 실측 (이 보드는 0x69. 0x68 이면 AD0 가 L 인 보드다)
//i2cdetect -y 1
//
//# 주의: 순서 주의: DT 노드가 먼저 있어야 insmod 때 매칭된다. 오버레이를
//#   나중에 올리면 compatible 속성만 바뀔 뿐 **재매칭이 안 일어나** probe 가
//#   조용히 안 불린다. config.txt 로 부팅에 걸어두는 쪽이 확실하다.
//Roll: 좌우 기울기, Pitch: 상하 기울기
