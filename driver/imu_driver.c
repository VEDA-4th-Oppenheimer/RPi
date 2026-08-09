/*
 * imu_driver.c — MPU-6050 I2C 캐릭터 디바이스 드라이버
 *
 * 수정 사항 (원본 대비):
 *  1. imu_client 접근을 mutex로 보호 (probe/remove와의 레이스 컨디션 방지)
 *  2. offset을 조작하는 대신 .llseek = no_llseek 으로 seek 불가 스트림 디바이스로 명시
 *  3. PWR_MGMT_1 write 반환값 체크 (Sleep 모드 해제 실패 감지)
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

#define MPU6050_REG_PWR_MGMT_1    0x6B
#define MPU6050_REG_ACCEL_XOUT_H  0x3B

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
    return i2c_transfer(client->adapter, msgs, 2);
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

    ret = i2c_read_bytes(client, MPU6050_REG_ACCEL_XOUT_H, raw_buf, 6);
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

    /* offset은 건드리지 않는다 (no_llseek 로 seek 자체를 막아 의미를 명확히 함) */
    return 6;
}

static struct file_operations imu_fops = {
    .owner   = THIS_MODULE,
    .read    = imu_read,
    .llseek  = noop_llseek, /* 스트림 디바이스: seek 무의미 (커널 6.x에서 no_llseek 제거됨) */
};

/* /dev/imu 노드 권한. devtmpfs 가 노드를 만들 때 이 콜백으로 모드를 묻는다.
 *
 * ⚠️ turret 은 miscdevice 라 구조체의 .mode 필드 하나로 끝나지만, 여기는
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

static int mpu6050_probe(struct i2c_client *client)
{
    int ret;

    mutex_lock(&imu_lock);
    imu_client = client;
    mutex_unlock(&imu_lock);

    /* 1) DEVICE_RESET (bit7=1): 이전 세션(커널 내장 드라이버 등)이 FIFO나
     *    다른 설정을 걸어놓았을 가능성을 배제하기 위해 레지스터를 완전히
     *    초기화한다. 리셋 중에는 응답이 없을 수 있어 반환값은 체크하지 않고
     *    충분히 대기만 한다. */
    i2c_smbus_write_byte_data(client, MPU6050_REG_PWR_MGMT_1, 0x80);
    msleep(100);

    /* 2) Sleep 모드 해제 — 반환값 반드시 체크 */
    ret = i2c_smbus_write_byte_data(client, MPU6050_REG_PWR_MGMT_1, 0x00);
    if (ret < 0) {
        pr_err("IMU Driver: PWR_MGMT_1 write failed (%d), sensor may stay asleep\n", ret);
        mutex_lock(&imu_lock);
        imu_client = NULL;
        mutex_unlock(&imu_lock);
        return ret;
    }
    msleep(50);

    pr_info("IMU Driver: MPU-6050 probed successfully!\n");
    return 0;
}

static void mpu6050_remove(struct i2c_client *client)
{
    mutex_lock(&imu_lock);
    imu_client = NULL;
    mutex_unlock(&imu_lock);
    pr_info("IMU Driver: I2C Client Removed\n");
}

/* sysfs new_device 로 손수 인스턴스화할 때 쓰이는 이름 매칭.
 *   echo mpu6050 0x68 > /sys/bus/i2c/devices/i2c-1/new_device */
static const struct i2c_device_id mpu6050_id[] = {
    { "mpu6050", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, mpu6050_id);

/* Device Tree 매칭 (overlays/imu-overlay.dts).
 *
 * ★ 이게 있어야 오버레이가 선언한 imu@68 노드에 probe 가 붙는다. id_table
 *   만으로는 DT 노드와 매칭되지 않으므로, 오버레이를 올려도 아무 일이 안
 *   일어난다. 반대로 이것만 있고 오버레이가 없으면 붙을 노드가 없다 —
 *   **둘이 짝이다.**
 *
 * ⚠️ compatible 에 표준 문자열("invensense,mpu6050")을 쓰지 않는다. 커널
 *   내장 inv-mpu6050-i2c 가 같은 문자열을 물고 있어 어느 쪽이 바인딩될지
 *   경합한다. 아래 driver.name 을 고유하게 둔 것과 같은 이유다. */
static const struct of_device_id mpu6050_of_match[] = {
    { .compatible = "adts,imu-mpu6050" },
    { }
};
MODULE_DEVICE_TABLE(of, mpu6050_of_match);

static struct i2c_driver mpu6050_driver = {
    .driver = {
        /* 커널 내장 inv-mpu6050-i2c 드라이버도 "mpu6050"이라는 이름으로
         * 등록하기 때문에, 그 모듈이 로드되어 있으면 i2c_add_driver()가
         * "Driver 'mpu6050' is already registered" 로 실패한다.
         * driver.name은 id_table 매칭과 무관한 순수 등록용 식별자이므로
         * 고유한 이름으로 바꿔서 충돌 자체를 없앤다.
         * (echo mpu6050 0x68 > new_device 로 디바이스 만드는 건 그대로 동작함) */
        .name           = "imu_mpu6050_custom",
        .of_match_table = mpu6050_of_match,
        .owner          = THIS_MODULE,
    },
    .probe    = mpu6050_probe,
    .remove   = mpu6050_remove,
    .id_table = mpu6050_id,
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

    /* ⚠️ device_create() **전에** 달아야 한다. 노드는 device_create 시점에
     *   만들어지므로, 그 뒤에 콜백을 붙이면 이미 만들어진 노드의 권한은
     *   그대로 0600 으로 남는다. */
    imu_class->devnode = imu_devnode;

    imu_dev = device_create(imu_class, NULL, MKDEV(imu_major, 0), NULL, "imu");
    if (IS_ERR(imu_dev)) {
        class_destroy(imu_class);
        unregister_chrdev(imu_major, "imu");
        return PTR_ERR(imu_dev);
    }

    ret = i2c_add_driver(&mpu6050_driver);
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
    i2c_del_driver(&mpu6050_driver);
    if (imu_dev) {
        device_destroy(imu_class, MKDEV(imu_major, 0));
        class_destroy(imu_class);
        unregister_chrdev(imu_major, "imu");
    }
}

module_init(imu_init);
module_exit(imu_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MPU-6050 Level Detection Driver Fix");

//# 1. 기존 모듈 제거
//sudo rmmod imu_driver
//
//# 2. 모듈 재로드
//sudo insmod imu_driver.ko
//
//# 3. I2C 1번 버스의 0x68 주소에 mpu6050 장치를 드라이버와 매칭 (핵심!)
//echo mpu6050 0x68 | sudo tee /sys/bus/i2c/devices/i2c-1/new_device
//
//# 4. probe() 성공 로그 찍혔는지 커널 로그 확인
//dmesg | tail -n 10
//Roll: 좌우 기울기, Pitch: 상하 기울기