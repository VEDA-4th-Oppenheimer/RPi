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

    /* 임시 디버그: raw byte가 매 호출마다 실제로 바뀌는지 확인용.
     * 문제 해결 후에는 제거하거나 pr_debug로 낮춰서 평소엔 안 찍히게 하세요. */
    pr_info("IMU Driver: raw = %02x %02x %02x %02x %02x %02x\n",
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

static const struct i2c_device_id mpu6050_id[] = {
    { "mpu6050", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, mpu6050_id);

static struct i2c_driver mpu6050_driver = {
    .driver = {
        /* 커널 내장 inv-mpu6050-i2c 드라이버도 "mpu6050"이라는 이름으로
         * 등록하기 때문에, 그 모듈이 로드되어 있으면 i2c_add_driver()가
         * "Driver 'mpu6050' is already registered" 로 실패한다.
         * driver.name은 id_table 매칭과 무관한 순수 등록용 식별자이므로
         * 고유한 이름으로 바꿔서 충돌 자체를 없앤다.
         * (echo mpu6050 0x68 > new_device 로 디바이스 만드는 건 그대로 동작함) */
        .name  = "imu_mpu6050_custom",
        .owner = THIS_MODULE,
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