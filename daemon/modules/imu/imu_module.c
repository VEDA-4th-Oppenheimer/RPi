/* ============================================================================
 *  imu_module.c  --  /dev/imu (MPU-6050) 수평 기준 제공
 *  담당: 송영빈 (드라이버 + 이 모듈)
 *
 *  MPU-6050(6축) 을 RPi I2C 에 직결하고 자작 캐릭터 드라이버 /dev/imu 로 노출.
 *  이 모듈은 가속도계 중력벡터를 읽어 roll/pitch 를 산출해 ctx->level 에 채운다.
 *
 *  ★ 역할 분리: "측정"은 이 모듈, "판정(수평 게이트)"은 코어가 한다.
 *    코어는 SCANNING 진입 시 ctx->level 을 보고 LEVEL_GATE_MAX_DEG 초과면 거부.
 *    (방식 A = 게이트. 좌표 회전보정 아님 — 수평을 보장한 뒤 스캔한다.)
 *
 *  ★ roll/pitch 원본은 버리지 말 것: 향후 방식 B(보정) 확장 시 그대로 재사용.
 *    (imu_test.c 의 'z' 영점 tare 같은 오프셋 보정은 여기서 하지 않는다 — 원본 유지.)
 *
 *  드라이버 read() 계약 (driver/imu_driver.c):
 *    read(fd, buf, 6) 이 성공하면 정확히 6바이트, MPU-6050 레지스터 0x3B
 *    (ACCEL_XOUT_H) 부터 순서대로 [ax_hi,ax_lo,ay_hi,ay_lo,az_hi,az_lo]
 *    (16-bit big-endian, ±2g 풀스케일 → 16384 LSB/g). 실패 시 <0 (예: 클라이언트
 *    없으면 -ENODEV, I2C 오류면 -EIO). read()는 offset을 쓰지 않는 스트림
 *    디바이스(no_llseek)라 매 호출이 최신 레지스터 값을 그대로 준다.
 *
 *  중력벡터 -> 각도 (imu_test.c 와 동일 공식):
 *      roll  = atan2(ay, az)                        * 180/PI
 *      pitch = atan2(-ax, sqrt(ay*ay + az*az))      * 180/PI
 *
 *  ※ 스캔 전 1회성 판정이라 실시간성이 필요 없다. I2C read 는 블로킹이므로
 *    on_tick(100ms) 마다 읽지 않고 저속(~1초)으로만 갱신한다(단일 스레드 보호).
 *    스캔 중(ST_SCANNING)에는 갱신 자체를 건너뛴다(원 설계 의도 그대로).
 * ==========================================================================*/
#include "daemon_module.h"

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#define IMU_DEV "/dev/imu"

/* on_tick 은 100ms 주기다. 10 tick = 약 1초 간격으로만 실제 I2C read 수행. */
#define IMU_READ_EVERY_TICKS 10u
#define IMU_ACCEL_SCALE_LSB_PER_G 16384.0f   /* ±2g 풀스케일 (드라이버 기본 설정) */

static int      g_fd = -1;
static unsigned g_tick_count = 0u;

static int imu_init(struct shared_ctx *ctx)
{
    ctx->level.valid = 0u;      /* 아직 측정 없음 (코어가 게이트 생략 로그) */

    g_fd = open(IMU_DEV, O_RDONLY);
    if (g_fd < 0) {
        /* 드라이버가 없을 때 데몬 전체가 죽으면 개발이 막히므로 degraded 로 계속. */
        (void)fprintf(stderr, "[imu     ] open %s 실패 (%s) — degraded 로 계속\n",
                      IMU_DEV, strerror(errno));
        return 0;
    }

    (void)fprintf(stderr, "[imu     ] init 완료 (%s)\n", IMU_DEV);
    return 0;
}

static int imu_get_fd(void)
{
    return -1;   /* 폴링 방식(주기 read). 인터럽트 쓰면 여기서 fd 노출 */
}

static void imu_read_once(struct shared_ctx *ctx)
{
    uint8_t buf[6];
    const ssize_t n = read(g_fd, buf, sizeof(buf));
    if (n != (ssize_t)sizeof(buf)) {
        (void)fprintf(stderr, "[imu     ] read 실패 (%s) — 이전 값 유지\n", strerror(errno));
        return;   /* 일시 오류로 valid 를 꺼버리지 않는다 — 마지막 유효값 유지 */
    }

    const int16_t raw_ax = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
    const int16_t raw_ay = (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);
    const int16_t raw_az = (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);

    const float ax = (float)raw_ax / IMU_ACCEL_SCALE_LSB_PER_G;
    const float ay = (float)raw_ay / IMU_ACCEL_SCALE_LSB_PER_G;
    const float az = (float)raw_az / IMU_ACCEL_SCALE_LSB_PER_G;

    ctx->level.roll_deg  = atan2f(ay, az) * 180.0f / (float)M_PI;
    ctx->level.pitch_deg = atan2f(-ax, sqrtf((ay * ay) + (az * az))) * 180.0f / (float)M_PI;
    ctx->level.valid     = 1u;
}

static void imu_on_tick(struct shared_ctx *ctx, daemon_state_t state)
{
    ++g_tick_count;
    if (g_fd < 0) {
        return;                              /* degraded: 드라이버 없음 */
    }
    if (state == ST_SCANNING) {
        return;                              /* 스캔 중엔 갱신 불필요 */
    }
    if ((g_tick_count % IMU_READ_EVERY_TICKS) != 0u) {
        return;                              /* ~1초 간격으로만 실제 read */
    }
    imu_read_once(ctx);
}

static void imu_deinit(struct shared_ctx *ctx)
{
    (void)ctx;
    if (g_fd >= 0) {
        (void)close(g_fd);
        g_fd = -1;
    }
}

static const struct daemon_module k_imu = {
    "imu",
    imu_init,
    imu_get_fd,
    NULL,          /* on_event  (fd 없음) */
    imu_on_tick,
    NULL,          /* on_state */
    imu_deinit,
};

const struct daemon_module *imu_module_get(void)
{
    return &k_imu;
}
