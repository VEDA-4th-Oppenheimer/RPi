/* ============================================================================
 *  imu_module.c  --  /dev/imu (ICM-20948) 수평 기준 제공
 *  담당: 송영빈 (커널 드라이버 + 참조 구현) / 이현우 (데몬 모듈 연동)
 *
 *  ICM-20948(9축) 을 RPi I2C 에 직결하고 자작 캐릭터 드라이버 /dev/imu 로 노출.
 *  이 모듈은 가속도계 중력벡터를 읽어 roll/pitch 를 산출해 ctx->level 에 채운다.
 *
 *  ⚠️ MPU-6050 에서 교체됐지만 **이 파일의 코드는 바뀌지 않았다.** 드라이버가
 *    /dev/imu 계약(6바이트, 빅엔디안, ±2g)을 그대로 유지하기 때문이다.
 *    자력계는 안 쓴다 — MT6701 이 자기식 엔코더라 축마다 자석이 붙어 있고
 *    스텝모터 코일도 바로 옆이라 방위 측정이 의미가 없다.
 *
 *  ★ 역할 분리: "측정"은 이 모듈, "판정(수평 게이트)"은 코어가 한다.
 *    코어 level_gate_ok() 가 SCANNING 진입 시 ctx->level 을 보고
 *    LEVEL_GATE_MAX_DEG(1.5도) 초과면 스캔을 거부한다.
 *    (방식 A = 게이트. 좌표 회전보정 아님 — 수평을 보장한 뒤 스캔한다.)
 *
 *  ★ roll/pitch 원본은 버리지 않는다: 향후 방식 B(보정) 확장 시 그대로 재사용.
 *    ctx->level 은 mqtt_module 이 adts/state/daemon 의 "level" 블록으로
 *    그대로 발행하므로 Qt 관제에서도 실시간으로 보인다.
 *
 *  ── /dev/imu 계약 (driver/imu_driver.c) ──────────────────────────────────
 *    read(fd, buf, 6) 이 ICM-20948 뱅크0 레지스터 0x2D(ACCEL_XOUT_H) 부터
 *    6바이트를 **원본 그대로** 준다. 즉 ax/ay/az 각각 **빅엔디안 int16** 이다.
 *      buf[0..1] = ax,  buf[2..3] = ay,  buf[4..5] = az
 *    len < 6 이면 -EINVAL, 드라이버는 떴는데 칩이 없으면 -ENODEV/-EIO.
 *
 *  ── 중력벡터 -> 각도 (driver/imu_test.c 와 동일한 식) ────────────────────
 *      roll  = atan2(ay, az)
 *      pitch = atan2(-ax, sqrt(ay^2 + az^2))
 *
 *    스케일 16384 LSB/g (±2g) 도 imu_test.c 를 그대로 따른다. ICM-20948 도
 *    드라이버가 ACCEL_FS_SEL=±2g 로 잡으므로 MPU-6050 과 같은 값이다.
 *    ⚠️ 사실 atan2 는 비율만 쓰므로 스케일이 결과를 바꾸지 않는다. 그래도
 *      맞춰두는 이유는 같은 장비를 두 프로그램이 읽을 때 숫자가 어긋나면
 *      "어느 쪽이 맞나" 를 매번 따지게 되기 때문이다.
 *
 *  ⚠️ 축 방향은 **실측으로 확인해야 한다.** 칩이 바뀌면서 패키지 축 방향이나
 *    브레이크아웃 실장 방향이 달라졌을 수 있어, 아래 roll/pitch 식의 부호가
 *    뒤집히거나 두 축이 서로 바뀔 수 있다. 배선만 같다고 같은 각도가 나오지
 *    않는다 — imu_test 로 앞뒤/좌우로 기울여 확인할 것.
 *
 *  ⚠️ read() 는 블로킹 I2C 다. 스캔 전 1회성 판정이라 실시간성이 없으므로
 *    저속(1초)으로만 갱신하고, 스캔 중에는 아예 읽지 않는다 — 단일스레드
 *    epoll 루프가 I2C 왕복만큼 멈추면 라이다 점 배치 수신이 밀린다.
 * ==========================================================================*/
#include "daemon_module.h"

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define IMU_DEV            "/dev/imu"
#define IMU_SAMPLE_BYTES   6
#define IMU_PERIOD_MS      1000u    /* 사람이 수평을 맞추는 속도면 1Hz 로 충분 */
#define IMU_ERR_LOG_EVERY  10u      /* 연속 실패 로그 도배 방지 */
/* ±2g. 드라이버가 ICM20948_ACCEL_CFG_2G_5HZ 로 잡는 값과 짝이다 —
 * 드라이버에서 레인지를 바꾸면 여기도 반드시 같이 고칠 것. */
#define IMU_ACCEL_LSB_PER_G 16384.0f

static int      s_fd = -1;
static uint64_t s_last_ms;
static uint32_t s_err_run;
static uint8_t  s_first_report;      /* 0=아직, 1=정상 보고함, 2=이상 보고함 */

static uint64_t imu_mono_ms(void)
{
    struct timespec ts;
    (void)clock_gettime(CLOCK_MONOTONIC, &ts);
    return ((uint64_t)ts.tv_sec * 1000u) + ((uint64_t)ts.tv_nsec / 1000000u);
}

/* 빅엔디안 int16 복원. ICM-20948 도 MPU-6050 과 같이 상위 바이트가 먼저다. */
static int16_t imu_be16(const unsigned char *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static int imu_init(struct shared_ctx *ctx)
{
    ctx->level.valid     = 0u;
    ctx->level.roll_deg  = 0.0f;
    ctx->level.pitch_deg = 0.0f;

    /* ⚠️ 열기 실패해도 0(성공) 을 반환한다. IMU 가 없다고 데몬 전체가 죽으면
     *   개발이 막히고, 코어는 level.valid==0 을 "게이트 생략" 으로 이미
     *   처리한다(그때 경고 로그가 남는다). */
    s_fd = open(IMU_DEV, O_RDONLY | O_CLOEXEC);
    if (s_fd < 0) {
        (void)fprintf(stderr,
            "[imu     ] %s 열기 실패(%s) — 수평 게이트 없이 진행\n",
            IMU_DEV, strerror(errno));
    } else {
        (void)fprintf(stderr, "[imu     ] init (%s, %ums 주기)\n",
                      IMU_DEV, IMU_PERIOD_MS);
    }
    s_last_ms      = 0u;
    s_err_run      = 0u;
    s_first_report = 0u;
    return 0;
}

static int imu_get_fd(void)
{
    /* 폴링이다. fd 를 epoll 에 넣지 않는다 — 드라이버가 poll() 을 구현하지
     * 않고, 1Hz 주기 판독이라 이벤트 구동이 필요 없다. */
    return -1;
}

/* cppcheck-suppress constParameterCallback ; on_tick 은 daemon_module 콜백 ABI
 * (void(*)(struct shared_ctx*, daemon_state_t)) 라 ctx 를 const 로 못 바꾼다. */
static void imu_on_tick(struct shared_ctx *ctx, daemon_state_t state)
{
    unsigned char buf[IMU_SAMPLE_BYTES];
    ssize_t       n;
    uint64_t      now;

    /* 스캔 중에는 읽지 않는다 — 판정은 진입 시점에 이미 끝났고, 블로킹 I2C 가
     * 점 수신 루프를 방해할 이유가 없다. */
    if ((s_fd < 0) || (state == ST_SCANNING)) {
        return;
    }

    now = imu_mono_ms();
    if ((now - s_last_ms) < (uint64_t)IMU_PERIOD_MS) {
        return;
    }
    s_last_ms = now;

    n = read(s_fd, buf, sizeof(buf));
    if (n != (ssize_t)IMU_SAMPLE_BYTES) {
        /* ⚠️ 실패하면 valid 를 내린다. 낡은 값을 남겨두면 센서가 빠진 뒤에도
         *   "수평 OK" 로 스캔이 통과해 게이트의 존재 이유가 사라진다. */
        ctx->level.valid = 0u;
        s_err_run++;
        if ((s_err_run % IMU_ERR_LOG_EVERY) == 1u) {
            (void)fprintf(stderr, "[imu     ] read 실패(%s) 연속 %u회\n",
                          (n < 0) ? strerror(errno) : "짧은 read", s_err_run);
        }
        return;
    }
    s_err_run = 0u;

    {
        const int16_t raw_ax = imu_be16(&buf[0]);
        const int16_t raw_ay = imu_be16(&buf[2]);
        const int16_t raw_az = imu_be16(&buf[4]);

        /* 3축이 전부 0 이면 칩이 응답만 하고 값을 안 주는 상태다(슬립 해제
         * 실패 등). atan2(0,0) 은 0 을 돌려주므로 그대로 두면 "완벽한 수평"
         * 으로 보여 게이트를 무사통과한다. */
        if ((raw_ax == 0) && (raw_ay == 0) && (raw_az == 0)) {
            ctx->level.valid = 0u;
            if (s_first_report == 0u) {
                (void)fprintf(stderr,
                    "[imu     ] 가속도 3축이 모두 0 — 칩 초기화 확인"
                    " (dmesg 에서 WHO_AM_I / PWR_MGMT_1 / BANK_SEL 로그)\n");
                s_first_report = 2u;
            }
            return;
        }

        {
            const float ax = (float)raw_ax / IMU_ACCEL_LSB_PER_G;
            const float ay = (float)raw_ay / IMU_ACCEL_LSB_PER_G;
            const float az = (float)raw_az / IMU_ACCEL_LSB_PER_G;

            ctx->level.roll_deg  = atan2f(ay, az) * 180.0f / (float)M_PI;
            ctx->level.pitch_deg = atan2f(-ax, sqrtf((ay * ay) + (az * az)))
                                 * 180.0f / (float)M_PI;
            ctx->level.valid     = 1u;

            if (s_first_report == 0u) {
                (void)fprintf(stderr,
                    "[imu     ] 첫 측정 roll=%.2f pitch=%.2f (임계 %.1f)\n",
                    (double)ctx->level.roll_deg,
                    (double)ctx->level.pitch_deg,
                    (double)LEVEL_GATE_MAX_DEG);
                s_first_report = 1u;
            }
        }
    }
}

static void imu_deinit(struct shared_ctx *ctx)
{
    ctx->level.valid = 0u;
    if (s_fd >= 0) {
        (void)close(s_fd);
        s_fd = -1;
    }
}

static const struct daemon_module k_imu = {
    "imu",
    imu_init,
    imu_get_fd,
    NULL,          /* on_event  (fd 없음 — 폴링) */
    imu_on_tick,
    NULL,          /* on_state */
    imu_deinit,
};

const struct daemon_module *imu_module_get(void)
{
    return &k_imu;
}
