/* ============================================================================
 *  imu_module.c  --  /dev/imu (MPU-6050) 수평 기준 제공  [STUB]
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
 *
 *  중력벡터 -> 각도 (가속도계 ax,ay,az):
 *      roll  = atan2(ay, az)                        * 180/PI
 *      pitch = atan2(-ax, sqrt(ay*ay + az*az))      * 180/PI
 *
 *  ※ 스캔 전 1회성 판정이라 실시간성이 필요 없다. 다만 I2C read 는 블로킹이므로
 *    on_tick 에서 매번 읽지 말고 저속(예: 1초)으로 갱신한다(단일 스레드 보호).
 * ==========================================================================*/
#include "daemon_module.h"
#include <stdio.h>

/* #define IMU_DEV "/dev/imu" */

static int imu_init(struct shared_ctx *ctx)
{
    ctx->level.valid = 0u;      /* 아직 측정 없음 (코어가 게이트 생략 로그) */
    /* TODO(송영빈): open(IMU_DEV) — 없으면 degraded 로 계속(0 반환)
     *   드라이버가 없을 때 데몬 전체가 죽으면 개발이 막히므로 실패해도 0 반환. */
    (void)fprintf(stderr,
        "[imu     ] init (STUB — /dev/imu 미구현. 송영빈)\n");
    return 0;
}

static int imu_get_fd(void)
{
    return -1;   /* 폴링 방식(주기 read). 인터럽트 쓰면 여기서 fd 노출 */
}

/* cppcheck-suppress constParameterCallback ; on_tick 은 daemon_module 콜백 ABI
 * (void(*)(struct shared_ctx*, daemon_state_t)) 라 ctx 를 const 로 못 바꾼다. */
static void imu_on_tick(struct shared_ctx *ctx, daemon_state_t state)
{
    (void)ctx;
    (void)state;
    /* TODO(송영빈): 저속(1초) 주기로 /dev/imu read -> ax,ay,az
     *   -> roll/pitch 산출 -> ctx->level.{roll_deg,pitch_deg}, valid=1
     *   ⚠️ I2C read 블로킹 최소화. 스캔 중(ST_SCANNING)에는 갱신 불필요. */
}

static void imu_deinit(struct shared_ctx *ctx)
{
    (void)ctx;
    /* TODO(송영빈): close(/dev/imu) */
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
