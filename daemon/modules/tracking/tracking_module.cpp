/* ============================================================================
 *  tracking_module.cpp  --  추적/융합 모듈  [STUB]
 *  담당: 이현우(칼만 예측 + 기하변환) + 송영빈(비주얼 서보잉)
 *
 *  계산형 모듈(fd 없음). 매 tick 에서:
 *   - 송영빈: pixerr(카메라 오차)로 비주얼 서보잉 → core_aim() 으로 조준각 갱신
 *   - 이현우: aim(현재각) + range(거리) + 칼만 예측 → 3D 융합좌표 ctx->fused
 *  ctx->range 는 코어가 /dev/turret CMD_DISTANCE 로 채워준다.
 * ==========================================================================*/
#include "daemon_module.h"
#include <cstdio>

namespace {

int tracking_init(shared_ctx *ctx)
{
    ctx->fused.valid = 0u;
    (void)std::fprintf(stderr, "[tracking] init (STUB — 이현우:칼만/기하, 송영빈:서보잉)\n");
    return 0;
}

void tracking_on_tick(shared_ctx *ctx, daemon_state_t state)
{
    (void)ctx;
    (void)state;
    /* TODO(송영빈): TRACK 상태에서 pixerr → 비주얼 서보잉 →
     *   core_aim(ctx->core, theta_ddeg, phi_ddeg)
     * TODO(이현우): LOCK_ON 에서 aim+range+칼만 예측 → ctx->fused.{x,y,z}, valid=1 */
}

const daemon_module k_tracking = {
    "tracking",
    tracking_init,
    nullptr,            /* get_fd (계산형, fd 없음) */
    nullptr,            /* on_event */
    tracking_on_tick,
    nullptr,            /* on_state */
    nullptr,            /* deinit   */
};

}  /* namespace */

extern "C" const daemon_module *tracking_module_get(void)
{
    return &k_tracking;
}
