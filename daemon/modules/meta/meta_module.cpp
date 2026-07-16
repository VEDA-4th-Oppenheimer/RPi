/* ============================================================================
 *  meta_module.cpp  --  카메라 AI 메타데이터 모듈  [STUB]
 *  담당: 이영민
 *
 *  한화 카메라 내부 AI 가 드론을 탐지 → bbox 중심을 화면중심 대비 픽셀오차
 *  (dx, dy) + 탐지 채널(0..3) 로 shared_ctx.pixerr 에 채운다.
 *  수신 경로(SUNAPI/ONVIF 이벤트 or MQTT)는 이영민이 구현.
 * ==========================================================================*/
#include "daemon_module.h"
#include <cstdio>

namespace {

int meta_init(shared_ctx *ctx)
{
    ctx->pixerr.valid   = 0u;
    ctx->pixerr.channel = 0u;
    (void)std::fprintf(stderr, "[meta    ] init (STUB — 이영민: 카메라 AI bbox → dx,dy,channel)\n");
    return 0;
}

int meta_get_fd()
{
    return -1;   /* TODO(이영민): SUNAPI/ONVIF 이벤트 소켓 또는 MQTT fd 반환 */
}

void meta_on_tick(shared_ctx *ctx, daemon_state_t state)
{
    (void)ctx;
    (void)state;
    /* TODO(이영민): 카메라 메타데이터 수신 →
     *   ctx->pixerr.{dx,dy,channel} 갱신, valid=1
     *   (fd 방식이면 get_fd/on_event 로, 폴링이면 여기서) */
}

const daemon_module k_meta = {
    "meta",
    meta_init,
    meta_get_fd,
    nullptr,        /* on_event */
    meta_on_tick,
    nullptr,        /* on_state */
    nullptr,        /* deinit   */
};

}  /* namespace */

extern "C" const daemon_module *meta_module_get(void)
{
    return &k_meta;
}
