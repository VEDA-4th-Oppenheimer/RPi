/* ============================================================================
 *  tls_module.cpp  --  Qt 관제 TLS push 모듈  [STUB]
 *  담당: 팀장(이광진)
 *
 *  융합 결과 ctx->fused (x,y,z) 를 OpenSSL TLS 소켓으로 Qt 관제(레이더 스코프)
 *  에 push. 연결되면 get_fd 로 소켓 fd 를 노출해 수신/keepalive 도 처리.
 * ==========================================================================*/
#include "daemon_module.h"
#include <cstdio>

namespace {

int tls_init(shared_ctx *ctx)
{
    (void)ctx;
    (void)std::fprintf(stderr, "[tls     ] init (STUB — 팀장: 융합 x,y,z → Qt TLS push)\n");
    return 0;
}

int tls_get_fd()
{
    return -1;   /* TODO(팀장): Qt 관제 TLS 소켓 fd (연결 수립 후) */
}

void tls_on_tick(shared_ctx *ctx, daemon_state_t state)
{
    (void)state;
    if (ctx->fused.valid != 0u) {
        /* TODO(팀장): OpenSSL TLS 로 (x,y,z) Qt 관제에 push */
    }
}

const daemon_module k_tls = {
    "tls",
    tls_init,
    tls_get_fd,
    nullptr,        /* on_event */
    tls_on_tick,
    nullptr,        /* on_state */
    nullptr,        /* deinit   */
};

}  /* namespace */

extern "C" const daemon_module *tls_module_get(void)
{
    return &k_tls;
}
