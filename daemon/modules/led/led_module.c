/* ============================================================================
 *  led_module.c  --  /dev/led (상태 LED x3 + 액티브 부저 x1)  [STUB]
 *  담당: 이현우
 *
 *  RPi GPIO 직결 자작 캐릭터 드라이버 /dev/led 를 통해 킷 상태를 표시한다.
 *  드라이버는 "얇게" — 핀 on/off 만. 점멸·비프 패턴의 타이밍은 이 모듈이 만든다.
 *
 *  ★ 부저는 액티브(단일 톤 on/off)라 이벤트 구분을 "비프 패턴"으로 한다.
 *    (패시브+PWM 별도 /dev/buzzer 는 과설계라 기각 — /dev/led 에 채널 편입)
 *
 *   상태          | LED        | 부저
 *   --------------|------------|---------------------
 *   수평 NG       | 빨강       | 길게 1회 (경고)
 *   SCANNING      | 진행 점멸  | 무음
 *   완료(EXPORT)  | 초록       | 짧게 2회
 *   에러(DISARM)  | 빨강       | 짧게 3회
 *
 *  ※ 수평 NG 는 FSM 상태가 아니라 "SCANNING 진입 거부"로 나타난다.
 *    코어가 게이트에서 막으면 IDLE 에 머무르므로, 그 표시는 코어 로그/ctx->level
 *    을 보고 이 모듈이 판단한다(TODO).
 * ==========================================================================*/
#include "daemon_module.h"
#include <stdio.h>

/* #define LED_DEV "/dev/led"
 * ioctl 규약(예정): LED_SET { ch_id, on }  — ch: 0=green,1=red,2=busy,3=buzzer */

static int led_init(struct shared_ctx *ctx)
{
    (void)ctx;
    /* TODO: open(LED_DEV) — 없으면 degraded 로 계속(0 반환) */
    (void)fprintf(stderr,
        "[led     ] init (STUB — /dev/led 미구현)\n");
    return 0;
}

static int led_get_fd(void)
{
    return -1;   /* 출력 전용. epoll 대상 아님 */
}

/* cppcheck-suppress constParameterCallback ; on_tick 은 daemon_module 콜백 ABI
 * (void(*)(struct shared_ctx*, daemon_state_t)) 라 ctx 를 const 로 못 바꾼다. */
static void led_on_tick(struct shared_ctx *ctx, daemon_state_t state)
{
    (void)ctx;
    (void)state;
    /* TODO: 100ms tick 을 세어 패턴 생성
     *   - ST_SCANNING : busy LED 를 500ms 주기로 토글(진행 표시)
     *   - 비프 패턴   : 남은 비프 수/길이를 카운터로 관리해 tick 마다 on/off
     *   블로킹 sleep 금지 (단일 스레드 루프) */
}

/* cppcheck-suppress constParameterCallback ; 위와 동일(콜백 ABI 고정) */
static void led_on_state(struct shared_ctx *ctx,
                         daemon_state_t old_st, daemon_state_t new_st)
{
    (void)ctx;
    (void)old_st;
    (void)new_st;
    /* TODO: 상태 전이 시 LED 색 전환 + 비프 패턴 예약
     *   -> ST_SCANNING : busy 점멸 시작, 무음
     *   -> ST_EXPORT   : 초록 on, 짧게 2회
     *   -> ST_DISARM   : 빨강 on, 짧게 3회
     *   -> ST_IDLE     : 소등(또는 대기 표시) */
}

static void led_deinit(struct shared_ctx *ctx)
{
    (void)ctx;
    /* TODO: 전 채널 off 후 close(LED_DEV) — 종료 시 부저가 켜진 채 남지 않게 */
}

static const struct daemon_module k_led = {
    "led",
    led_init,
    led_get_fd,
    NULL,          /* on_event (fd 없음) */
    led_on_tick,
    led_on_state,
    led_deinit,
};

const struct daemon_module *led_module_get(void)
{
    return &k_led;
}
