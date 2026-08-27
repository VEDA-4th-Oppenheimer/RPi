/* ============================================================================
 *  led_module.c  --  /dev/led_sw (상태 LED x3 + 스위치 x2 통합 연동)
 *  담당: 강유근
 *
 *  RPi GPIO 직결 캐릭터 드라이버 /dev/led_sw 를 통해 킷 상태를 표시하고
 *  스위치 입력에 대응하여 프로토콜 동작을 트리거한다.
 *
 *  하드웨어 구성 및 프로토콜 점등 규칙:
 *   상태 / 프로토콜          | LED_초록 | LED_노랑 | LED_빨강 | 설명
 *   -----------------------|----------|----------|----------|--------------------------------
 *   ST_IDLE (명령 대기)     | OFF      | ON       | OFF      | 명령 대기 중 (정상 상태)
 *   ST_SCANNING (제어 동작) | ON       | OFF      | OFF      | 명령코드 중 제어코드 동작 중
 *   ST_EXPORT (처리 마무리) | ON       | OFF      | OFF      | 데이터 파일 출력 중
 *   ST_DISARM / 에러 발생   | OFF      | OFF      | ON       | 에러(코드) 발생 / 비상 정지
 *
 *  스위치 연동:
 *   - 스위치_scan_start (SW_SCAN_START) : 스캔 시작 요청 (CMD_SCAN_START)
 *   - 스위치_ems (SW_EMS)               : 즉시 안전정지 요청 (CMD_DISARM)
 * ==========================================================================*/

#include "daemon_module.h"
#include "protocol.h"
#include "led_sw.h"

#include <errno.h>
#include <stdbool.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>

static int s_fd = -1;
static struct led_sw_ctrl s_last_ctrl = {0, 0, 0, 0};

enum buzzer_seq {
    BUZ_NONE = 0,
    BUZ_SCAN_DONE,
    BUZ_ERROR
};
static enum buzzer_seq s_buz_seq = BUZ_NONE;
static int s_buz_ticks = 0;
static uint8_t s_last_err = 0; /* 0 = ERR_NONE */

/* advance = true 면 부저 시퀀스의 시계를 한 칸 돌린다.
 *
 * ⚠️ 이 함수는 on_tick(100ms 주기)과 on_state(상태 전이) 양쪽에서 불린다.
 *   예전에는 둘 다 시계를 돌려서, 비프가 울리는 도중에 FSM 이 전이하면 그
 *   전이가 틱 하나를 대신 먹었다. 0.5초 울려야 할 완료음이 0.4초가 되고,
 *   전이가 몰리면 더 짧아진다. 시계는 **틱만** 돌리고, 전이는 지금 상태를
 *   즉시 반영하기만 한다. */
static void update_leds_buzzer(const struct shared_ctx *ctx, bool advance)
{
    struct led_sw_ctrl ctrl = {0, 0, 0, 0};

    if (s_fd < 0) {
        return;
    }

    /* 1. LED 상태 결정 */
    if (ctx->state == ST_DISARM || !ctx->link.link_alive) {
        ctrl.red = 1u;
    } else if (ctx->state == ST_SCANNING || ctx->state == ST_EXPORT || ctx->link.scanning) {
        ctrl.green = 1u;
    } else {
        ctrl.yellow = 1u;
    }

    /* 2. 부저 시퀀스 로직 (1 tick = 100ms) */
    if (s_buz_seq == BUZ_SCAN_DONE) {
        /* 정상 종료 알림음: 1.0초 1번 = 10 ticks ON */
        if (s_buz_ticks < 10) {
            ctrl.buzzer = 1u;
            if (advance) {
                s_buz_ticks++;
            }
        } else {
            ctrl.buzzer = 0u;
            s_buz_seq = BUZ_NONE;
        }
    } else if (s_buz_seq == BUZ_ERROR) {
        /* 0.2초 간격 2번 짧게 = 2 ticks ON, 2 ticks OFF, 2 ticks ON */
        if (s_buz_ticks < 2) {
            ctrl.buzzer = 1u;
        } else if (s_buz_ticks < 4) {
            ctrl.buzzer = 0u;
        } else if (s_buz_ticks < 6) {
            ctrl.buzzer = 1u;
        } else {
            ctrl.buzzer = 0u;
            s_buz_seq = BUZ_NONE;
        }

        if ((s_buz_seq != BUZ_NONE) && advance) {
            s_buz_ticks++;
        }
    }

    /* 3. 상태 변화 시에만 ioctl 호출 */
    if (ctrl.green != s_last_ctrl.green ||
        ctrl.yellow != s_last_ctrl.yellow ||
        ctrl.red != s_last_ctrl.red ||
        ctrl.buzzer != s_last_ctrl.buzzer) {

        if (ioctl(s_fd, LED_SW_SET_LEDS, &ctrl) < 0) {
            (void)fprintf(stderr, "[led     ] ioctl(LED_SW_SET_LEDS) failed: %s\n", strerror(errno));
        } else {
            s_last_ctrl = ctrl;
        }
    }
}

static int led_init(struct shared_ctx *ctx)
{
    (void)ctx;
    s_fd = open(LED_SW_DEV_PATH, O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (s_fd < 0) {
        (void)fprintf(stderr,
            "[led     ] %s open failed (%s) — degraded mode\n",
            LED_SW_DEV_PATH, strerror(errno));
        return 0; /* 드라이버 없어도 데몬 지속 */
    }

    (void)fprintf(stderr, "[led     ] init (%s opened successfully)\n", LED_SW_DEV_PATH);

    /* 초기 LED 상태 설정 (노랑 ON: 명령 대기) */
    s_last_ctrl.green  = 0;
    s_last_ctrl.yellow = 1;
    s_last_ctrl.red    = 0;
    s_last_ctrl.buzzer = 0;
    (void)ioctl(s_fd, LED_SW_SET_LEDS, &s_last_ctrl);

    return 0;
}

static int led_get_fd(void)
{
    return s_fd; /* epoll 에 등록하여 스위치 press 이벤트 수신 */
}

static void led_on_event(struct shared_ctx *ctx)
{
    struct led_sw_event evt;
    ssize_t ret;

    if (s_fd < 0 || ctx == NULL) {
        return;
    }

    /* FIFO 이벤트 모두 읽기 */
    while ((ret = read(s_fd, &evt, sizeof(evt))) == (ssize_t)sizeof(evt)) {
        if (evt.state != 1u) {
            continue; /* press 이벤트만 처리 */
        }

        if (evt.sw_id == SW_SCAN_START) {
            (void)fprintf(stderr, "[led     ] SW_SCAN_START pressed -> CMD_SCAN_START trigger\n");
            /* 표준 기본값(daemon_module.h). 여기에 숫자를 다시 적지 않는다 —
             * 예전에 그렇게 해서 버튼만 1.0도 격자로 찍고 웹·scan_batch 는
             * 0.9도로 찍는 상태가 됐다. 같은 킷의 산출물끼리 격자가 다르면
             * 소비자가 조용히 틀린 계산을 한다. */
            if (ctx->req.valid == 0u) {
                ctx->req.pan_start_ddeg   = SCAN_DEF_PAN_START_DDEG;
                ctx->req.pan_end_ddeg     = SCAN_DEF_PAN_END_DDEG;
                ctx->req.tilt_start_ddeg  = SCAN_DEF_TILT_START_DDEG;
                ctx->req.tilt_end_ddeg    = SCAN_DEF_TILT_END_DDEG;
                ctx->req.step_ddeg        = SCAN_DEF_STEP_DDEG;
                ctx->req.sensor_height_mm = SCAN_DEF_HEIGHT_MM;
                ctx->req.valid            = 1u;      /* 코어가 소비 */
            }
        } else if (evt.sw_id == SW_EMS) {
            if (ctx->state == ST_DISARM) {
                (void)fprintf(stderr, "[led     ] SW_EMS pressed in DISARM -> CMD_REARM trigger\n");
                ctx->req_rearm = 1u;
            } else {
                (void)fprintf(stderr, "[led     ] SW_EMS pressed -> CMD_DISARM trigger\n");
                ctx->req_disarm = 1u;
            }
        }
    }
}

/* cppcheck-suppress constParameterCallback ; on_tick 은 daemon_module 콜백 ABI */
static void led_on_tick(struct shared_ctx *ctx, daemon_state_t state)
{
    (void)state;

    /* CMD_ERROR 엣지 검출 */
    if (ctx->link.last_err != 0 && ctx->link.last_err != s_last_err) {
        s_buz_seq = BUZ_ERROR;
        s_buz_ticks = 0;
    }
    s_last_err = ctx->link.last_err;

    update_leds_buzzer(ctx, true);    /* 틱이 시퀀스 시계다 */
}

/* cppcheck-suppress constParameterCallback ; 콜백 ABI 고정 */
static void led_on_state(struct shared_ctx *ctx,
                         daemon_state_t old_st, daemon_state_t new_st)
{
    /* SCAN_DONE: ST_EXPORT -> ST_IDLE 복귀 시 1.0초 완료 알림음 (10 ticks)
     *
     * ⚠️ 이전에는 `new_st == ST_EXPORT` 진입 시 부저를 켰는데, 바로 다음 순서인
     *   camera_module 의 mTLS 업로드가 동기 블로킹(수 초~수십 초)으로 실행되면서
     *   100ms 틱 타이머가 멈춰 카메라 통신 내내 부저가 꺼지지 않고 계속 울리는
     *   버그가 발생했다.
     *   업로드가 모두 끝나고 메인 루프로 복귀하는 `ST_EXPORT -> ST_IDLE` 시점에
     *   트리거하여 타이머 틱이 블로킹 없이 정확히 1.0초 카운트 후 소등되도록 한다. */
    if (old_st == ST_EXPORT && new_st == ST_IDLE) {
        s_buz_seq = BUZ_SCAN_DONE;
        s_buz_ticks = 0;
    } else if (new_st == ST_DISARM && old_st != ST_IDLE) {
        /* 비상정지 (ST_DISARM 진입) 감지 -> 0.2초 2회 경고음
         * (스캔 완료 15초 후의 정상 자동 절전 DISARM 은 에러음에서 제외) */
        s_buz_seq = BUZ_ERROR;
        s_buz_ticks = 0;
    }

    update_leds_buzzer(ctx, false);   /* 전이는 표시만 — 시계는 안 돌린다 */
}

static void led_deinit(struct shared_ctx *ctx)
{
    (void)ctx;
    if (s_fd >= 0) {
        struct led_sw_ctrl off = {0, 0, 0, 0};
        (void)ioctl(s_fd, LED_SW_SET_LEDS, &off);
        (void)close(s_fd);
        s_fd = -1;
    }
}

static const struct daemon_module k_led = {
    "led",
    led_init,
    led_get_fd,
    led_on_event,
    led_on_tick,
    led_on_state,
    led_deinit,
};

const struct daemon_module *led_module_get(void)
{
    return &k_led;
}
