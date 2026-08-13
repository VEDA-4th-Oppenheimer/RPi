/* ============================================================================
 *  main.c  --  통합 데몬 코어 (1D LiDAR 스캐너 / 자동 캘리브레이션 킷)
 *  담당: 이현우
 *
 *  - 단일 스레드 epoll 이벤트 루프 (락 불필요, 콜백 블로킹 금지)
 *  - 상태머신 IDLE -> SCANNING -> EXPORT -> IDLE (+ DISARM)  [daemon_module.h v3]
 *  - shared_ctx 소유. 모듈(mqtt/imu/led)을 등록·구동
 *  - /dev/turret 은 코어가 직접 다룬다 (protocol.h v5):
 *      ioctl : HOME / SCAN_START / SCAN_STOP / DISARM / GET_STATE / PING
 *      read(): CMD_SCAN_DATA 스트림 (struct proto_scan_point 배치)
 *    없으면 degraded 모드로 계속 구동 (개발 PC/컨테이너 대응)
 *  - 100ms timerfd tick 에서 heartbeat(PING/300ms link_dead) + 모듈 on_tick
 *  - 스캔 점을 organized 격자(row,col)에 배치 후 종료 시 두 파일 동시 산출:
 *      .json = 원시 측정 (계약 golden reference, x/y/z 없음)
 *      .pcd  = (x,y,z) 변환본 (organized, 뷰어·소비 편의)
 *  - STM 이 올리는 각도는 기구각이라, 격자에 넣기 전에 계약각으로 옮긴다
 *    (mech_to_contract). 2축 스윕은 바닥을 넘어가 기구각과 1:1 이 아니다.
 *
 *  언어: C11 (systems/epoll 은 C API. fd 는 raw int, shutdown 에서 명시 close)
 * ==========================================================================*/
#define PROTO_WANT_IOCTL 1          /* protocol.h 의 ioctl 인터페이스 노출 */

#include "daemon_module.h"
#include "scan_output.h"
#include "protocol.h"

#include <sys/epoll.h>
#include <sys/timerfd.h>
#include <sys/signalfd.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <time.h>

#define TICK_MS       100           /* heartbeat/tick 주기 (HB_PING_PERIOD_MS) */
#define MAX_EVENTS    16
#define MAX_MODULES   8
#define TURRET_DEV    "/dev/turret"
#define SCAN_BATCH    64            /* read() 1회에 받을 최대 점 수 */




/* ---------------------------------------------------------------------------
 *  코어 구조체
 * ------------------------------------------------------------------------- */
struct core {
    struct shared_ctx ctx;
    int  epoll_fd;
    int  turret_fd;                                 /* 없을 수 있음(개발 PC) */
    int  timer_fd;
    int  signal_fd;

    const struct daemon_module *modules[MAX_MODULES];
    int  module_fd[MAX_MODULES];                    /* 모듈별 epoll fd 또는 -1 */
    int  n_modules;
    bool running;

    /* heartbeat 상태(데몬 소유). 드라이버 pong_seq 증가를 자기 시계로 스탬프 */
    uint32_t hb_last_seq;      /* 마지막으로 관측한 pong_seq        */
    uint64_t hb_last_pong;     /* 마지막 PONG 관측 시각(mono ms)    */
    bool     hb_primed;        /* 최초 GET_STATE 로 기준선 설정 여부 */

    /* 마지막으로 **로그에 찍은** STM 에러코드. 드라이버가 주는 last_err 는
     * 다음 성공까지 값이 유지되므로, 그대로 찍으면 100ms tick 마다 같은 줄이
     * 초당 10회 쌓인다. 값이 바뀔 때만 찍기 위한 에지 검출용이다. */
    uint8_t  last_err_seen;

    /* 스캔 전 자동 홈. STM 은 홈 전 SCAN_START 를 ERR_NOT_HOMED 로 거절하므로
     * 요청이 들어오면 먼저 홈을 세우고 STF_HOMED 를 기다린다. */
    uint64_t home_req_first_ms;   /* 첫 요청 시각 (0 = 요청 안 함)  */
    uint64_t home_req_last_ms;    /* 마지막 송신 시각 (재시도 간격) */

    /* cmd/home 으로 들어온 **단독** 홈이 진행 중인가. 스캔 전 자동 홈과 같은
     * 대기 로직(core_await_home)을 쓰지만, 끝났을 때 스캔으로 넘어가지 않고
     * IDLE 에 그대로 머문다는 점만 다르다. */
    bool     home_manual;

    /* 스캔 완료 후 자동 DISARM 예약. 0 = 예약 없음, 그 외 = EXPORT 를 빠져나온
     * 시각(mono ms). 이 시각 + POST_SCAN_DISARM_MS 가 지나면 DISARM 으로 간다.
     * 지연을 두는 이유는 POST_SCAN_DISARM_MS 주석 참조. */
    uint64_t auto_disarm_at_ms;

    /* 스캔 산출물 핸들. NULL 이면 스캔 중 아님.
     * 격자·통계·경로는 전부 scan_output.c 안에 있다 — 예전엔 그 15개 필드가
     * 여기 섞여 있어 heartbeat/FSM 을 읽을 때도 계속 눈에 밟혔다. */
    struct scan_out *out;

    /* 축교점→라이다 발광면 거리(mm). 기구 상수라 scan_request 가 아니라 여기
     * 둔다 — 스캔마다 바뀌는 값이 아니고, 요청 구조체에 넣으면 MQTT 가 채운
     * 요청이 0 으로 들어와 보정이 조용히 사라질 수 있다. */
    int32_t  lidar_offset_mm;

    /* CLI --once : 스캔 1회 완료 후 데몬 종료 (레이어별 실행용) */
    bool     exit_after_scan;

    /* 정상 완료로 종료하는 중인가.
     *
     * ⚠️ true 면 core_shutdown() 이 DISARM 을 보내지 않는다.
     *   STM32 는 SCAN_DONE 을 보낸 **뒤** 케이블 되감기(역회전)를 수행하는데,
     *   DISARM 은 그 시퀀스를 즉시 중단시켜 축이 시작 각도로 못 돌아온다
     *   (라이다 케이블이 감긴 채 남아 다음 스캔 불가).
     *   되감기 종료 시 STM32 가 스스로 PWM 을 끄므로 방치되는 것은 없다.
     *   비상 정지(link_dead / CMD_ERROR / SIGTERM)는 이 플래그가 false 라
     *   기존대로 DISARM 이 나간다. */
    bool     clean_exit;
    /* --once 로 돌 때 스캔이 실제로 산출까지 갔는지. 배치 스크립트가 종료코드
     * 하나로 성공/실패를 가릴 수 있게 한다 — 한국어 로그를 grep 하게 만들면
     * 문구를 바꿀 때마다 스크립트가 조용히 깨진다. */
    bool     scan_failed;

    /* 무입력 타임아웃 (SCAN_DONE 유실 대비 안전망) */
    uint64_t last_point_ms;        /* 마지막 점 수신 시각 */
};

/* SCAN_DONE 을 놓쳤을 때를 대비한 안전망: 이 시간 동안 점이 하나도 안 오면 종료.
 * 라이다 100Hz(10ms) 대비 충분히 길고, 줄 끝 방향전환보다도 길게 잡는다. */
#define SCAN_IDLE_TIMEOUT_MS   3000u

/* 첫 점 대기 한도. 스캔 시작 후 이 시간 안에 점이 하나도 안 오면
 * 라이다/링크 이상으로 보고 마감한다(안 그러면 ST_SCANNING 에 영구 체류).
 * 무입력(3000ms)보다 길게 잡는 이유: 시작 직후 STM 이 HOMING → MOVE_START
 * 로 시작 자세까지 이동하는 동안은 점이 없는 게 정상이다.
 * 최악(팬 180° @200Hz ≈ 8s + 틸트 ≈ 1s)을 덮도록 여유를 둔다. */
#define SCAN_FIRST_POINT_TIMEOUT_MS 15000u

/* 스캔 전 자동 홈 대기.
 * 홈은 절대 엔코더 판독 1회라 구동이 없어 수 ms 면 끝난다. UART 왕복까지
 * 쳐도 여유가 크므로 재시도 간격은 짧게, 포기 시각은 넉넉히 잡는다. */
#define HOME_RETRY_MS           500u

/* ⚠️ 3000 이었는데 늘렸다. 홈이 판독만 하던 시절엔 응답이 수 ms 였지만, 지금은
 *   판독 후 **양축을 기구각 0(홈 자세)으로 보내고** 도달·정착까지 확인한 뒤에야
 *   CMD_HOMED 를 올린다. 최악의 경우 180도 이동(22.5도/s = 8초) + 정착 3초라
 *   3초 타임아웃이면 정상 동작을 무응답으로 오판해 스캔이 매번 취소된다.
 *   (STM 쪽은 홈 절차 중 들어온 CMD_HOME 재시도를 무시한다 — 안 그러면
 *    500ms 마다 정착 대기가 처음부터 다시 시작돼 영영 안 끝난다) */
#define HOME_TIMEOUT_MS       20000u

/* 스캔 완료 후 자동 DISARM 까지의 유예(ms).
 *
 * 스캔이 끝나면 킷을 안전 상태로 두는 편이 낫다 — IDLE 로 두면 스텝이 계속
 * 여자된 채 방치되고, 조작자는 화면만 보고 "끝난 건지 대기 중인지" 를 가릴 수
 * 없다. 그래서 EXPORT 를 지나면 DISARM 으로 내린다.
 *
 * ⚠️ **곧바로 보내면 안 된다.** STM32 는 CMD_SCAN_DONE 을 올린 **뒤** 케이블
 *   되감기(역회전)를 스스로 수행하는데, CMD_DISARM 은 그 시퀀스를 즉시
 *   중단시킨다. 축이 시작 각도로 못 돌아오고 라이다 케이블이 감긴 채 남아
 *   다음 스캔이 불가능해진다. 되감기가 끝나면 STM32 가 스스로 PWM 을 끄므로,
 *   그때까지 기다렸다가 보내는 DISARM 은 안전 재확인이지 방해가 아니다.
 *
 * ⚠️ 이 값은 **되감기 실측 전까지의 잠정치**다. 프로토콜 v5 에 "되감기 완료"
 *   통지가 없어서(STM 은 CMD_STATUS 를 주기 발행하지 않는다) 시간으로 때우는
 *   수밖에 없다. 홈 이동 최악치(팬 180도 @22.5도/s ≈ 8초)에 여유를 더해 잡았다.
 *   실측 후 조정하거나, 펌웨어에 완료 통지를 추가하면 그걸로 갈아탈 것.
 *   짧으면 되감기가 끊기고, 길면 그동안 스텝이 여자된 채 남는다. */
#define POST_SCAN_DISARM_MS   15000u

/* ---------------------------------------------------------------------------
 *  파일 디스크립터 헬퍼 (C: RAII 없음 → 명시 관리)
 * ------------------------------------------------------------------------- */
static void close_fd(int *fd)
{
    if (*fd >= 0) {
        (void)close(*fd);
        *fd = -1;
    }
}

/* 주기 tick 용 timerfd 생성 (period_ms 간격 반복). 실패 시 -1 */
static int make_timerfd(int period_ms)
{
    int fd = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK | TFD_CLOEXEC);
    if (fd < 0) {
        return -1;
    }
    struct itimerspec its;
    memset(&its, 0, sizeof(its));
    its.it_interval.tv_sec  = period_ms / 1000;
    its.it_interval.tv_nsec = (long)(period_ms % 1000) * 1000000L;
    its.it_value = its.it_interval;
    if (timerfd_settime(fd, 0, &its, NULL) < 0) {
        (void)close(fd);
        return -1;
    }
    return fd;
}

/* SIGINT/SIGTERM 를 signalfd 로 받아 epoll 에서 처리 (graceful shutdown) */
static int make_signalfd(void)
{
    sigset_t mask;
    sigemptyset(&mask);
    sigaddset(&mask, SIGINT);
    sigaddset(&mask, SIGTERM);
    /* ★ SIGHUP 도 받는다. 안 받으면 기본 동작이 "즉시 종료" 라서 core_shutdown
     *   이 안 불리고, **격자가 메모리에 있는 채로 프로세스가 사라진다** —
     *   산출물은 스캔이 끝날 때 한 번에 쓰이므로 27분짜리 스캔이 통째로 없어진다.
     *
     *   SIGHUP 은 SSH 접속이 끊길 때 온다. 터미널의 pty 가 닫히면 커널이 전경
     *   프로세스 그룹 전체에 보내므로, tmux 없이 원격에서 돌리다 회선이 끊기면
     *   그대로 당한다(§19-3 의 권한 사고와 결과가 같다).
     *
     *   데몬에 다시 읽을 설정 파일이 없으므로 관례적 의미("설정 재적재")와
     *   충돌하지 않는다. TERM 과 똑같이 graceful shutdown 으로 처리한다 —
     *   DISARM 을 보내 축을 세우고 여기까지 받은 점으로 파일을 마감한다. */
    sigaddset(&mask, SIGHUP);
    if (sigprocmask(SIG_BLOCK, &mask, NULL) < 0) {
        return -1;
    }
    return signalfd(-1, &mask, SFD_NONBLOCK | SFD_CLOEXEC);
}

/* 단조 증가 시각(ms). heartbeat 300ms 판정은 이 시계 기준(데몬 소유). */
static uint64_t mono_ms(void)
{
    struct timespec ts;
    memset(&ts, 0, sizeof(ts));
    (void)clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000u + (uint64_t)ts.tv_nsec / 1000000u;
}

/* 단조 증가 시각(ns). JSON 계약의 timestamp_ns 는 ns 고정이다.
 *
 * ⚠️ **clock domain 규칙**: scan.started_at_ns / ended_at_ns / measurement
 *   timestamp_ns 는 반드시 같은 시계여야 한다(계약 §일관성 검증 5번).
 *   따라서 셋 다 이 RPi 단조시계를 쓴다.
 *
 *   v5 에서 STM32 HAL tick 이 들어오지만 그건 **다른 clock domain** 이라
 *   timestamp_ns 로 쓰면 안 된다(실측: measurement 24초 vs scan 749초로
 *   349/349 전부 범위 밖이 됐다). STM 시계는 stm32_time_ms 필드로 따로 보존해
 *   나중에 offset 추정이 가능하게 한다.
 *
 *   ⚠️ 이 값은 UART 큐잉 지연(수 ms)을 포함한 **수신 시각**이다. 정밀 동기가
 *     필요해지면 stm32_time_ms 와의 회귀로 offset·drift 를 추정할 것. */

static bool epoll_add(int epfd, int fd, uint32_t events)
{
    struct epoll_event ev;
    memset(&ev, 0, sizeof(ev));
    ev.events  = events;
    ev.data.fd = fd;
    return epoll_ctl(epfd, EPOLL_CTL_ADD, fd, &ev) == 0;
}

/* 전방 선언 (상호 호출) */
static void core_transition(struct core *c, daemon_state_t want);


/* ---------------------------------------------------------------------------
 *  스캔 제어 (/dev/turret ioctl)
 * ------------------------------------------------------------------------- */

static bool scan_request_valid(const struct scan_request *r)
{
    return  r->step_ddeg > 0u
        &&  r->pan_start_ddeg  >= PAN_MIN  && r->pan_start_ddeg  <= PAN_MAX
        &&  r->pan_end_ddeg    >= PAN_MIN  && r->pan_end_ddeg    <= PAN_MAX
        &&  r->tilt_start_ddeg >= TILT_MIN && r->tilt_start_ddeg <= TILT_MAX
        &&  r->tilt_end_ddeg   >= TILT_MIN && r->tilt_end_ddeg   <= TILT_MAX;
}

/* 수평 게이트: IMU roll/pitch 가 임계값을 넘으면 스캔을 거부한다(방식 A).
 * IMU 가 없거나 아직 측정 전이면 통과시키되 로그로 남긴다(개발 편의). */
static bool level_gate_ok(struct core *c)
{
    const struct level_state *l = &c->ctx.level;
    if (l->valid == 0u) {
        core_log(c, "LEVEL", "IMU 값 없음 — 수평 게이트 생략(주의)");
        return true;
    }
    const float ar = (l->roll_deg  < 0.0f) ? -l->roll_deg  : l->roll_deg;
    const float ap = (l->pitch_deg < 0.0f) ? -l->pitch_deg : l->pitch_deg;
    if (ar > LEVEL_GATE_MAX_DEG || ap > LEVEL_GATE_MAX_DEG) {
        core_log(c, "LEVEL", "수평 NG: roll=%.2f pitch=%.2f (임계 %.1f) — 스캔 거부",
                 (double)l->roll_deg, (double)l->pitch_deg, (double)LEVEL_GATE_MAX_DEG);
        return false;
    }
    core_log(c, "LEVEL", "수평 OK: roll=%.2f pitch=%.2f",
             (double)l->roll_deg, (double)l->pitch_deg);
    return true;
}

static bool core_scan_begin(struct core *c)
{
    const struct scan_request *r = &c->ctx.req;

    if (!scan_request_valid(r)) {
        core_log(c, "SCAN", "잘못된 스캔 요청 (범위/격자) — 거부");
        return false;
    }
    scan_out_warn_seam(r, c);
    if (!level_gate_ok(c)) {
        return false;
    }
    c->out = scan_out_open(r, c->lidar_offset_mm, c);
    if (c->out == NULL) {
        return false;
    }

    if (c->turret_fd >= 0) {
        struct proto_scan_start ss;
        memset(&ss, 0, sizeof(ss));
        ss.pan_start_ddeg  = r->pan_start_ddeg;
        ss.pan_end_ddeg    = r->pan_end_ddeg;
        ss.tilt_start_ddeg = r->tilt_start_ddeg;
        ss.tilt_end_ddeg   = r->tilt_end_ddeg;
        ss.step_ddeg       = r->step_ddeg;
        if (ioctl(c->turret_fd, TURRET_SCAN_START, &ss) < 0) {
            core_log(c, "SCAN", "SCAN_START ioctl 실패: %s", strerror(errno));
            scan_out_close(&c->out);
            return false;
        }
    } else {
        core_log(c, "SCAN", "degraded — turret 없이 파일만 준비");
    }

    memset(&c->ctx.progress, 0, sizeof(c->ctx.progress));
    c->ctx.progress.expected = scan_out_expected_points(r);
    memset(&c->ctx.result, 0, sizeof(c->ctx.result));

    c->last_point_ms = mono_ms();      /* 무입력 타임아웃 기준 */

    core_log(c, "SCAN", "시작: pan[%d..%d] tilt[%d..%d] step=%u (예상 %u점)",
             r->pan_start_ddeg, r->pan_end_ddeg,
             r->tilt_start_ddeg, r->tilt_end_ddeg,
             r->step_ddeg, c->ctx.progress.expected);
    return true;
}

/* 스캔 중단 요청 (사용자 stop / 에러). 파일은 EXPORT 에서 마감된다. */
static void core_scan_abort(struct core *c)
{
    if (c->turret_fd >= 0) {
        (void)ioctl(c->turret_fd, TURRET_SCAN_STOP);
    }
    core_log(c, "SCAN", "중단 요청");
}

/* 스캔 점 스트림 수신: read() 로 배치를 받아 즉시 (x,y,z) 로 변환·기록.
 * 드라이버는 kfifo 에 누적하고 POLLIN 으로 알린다(ioctl 아님). */
static void core_drain_scan_points(struct core *c)
{
    if (c->turret_fd < 0) {
        return;
    }
    struct proto_scan_point batch[SCAN_BATCH];

    for (;;) {
        const ssize_t n = read(c->turret_fd, batch, sizeof(batch));
        if (n <= 0) {
            break;                       /* EAGAIN / EOF */
        }
        const size_t cnt = (size_t)n / sizeof(batch[0]);
        for (size_t i = 0; i < cnt; ++i) {
            scan_out_add(c->out, &batch[i]);
        }
        c->last_point_ms = mono_ms();      /* 무입력 타임아웃 기준 갱신 */
        c->ctx.progress.points = scan_out_point_count(c->out);
        if (c->ctx.progress.expected > 0u) {
            const uint64_t pct = (uint64_t)c->ctx.progress.points * 100u
                               / c->ctx.progress.expected;
            c->ctx.progress.percent = (uint8_t)((pct > 100u) ? 100u : pct);
        }
        if ((size_t)n < sizeof(batch)) {
            break;                       /* 더 읽을 것 없음 */
        }
    }
}

/* ---------------------------------------------------------------------------
 *  heartbeat + STM 상태 캐시
 *   - PING 송신(fire-and-forget): 주기는 tick(100ms) = HB_PING_PERIOD_MS
 *   - PONG 도착은 드라이버 pong_seq 증가로 감지 → 자기 CLOCK_MONOTONIC 스탬프
 *   - HB_TIMEOUT_MS(300ms) 무응답 시 link_dead 판정 → DISARM (정책=데몬 소유)
 * ------------------------------------------------------------------------- */
/* 상태만 읽는다(PING 송신 없음). 스캔 중 이벤트 경로에서 쓰인다. */
static void core_read_state(struct core *c)
{
    if (c->turret_fd < 0) {
        return;                          /* degraded: STM 링크 없음 */
    }

    struct turret_link_state st;
    memset(&st, 0, sizeof(st));
    if (ioctl(c->turret_fd, TURRET_GET_STATE, &st) < 0) {
        return;
    }
    c->ctx.link.cur_pan_ddeg  = st.cur_pan_ddeg;
    c->ctx.link.cur_tilt_ddeg = st.cur_tilt_ddeg;
    c->ctx.link.homed         = ((st.flags & STF_HOMED)    != 0u) ? 1u : 0u;
    c->ctx.link.scanning      = ((st.flags & STF_SCANNING) != 0u) ? 1u : 0u;
    c->ctx.link.last_err      = st.last_err;

    if ((st.flags & STF_HOMED) != 0u) {
        scan_out_set_home(c->out, st.home_pan_encoder_raw,
                          st.home_tilt_encoder_raw,
                          st.home_pan_ddeg, st.home_tilt_ddeg);
    }

    const uint64_t now = mono_ms();
    if (!c->hb_primed) {
        c->hb_primed    = true;          /* 시작 시점부터 grace 부여 */
        c->hb_last_seq  = st.pong_seq;
        c->hb_last_pong = now;
    } else if (st.pong_seq != c->hb_last_seq) {
        c->hb_last_seq  = st.pong_seq;
        c->hb_last_pong = now;
    }

    const bool alive = (now - c->hb_last_pong) <= HB_TIMEOUT_MS;
    c->ctx.link.link_alive = alive ? 1u : 0u;

    if (!alive && c->ctx.state != ST_DISARM) {
        core_log(c, "LINK", "link_dead (PONG > %ums) -> DISARM", HB_TIMEOUT_MS);
        core_transition(c, ST_DISARM);
        return;
    }
    /* ⚠️ 예전에는 이 로그가 `state == ST_SCANNING` 안에 갇혀 있었다. 그런데
     *   홈 대기 중 상태는 ST_IDLE 이라, STM 이 홈 도중 올린 ERR_NOT_HOMED
     *   (엔코더 I2C 판독 실패) / ERR_STALL(홈 수렴 실패) 이 last_err 에 담기기만
     *   하고 아무 데도 안 찍혔다. 데몬 화면에는 20초 침묵 뒤 "홈 무응답" 만
     *   남아서, 실제로는 STM 이 또렷하게 이유를 말했는데도 링크 문제로 오해했다.
     *
     *   그래서 **보고(어느 상태에서든)와 DISARM(스캔 중에만)을 분리**한다.
     *   IDLE 에서의 에러는 요청 거절이지 비상정지가 아니므로 전이하지 않는다.
     *
     *   같은 코드가 매 tick(100ms) 반복 출력되지 않도록 값이 바뀔 때만 찍는다.
     *   STM 은 last_err 를 다음 성공까지 유지하므로 그러지 않으면 초당 10줄이다. */
    if (st.last_err != c->last_err_seen) {
        c->last_err_seen = st.last_err;
        if (st.last_err != ERR_NONE) {
            core_log(c, "STM", "CMD_ERROR code=%u (state=%s)",
                     st.last_err, daemon_state_str(c->ctx.state));
        }
    }
    if (st.last_err != ERR_NONE && c->ctx.state == ST_SCANNING) {
        core_log(c, "STM", "스캔 중 오류 -> DISARM");
        core_transition(c, ST_DISARM);
    }
}

/* heartbeat PING 송신 + 상태 갱신. 100ms tick 에서만 호출한다.
 * ⚠️ 이벤트 경로(POLLIN)에서 부르면 스캔 중 PING 이 초당 100회 나가
 *   STM 메인루프를 마비시킨다. 그쪽은 core_read_state() 를 쓸 것. */
static void core_poll_link(struct core *c)
{
    if (c->turret_fd < 0) {
        return;
    }
    (void)ioctl(c->turret_fd, TURRET_PING);   /* 실패는 타임아웃이 흡수 */
    core_read_state(c);
}

/* ---------------------------------------------------------------------------
 *  상태 평가 / 전이
 * ------------------------------------------------------------------------- */
/* 스캔 요청이 들어왔는데 아직 홈이 안 섰을 때 호출.
 *
 * STM 은 홈 전 SCAN_START 를 ERR_NOT_HOMED 로 거절한다(protocol.h §4).
 * 예전에는 데몬이 CMD_HOME 을 아예 보내지 않아, 사용자가 turret_test home 을
 * 따로 치지 않으면 스캔이 항상 거절됐다.
 *
 * 요청을 소비하지 않고 남겨둔 채 홈만 세운다. 홈이 서면 다음 tick 에서
 * 그대로 SCANNING 으로 넘어간다.
 *   반환 true  = 아직 대기 중 (이번 tick 은 전이하지 않는다)
 *        false = 홈 완료 또는 포기 — 호출자가 판단 */
static bool core_await_home(struct core *c)
{
    const uint64_t now = mono_ms();
    bool waiting = true;

    if (c->home_req_first_ms == 0u) {
        /* 이번 요청의 첫 진입 — **캐시된 homed 를 보지 않고 무조건** 보낸다.
         *
         * 캐시를 믿으면 안 되는 이유(실기에서 발생):
         *   드라이버의 STF_HOMED 는 CMD_HOMED 를 받을 때 서기만 하고, STM 은
         *   CMD_STATUS 를 주기 송신하지 않아 갱신될 일도 없다. 그래서 한 번
         *   홈을 잡은 뒤 STM 을 리셋/재플래시하면 STM 의 s.homed 는 false 인데
         *   드라이버 캐시만 참으로 남는다. 그 상태로 홈을 건너뛰면 SCAN_START
         *   가 매번 ERR_NOT_HOMED 로 거절된다.
         *
         * 홈은 절대 엔코더 판독 1회라 구동이 없어 비용이 사실상 0 이다.
         * 스캔마다 다시 잡는 편이 캐시를 신뢰하는 것보다 안전하다.
         * (드라이버도 이 ioctl 에서 STF_HOMED 를 내리므로, 이후 homed==1 은
         *  반드시 이번 HOME 에 대한 응답이다) */
        c->home_req_first_ms = now;
        c->home_req_last_ms  = now;
        if (ioctl(c->turret_fd, TURRET_HOME) < 0) {
            core_log(c, "HOME", "TURRET_HOME ioctl 실패: %s", strerror(errno));
        } else {
            core_log(c, "HOME", "%s 홈 확립 요청 (CMD_HOME)",
                     c->home_manual ? "단독" : "스캔 전");
        }
    } else if (c->ctx.link.homed != 0u) {
        waiting = false;                   /* 이번 HOME 에 대한 응답 도착 */
    } else if ((now - c->home_req_first_ms) > HOME_TIMEOUT_MS) {
        core_log(c, "HOME",
                 "홈 무응답 %ums — 요청 취소 (링크/펌웨어 확인)",
                 HOME_TIMEOUT_MS);
        /* 스캔 대기 중이었다면 그 요청도 함께 버린다. 단독 홈(req_home)이면
         * 이미 0 이라 무해하다. 성공/실패 구분은 호출자가 link.homed 로 한다. */
        c->ctx.req.valid = 0u;
        waiting = false;
    } else if ((now - c->home_req_last_ms) >= HOME_RETRY_MS) {
        c->home_req_last_ms = now;
        (void)ioctl(c->turret_fd, TURRET_HOME);   /* 재시도는 조용히 */
    } else {
        /* 계속 대기 */
    }
    return waiting;
}

/* DISARM -> IDLE 복구(REARM).
 *
 * ★ 모터 재인가 명령은 보내지 않는다. protocol v5 에 CMD_ARM 이 없고, STM 은
 *   다음 HOME/SCAN_START 에서 스텝을 다시 켠다. 데몬은 스캔 직전에 항상 홈을
 *   다시 잡으므로(core_await_home), 복구는 데몬 상태만 되돌리면 충분하다.
 *   ⚠️ 즉 REARM 직후의 축 위치는 여전히 미지다 — "다시 스캔을 걸 수 있는
 *     상태" 로 돌아갈 뿐, 물리적으로 홈에 선 것이 아니다.
 *
 * ★ 링크가 죽어 있으면 거부한다. 그 상태로 IDLE 로 올려봐야 다음 tick 의
 *   heartbeat 판정(core_read_state)이 곧바로 다시 DISARM 으로 떨어뜨린다.
 *   그 왕복은 retained 상태 발행으로 그대로 나가 Qt 화면만 깜빡이고, 조작자는
 *   "복구가 왜 안 되는지" 를 읽을 수 없다. 여기서 이유를 로그로 남기고 막는다.
 *   (link_alive 는 DISARM 중에도 매 tick 갱신되므로 STM 이 돌아오면 저절로
 *    통과한다 — 데몬 재시작이 필요하지 않다) */
static void core_rearm(struct core *c)
{
    if (c->ctx.state != ST_DISARM) {
        core_log(c, "REARM", "%s 상태 — 무시 (DISARM 에서만 복구)",
                 daemon_state_str(c->ctx.state));
        return;
    }
    if ((c->turret_fd >= 0) && (c->ctx.link.link_alive == 0u)) {
        core_log(c, "REARM", "링크 단절 중 — 복구 거부 (STM32/UART 확인)");
        return;
    }

    /* 대기 중이던 스캔 요청을 버린다. DISARM 직전에 들어와 있던 요청을 그대로
     * 두면 복구하자마자 조작자가 시키지도 않은 스캔이 시작된다 — 비상정지를
     * 누른 사람이 가장 원하지 않는 동작이다. */
    c->ctx.req.valid     = 0u;
    c->ctx.req_scan_stop = 0u;
    c->home_manual       = false;   /* 홈 대기 중 정지했을 수 있다 */
    c->home_req_first_ms = 0u;
    c->home_req_last_ms  = 0u;

    core_log(c, "REARM", "DISARM 해제 — IDLE 복귀 (스캔은 다시 요청해야 한다)");
    core_transition(c, ST_IDLE);
}

static void core_eval_state(struct core *c)
{
    switch (c->ctx.state) {
    case ST_IDLE:
        /* 새 작업이 들어오면 예약된 자동 DISARM 을 취소한다. 조작자가 킷을
         * 계속 쓰는 중인데 유예가 끝났다고 안전정지로 떨어지면, 다음 스캔이
         * 갑자기 거절되고 왜 그런지도 화면에 안 나온다. */
        if ((c->ctx.req.valid != 0u) || c->home_manual) {
            c->auto_disarm_at_ms = 0u;
        }
        if (c->ctx.req.valid != 0u) {                 /* MQTT scan/start 수신 */
            /* turret 이 있으면 스캔 전에 항상 홈을 다시 잡는다(위 주석 참조).
             * 대기 중에는 요청을 소비하지 않고 다음 tick 에 다시 본다 —
             * 소비해버리면 홈이 선 뒤에 스캔이 사라진다. */
            if ((c->turret_fd >= 0) && core_await_home(c)) {
                /* 홈 대기 중 */
            } else {
                const bool cancelled = (c->ctx.req.valid == 0u);

                c->home_req_first_ms = 0u;            /* 대기 상태 정리 */
                c->home_req_last_ms  = 0u;
                if (!cancelled) {
                    c->ctx.req.valid = 0u;            /* 요청 소비 */
                    core_transition(c, ST_SCANNING);
                } else if (c->exit_after_scan) {
                    /* ⚠️ 홈 무응답·수평 NG 로 요청이 버려졌다. 예전에는 여기서
                     *   아무것도 안 해서 --once 데몬이 **영원히 살아 있었다.**
                     *   스캔은 영영 안 오는데 프로세스는 안 죽으니, 배치로
                     *   돌리면 첫 실패에서 통째로 멈춘다. 종료한다. */
                    core_log(c, "CLI", "--once: 스캔 요청이 취소됨 → 실패 종료");
                    c->scan_failed = true;
                    c->clean_exit  = true;
                    c->running     = false;
                } else {
                    /* 상주 모드면 다음 요청을 계속 기다린다 */
                }
            }
        } else if (c->home_manual) {
            /* cmd/home 으로 들어온 단독 홈. 대기 로직은 스캔 전 자동 홈과
             * 완전히 같고, 끝난 뒤 SCANNING 으로 넘어가지 않는 것만 다르다.
             *
             * 스캔 요청이 나중에 겹쳐 들어오면 위 분기가 우선권을 갖는데,
             * home_req_* 를 그대로 물려받으므로 진행 중이던 홈이 취소되거나
             * 처음부터 다시 시작되지 않는다 — 그대로 이어서 기다렸다가
             * 홈이 서면 스캔으로 넘어간다. */
            if ((c->turret_fd >= 0) && core_await_home(c)) {
                /* 홈 대기 중 */
            } else {
                const bool homed = (c->ctx.link.homed != 0u);

                c->home_manual       = false;
                c->home_req_first_ms = 0u;
                c->home_req_last_ms  = 0u;
                if (c->turret_fd < 0) {
                    core_log(c, "HOME", "degraded — turret 없이 홈 생략");
                } else {
                    core_log(c, "HOME", "단독 홈 %s", homed ? "완료" : "실패");
                }
            }
        } else if ((c->auto_disarm_at_ms != 0u)
                   && (mono_ms() >= c->auto_disarm_at_ms)) {
            /* 스캔 후 되감기 유예가 끝났다 — 안전 상태로 내린다.
             * 다시 스캔하려면 조작자가 REARM 을 눌러야 한다(cmd/rearm). */
            c->auto_disarm_at_ms = 0u;
            core_log(c, "SCAN", "되감기 유예 종료 — 자동 DISARM");
            core_transition(c, ST_DISARM);
        } else {
            /* 요청 없음 — 대기 */
        }
        break;

    case ST_SCANNING:
        /* 완료 판정 = STF_SCANNING 해제.
         *   세움 : 드라이버가 SCAN_START ioctl 시점에 (유실 없음)
         *   해제 : 드라이버가 STM 의 CMD_SCAN_DONE 수신 시
         *
         * ⚠️ 과거 이 플래그가 계속 0 이라(STM 이 CMD_STATUS 를 주기 발행하지
         *   않음) 첫 배치 몇 점만 받고 즉시 완료로 오판해 .pcd 가 4점짜리로
         *   끊겼다(실측 확인). 드라이버가 명령 시점에 세우도록 고쳐 해결.
         *
         * 안전망: SCAN_DONE 프레임을 놓쳐도 무입력이 지속되면 마감한다. */
        if (c->turret_fd >= 0) {
            const bool done_sig =
                (c->ctx.link.scanning == 0u) && (c->ctx.progress.points > 0u);

            /* 점이 아직 0 개면 "첫 점 대기"(길게), 하나라도 왔으면
             * "무입력"(짧게). points 를 탈출 자격이 아니라 타임아웃 길이
             * 선택에만 쓰므로 어느 경우에도 반드시 빠져나온다. */
            const bool     started  = (c->ctx.progress.points > 0u);
            const uint32_t limit_ms = started ? SCAN_IDLE_TIMEOUT_MS
                                              : SCAN_FIRST_POINT_TIMEOUT_MS;
            const bool timed_out =
                (mono_ms() - c->last_point_ms) > (uint64_t)limit_ms;

            if (done_sig) {
                core_transition(c, ST_EXPORT);
            } else if (timed_out) {
                if (started) {
                    core_log(c, "SCAN", "무입력 %ums — SCAN_DONE 없이 마감",
                             SCAN_IDLE_TIMEOUT_MS);
                } else {
                    core_log(c, "SCAN",
                             "첫 점이 %ums 동안 없음 — 라이다/링크 확인 요망."
                             " 0 점으로 마감",
                             SCAN_FIRST_POINT_TIMEOUT_MS);
                }
                core_transition(c, ST_EXPORT);
            } else {
                /* 계속 대기 */
            }
        }
        break;

    case ST_EXPORT:
        /* 파일 마감은 전이 진입 시 처리됨. 곧바로 IDLE 로 복귀. */
        core_transition(c, ST_IDLE);
        if (c->exit_after_scan) {
            /* 되감기(STM32 자율 수행)를 끊지 않도록 DISARM 없이 종료한다. */
            core_log(c, "CLI", "--once: 스캔 완료 → 종료 (STM32 되감기 진행 중)");
            c->clean_exit = true;
            c->running    = false;
        } else {
            /* 되감기가 끝날 때까지 기다렸다가 자동으로 안전 상태로 내린다.
             * degraded(turret 없음)에서도 상태는 똑같이 움직인다 — 실제
             * CMD_DISARM 송신만 core_transition 이 막는다. */
            c->auto_disarm_at_ms = mono_ms() + POST_SCAN_DISARM_MS;
            core_log(c, "SCAN", "스캔 완료 — %u초 후 자동 DISARM (되감기 대기)",
                     POST_SCAN_DISARM_MS / 1000u);
        }
        break;

    case ST_DISARM:
    default:
        break;
    }
}

static void core_transition(struct core *c, daemon_state_t want)
{
    const daemon_state_t cur = c->ctx.state;
    if (want == cur) {
        return;
    }
    bool ok = false;
    switch (cur) {
    case ST_IDLE:     ok = (want == ST_SCANNING) || (want == ST_DISARM); break;
    case ST_SCANNING: ok = (want == ST_EXPORT)   || (want == ST_IDLE) || (want == ST_DISARM); break;
    case ST_EXPORT:   ok = (want == ST_IDLE)     || (want == ST_DISARM); break;
    case ST_DISARM:   ok = (want == ST_IDLE); break;   /* rearm */
    default:          ok = false; break;
    }
    if (!ok) {
        core_log(c, "FSM", "reject %s -> %s",
                 daemon_state_str(cur), daemon_state_str(want));
        return;
    }

    /* --- 전이 진입 동작 --- */
    if (want == ST_SCANNING) {
        if (!core_scan_begin(c)) {           /* 게이트/ioctl 실패 → 전이 취소 */
            core_log(c, "FSM", "SCANNING 진입 실패 — IDLE 유지");
            return;
        }
    } else if (want == ST_EXPORT) {
        /* 경로·점수를 **닫기 전에** 뽑는다 — close 가 핸들을 해제한다. */
        (void)snprintf(c->ctx.result.path, sizeof(c->ctx.result.path), "%s",
                       scan_out_path(c->out));
        (void)snprintf(c->ctx.result.json_path, sizeof(c->ctx.result.json_path),
                       "%s", scan_out_json_path(c->out));
        c->ctx.result.point_count = scan_out_point_count(c->out);
        scan_out_close(&c->out);
        c->ctx.result.valid       = 1u;
        core_log(c, "EXPORT", "%s (%u점) — 카메라 단 전달 대기",
                 c->ctx.result.path, c->ctx.result.point_count);
    } else if (want == ST_DISARM) {
        if (c->turret_fd >= 0) {
            (void)ioctl(c->turret_fd, TURRET_DISARM);
        }
        scan_out_close(&c->out);              /* 스캔 중이었으면 파일 마감 */
        /* 홈 대기 중 정지했을 수 있다. 안 지우면 복구 후 IDLE 로 돌아온
         * 순간 취소된 홈이 되살아난다. */
        c->home_manual       = false;
        c->home_req_first_ms = 0u;
        c->home_req_last_ms  = 0u;
        c->auto_disarm_at_ms = 0u;      /* 이미 DISARM — 예약은 의미 없다 */
    }

    c->ctx.state = want;
    core_log(c, "FSM", "%s -> %s", daemon_state_str(cur), daemon_state_str(want));

    for (int i = 0; i < c->n_modules; ++i) {
        const struct daemon_module *m = c->modules[i];
        if (m->on_state != NULL) {
            m->on_state(&c->ctx, cur, want);
        }
    }
}

/* ---------------------------------------------------------------------------
 *  셋업 / 루프 / 종료
 * ------------------------------------------------------------------------- */
static bool core_setup(struct core *c)
{
    c->ctx.core  = c;
    c->ctx.state = ST_IDLE;

    /* /dev/turret (없으면 degraded 로 계속 — 개발 PC/컨테이너 대응) */
    c->turret_fd = open(TURRET_DEV, O_RDWR | O_CLOEXEC | O_NONBLOCK);
    if (c->turret_fd < 0) {
        core_log(c, "TURRET", "open %s 실패 (%s) — degraded 모드로 구동",
                 TURRET_DEV, strerror(errno));
    }

    c->timer_fd  = make_timerfd(TICK_MS);
    c->signal_fd = make_signalfd();
    c->epoll_fd  = epoll_create1(EPOLL_CLOEXEC);
    if (c->timer_fd < 0 || c->signal_fd < 0 || c->epoll_fd < 0) {
        core_log(c, "SETUP", "timer/signal/epoll fd 생성 실패");
        return false;
    }

    if (!epoll_add(c->epoll_fd, c->timer_fd,  EPOLLIN) ||
        !epoll_add(c->epoll_fd, c->signal_fd, EPOLLIN)) {
        core_log(c, "SETUP", "epoll 등록 실패");
        return false;
    }
    if (c->turret_fd >= 0) {
        (void)epoll_add(c->epoll_fd, c->turret_fd, EPOLLIN | EPOLLERR);
    }

    /* 모듈 등록 (v3: mqtt / imu / led) */
    const struct daemon_module *regs[] = {
        mqtt_module_get(),
        imu_module_get(),
        led_module_get(),
        camera_module_get(),
    };
    const int n_regs = (int)(sizeof(regs) / sizeof(regs[0]));

    for (int i = 0; i < n_regs; ++i) {
        const struct daemon_module *m = regs[i];
        if (c->n_modules >= MAX_MODULES) {
            core_log(c, "SETUP", "모듈 수 초과 (MAX_MODULES=%d)", MAX_MODULES);
            return false;
        }
        if (m->init != NULL && m->init(&c->ctx) < 0) {
            core_log(c, "SETUP", "module '%s' init 실패", m->name);
            return false;
        }
        const int mfd = (m->get_fd != NULL) ? m->get_fd() : -1;
        if (mfd >= 0) {
            if (!epoll_add(c->epoll_fd, mfd, EPOLLIN)) {
                core_log(c, "SETUP", "module '%s' fd epoll 등록 실패", m->name);
                return false;
            }
        }
        c->modules[c->n_modules]   = m;
        c->module_fd[c->n_modules] = mfd;
        c->n_modules++;
        core_log(c, "SETUP", "module '%s' 등록 (fd=%d)", m->name, mfd);
    }

    core_log(c, "SETUP", "코어 준비 완료 (turret=%s)",
             (c->turret_fd >= 0) ? "on" : "off");
    return true;
}

/* 모듈 fd 가 바뀌었으면 epoll 등록을 갱신한다.
 *
 * ★ 예전에는 setup 에서 get_fd() 를 **한 번만** 부르고 끝이었다. 그런데
 *   소켓을 쓰는 모듈은 fd 가 런타임에 바뀐다:
 *     · mqtt — 브로커 재접속하면 새 소켓이라 fd 가 달라진다. 갱신하지 않으면
 *       코어는 **닫힌 옛 fd** 를 계속 감시하고 새 소켓의 수신은 영영 못 본다
 *       (증상: 브로커가 재시작하면 명령이 안 먹는데 로그는 "접속됨" 이다).
 *     · 업로드처럼 필요할 때만 소켓을 여는 모듈은 처음엔 -1 이다가 나중에 생긴다.
 *
 *   daemon_module.h 의 get_fd 주석이 "코어는 필요 시 재등록한다" 라고 이미
 *   약속하고 있었는데 구현이 없었다. 여기서 지킨다.
 *
 * 100ms tick 마다 get_fd() 를 부르는 비용은 함수 호출 몇 개라 무시할 수준이다.
 * ⚠️ EPOLL_CTL_DEL 은 **닫히기 전에** 불려야 하는데, 모듈이 이미 close 했다면
 *   커널이 알아서 등록을 해제하므로 실패해도 무해하다(반환값을 안 본다). */
static void core_refresh_module_fds(struct core *c)
{
    for (int i = 0; i < c->n_modules; ++i) {
        const struct daemon_module *m = c->modules[i];

        if (m->get_fd == NULL) {
            continue;
        }
        const int now_fd = m->get_fd();
        const int old_fd = c->module_fd[i];

        if (now_fd == old_fd) {
            continue;
        }
        if (old_fd >= 0) {
            (void)epoll_ctl(c->epoll_fd, EPOLL_CTL_DEL, old_fd, NULL);
        }
        if (now_fd >= 0) {
            if (!epoll_add(c->epoll_fd, now_fd, EPOLLIN)) {
                core_log(c, "SETUP", "module '%s' fd 재등록 실패 (fd=%d)",
                         m->name, now_fd);
                c->module_fd[i] = -1;      /* 다음 tick 에 다시 시도한다 */
                continue;
            }
        }
        c->module_fd[i] = now_fd;
        core_log(c, "SETUP", "module '%s' fd %d -> %d", m->name, old_fd, now_fd);
    }
}

static void core_tick(struct core *c)
{
    uint64_t expirations = 0;
    if (read(c->timer_fd, &expirations, sizeof(expirations)) < 0) {
        /* EAGAIN 등은 무시 */
    }

    core_poll_link(c);           /* STM 링크 캐시 + heartbeat 판정 */
    core_refresh_module_fds(c);  /* 소켓 재접속 등으로 fd 가 바뀌었나 */

    /* ⚠️ 복구를 정지보다 **먼저** 소비한다. 같은 tick 에 둘 다 서면(예: 조작자가
     *   REARM 을 누른 직후 링크가 끊겨 자동 정지가 걸린 경우) 나중에 처리되는
     *   쪽이 최종 상태가 되므로, 안전정지가 항상 이기도록 순서를 고정한다. */
    if (c->ctx.req_rearm != 0u) {           /* DISARM 해제 요청 */
        c->ctx.req_rearm = 0u;
        core_rearm(c);
    }
    if (c->ctx.req_disarm != 0u) {          /* 모듈이 정지 요청 */
        c->ctx.req_disarm = 0u;
        core_transition(c, ST_DISARM);
    }
    if (c->ctx.req_home != 0u) {            /* cmd/home — 스캔 없이 홈만 */
        c->ctx.req_home = 0u;
        if (c->ctx.state != ST_IDLE) {
            core_log(c, "HOME", "%s 상태 — 단독 홈 무시 (IDLE 에서만)",
                     daemon_state_str(c->ctx.state));
        } else if (c->turret_fd < 0) {
            core_log(c, "HOME", "turret 미연결(degraded) — 단독 홈 무시");
        } else if (c->home_manual || (c->ctx.req.valid != 0u)) {
            /* 이미 홈이 돌고 있거나 스캔이 곧 홈을 잡는다. 여기서 다시 걸면
             * home_req_* 가 리셋돼 20초 타임아웃이 처음부터 다시 시작된다. */
            core_log(c, "HOME", "이미 홈 진행 중 — 중복 요청 무시");
        } else {
            c->home_manual = true;          /* core_eval_state 가 수행 */
        }
    }
    if (c->ctx.req_scan_stop != 0u) {       /* 사용자 stop */
        c->ctx.req_scan_stop = 0u;
        if (c->ctx.state == ST_SCANNING) {
            core_scan_abort(c);
            core_transition(c, ST_EXPORT);  /* 여기까지 받은 점으로 파일 마감 */
        }
    }

    for (int i = 0; i < c->n_modules; ++i) {
        const struct daemon_module *m = c->modules[i];
        if (m->on_tick != NULL) {
            m->on_tick(&c->ctx, c->ctx.state);
        }
    }

    core_eval_state(c);          /* 요청/링크 상태 기반 자동 전이 */
}

static void core_on_turret_event(struct core *c)
{
    if (c->ctx.state == ST_SCANNING) {
        core_drain_scan_points(c);   /* POLLIN: 스캔 점 배치 도착 */
    }

    /* ⚠️ 여기서 core_poll_link() 를 부르면 안 된다 — 그 안에 PING 송신이 있어
     *   스캔 중 POLLIN(초당 100회)마다 PING 이 나가 STM 하행이 폭주한다.
     *   실측: STM 메인루프가 PING 처리에 묶여 scan_tick 이 굶고 FIFO 가 넘쳐
     *   점이 뭉텅이로 유실됐다(121/320점, ~210ms 주기 끊김).
     *   PING 주기는 100ms tick 이 소유하고, 여기서는 상태만 읽는다. */
    core_read_state(c);
}

static void core_run(struct core *c)
{
    struct epoll_event events[MAX_EVENTS];
    while (c->running) {
        const int nfds = epoll_wait(c->epoll_fd, events, MAX_EVENTS, -1);
        if (nfds < 0) {
            if (errno == EINTR) {
                continue;
            }
            core_log(c, "LOOP", "epoll_wait 오류: %s", strerror(errno));
            break;
        }
        for (int i = 0; i < nfds; ++i) {
            const int fd = events[i].data.fd;
            if (fd == c->timer_fd) {
                core_tick(c);
            } else if (fd == c->signal_fd) {
                struct signalfd_siginfo si;
                memset(&si, 0, sizeof(si));
                if (read(c->signal_fd, &si, sizeof(si)) > 0) {
                    core_log(c, "SIGNAL", "signal %u 수신 -> 종료", si.ssi_signo);
                }
                c->running = false;
            } else if (c->turret_fd >= 0 && fd == c->turret_fd) {
                core_on_turret_event(c);
            } else {
                for (int j = 0; j < c->n_modules; ++j) {
                    if (c->module_fd[j] == fd && c->modules[j]->on_event != NULL) {
                        c->modules[j]->on_event(&c->ctx);
                        break;
                    }
                }
            }
        }
    }
}

static void core_shutdown(struct core *c)
{
    core_log(c, "SHUTDOWN", "정리 시작");
    if ((c->turret_fd >= 0) && !c->clean_exit) {
        core_transition(c, ST_DISARM);        /* 비정상 종료만 안전 정지 */
    } else if (c->clean_exit) {
        core_log(c, "SHUTDOWN", "정상 완료 — DISARM 생략(되감기 보호)");
    } else {
        /* turret 미연결(degraded) — 보낼 곳이 없다 */
    }
    scan_out_close(&c->out);                  /* 스캔 중이었으면 파일 보존 */
    for (int i = 0; i < c->n_modules; ++i) {
        const struct daemon_module *m = c->modules[i];
        if (m->deinit != NULL) {
            m->deinit(&c->ctx);
        }
    }
    close_fd(&c->turret_fd);
    close_fd(&c->timer_fd);
    close_fd(&c->signal_fd);
    close_fd(&c->epoll_fd);
    core_log(c, "SHUTDOWN", "완료");
}

/* ---------------------------------------------------------------------------
 *  코어 API 구현 (daemon_module.h 선언 — 모듈이 ctx->core 로 호출)
 * ------------------------------------------------------------------------- */
int core_request_state(void *core, daemon_state_t want)
{
    core_transition((struct core *)core, want);
    return 0;
}

void core_hb_reprime(void *core)
{
    struct core *c = core;

    if (c == NULL) {
        return;
    }
    /* 지금을 마지막 PONG 관측 시각으로 삼는다. seq 도 현재값으로 맞춰,
     * 다음 tick 이 "그새 PONG 이 왔다" 로 착각하지 않게 한다. */
    c->hb_last_pong = mono_ms();
    core_log(c, "HB", "heartbeat 기준선 재설정 (블로킹 작업 후)");
}

void core_log(void *core, const char *event, const char *fmt, ...)
{
    (void)core;
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    (void)vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    (void)fprintf(stderr, "[%-8s] %s\n", event, buf);
}

/* ---------------------------------------------------------------------------
 *  CLI (MQTT 없이 스캔을 트리거하기 위한 경로)
 *
 *  정식 경로는 MQTT scan/start 이지만 mqtt_module 이 아직 STUB 이라,
 *  실행 인자로 바로 스캔을 걸 수 있게 열어둔다.
 *  MQTT 가 붙으면 같은 scan_request 를 채우므로 이 경로는 그대로 남겨도 무해하다.
 * ------------------------------------------------------------------------- */
static void usage(const char *p)
{
    (void)fprintf(stderr,
        "사용법:\n"
        "  %s                                   데몬 상주 (MQTT 트리거 대기)\n"
        "  %s --scan <pan0> <pan1> <tilt0> <tilt1> <step> [--height <mm>]\n"
        "       [--lidar-offset <mm>] [--once]\n"
        "\n"
        "  각도는 **기구각**, 단위 0.1도.  pan %d..%d,  tilt %d..%d,  step 10 = 1.0도\n"
        "  --height 지면→라이다 높이(mm). 좌표엔 안 들어가고 메타데이터로만 실린다.\n"
        "  --lidar-offset  회전축 교점→라이다 발광면 거리(mm, 기본 %d).\n"
        "           라이다는 발광면 기준 거리를 주는데 좌표 원점은 축교점이라\n"
        "           r = 보고거리 + 이 값 으로 보정한다. **좌표에 들어간다.**\n"
        "           기구를 바꿔 재조립했을 때만 실측해서 넘기면 된다.\n"
        "  --once   스캔 1회 완료(EXPORT)되면 종료.\n"
        "\n"
        "  ⚠ 2축 스윕은 한 줄이 방위 p 와 p+180 을 같이 훑는다. 그래서 팬은\n"
        "    180도가 아니라 **179도까지**(1도 격자 기준) 돌려야 방위 360도가\n"
        "    정확히 한 번씩 덮인다. 180 까지 돌리면 양끝이 같은 평면이라 중복.\n"
        "\n"
        "예) 방 전체 3D 스캔 (팬 0~179도, 틸트 -90~+90도, 1도 격자, 높이 2400mm):\n"
        "  %s --scan 0 1790 %d %d 10 --height 2400 --once\n",
        p, p, PAN_MIN, PAN_MAX, TILT_MIN, TILT_MAX, LIDAR_RANGE_OFFSET_MM,
        p, TILT_MIN, TILT_MAX);
}

/* 인자 파싱. 스캔 요청이 있으면 req 를 채우고 true.
 *
 * lidar_off 는 req 가 아니라 별도 출력이다 — 스캔 파라미터가 아니라 기구
 * 상수라서(struct core 주석 참조). 호출자가 미리 기본값을 넣어두고 넘긴다. */
static bool parse_args(int argc, char **argv, struct scan_request *req,
                       int32_t *lidar_off, bool *once)
{
    bool have_scan = false;
    *once = false;

    for (int i = 1; i < argc; ++i) {
        if ((strcmp(argv[i], "--scan") == 0) && ((i + 5) < argc)) {
            req->pan_start_ddeg  = (int16_t)atoi(argv[i + 1]);
            req->pan_end_ddeg    = (int16_t)atoi(argv[i + 2]);
            req->tilt_start_ddeg = (int16_t)atoi(argv[i + 3]);
            req->tilt_end_ddeg   = (int16_t)atoi(argv[i + 4]);
            req->step_ddeg       = (uint16_t)atoi(argv[i + 5]);
            have_scan = true;
            i += 5;
        } else if ((strcmp(argv[i], "--height") == 0) && ((i + 1) < argc)) {
            req->sensor_height_mm = (int32_t)atoi(argv[i + 1]);
            i += 1;
        } else if ((strcmp(argv[i], "--lidar-offset") == 0) && ((i + 1) < argc)) {
            *lidar_off = (int32_t)atoi(argv[i + 1]);
            i += 1;
        } else if (strcmp(argv[i], "--once") == 0) {
            *once = true;
        } else if ((strcmp(argv[i], "-h") == 0) || (strcmp(argv[i], "--help") == 0)) {
            usage(argv[0]);
            exit(0);
        } else {
            (void)fprintf(stderr, "알 수 없는 인자: %s\n", argv[i]);
            usage(argv[0]);
            exit(1);
        }
    }
    return have_scan;
}

/* ---------------------------------------------------------------------------
 *  진입점
 * ------------------------------------------------------------------------- */
int main(int argc, char **argv)
{
    (void)fprintf(stderr, "=== ADTS Scanner Daemon (proto v%u, module v%u) ===\n",
                  PROTO_VERSION, DAEMON_MODULE_VERSION);

    struct core core;
    memset(&core, 0, sizeof(core));
    core.epoll_fd  = -1;
    core.turret_fd = -1;
    core.timer_fd  = -1;
    core.signal_fd = -1;
    core.running   = true;
    for (int i = 0; i < MAX_MODULES; ++i) {
        core.module_fd[i] = -1;
    }

    struct scan_request cli_req;
    memset(&cli_req, 0, sizeof(cli_req));
    bool once = false;

    /* 기구 상수라 CLI 스캔이든 MQTT 스캔이든 항상 적용된다. --lidar-offset 은
     * 재조립 후 실측값으로 덮어쓰는 용도. */
    core.lidar_offset_mm = LIDAR_RANGE_OFFSET_MM;

    const bool cli_scan = parse_args(argc, argv, &cli_req,
                                     &core.lidar_offset_mm, &once);

    if (!core_setup(&core)) {
        core_shutdown(&core);
        return 1;
    }

    if (cli_scan) {
        core.ctx.req       = cli_req;
        core.ctx.req.valid = 1u;         /* 첫 tick 에서 FSM 이 SCANNING 으로 전이 */
        core.exit_after_scan = once;
        core_log(&core, "CLI",
                 "스캔 요청: pan[%d..%d] tilt[%d..%d] step=%u z=%dmm "
                 "lidar_offset=%dmm%s",
                 cli_req.pan_start_ddeg, cli_req.pan_end_ddeg,
                 cli_req.tilt_start_ddeg, cli_req.tilt_end_ddeg,
                 cli_req.step_ddeg, cli_req.sensor_height_mm,
                 core.lidar_offset_mm,
                 once ? " (완료 후 종료)" : "");
    }

    core_run(&core);
    core_shutdown(&core);
    /* 스캔이 취소된 채 --once 로 끝났으면 실패로 알린다. 배치 스크립트가
     * 로그를 grep 하지 않고 종료코드만 보면 되도록. */
    return core.scan_failed ? 1 : 0;
}
