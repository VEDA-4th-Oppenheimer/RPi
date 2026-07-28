/* ============================================================================
 *  main.c  --  통합 데몬 코어 (1D LiDAR 스캐너 / 자동 캘리브레이션 킷)
 *  담당: 이현우
 *
 *  - 단일 스레드 epoll 이벤트 루프 (락 불필요, 콜백 블로킹 금지)
 *  - 상태머신 IDLE -> SCANNING -> EXPORT -> IDLE (+ DISARM)  [daemon_module.h v3]
 *  - shared_ctx 소유. 모듈(mqtt/imu/led)을 등록·구동
 *  - /dev/turret 은 코어가 직접 다룬다 (protocol.h v4):
 *      ioctl : HOME / SCAN_START / SCAN_STOP / DISARM / GET_STATE / PING
 *      read(): CMD_SCAN_DATA 스트림 (struct proto_scan_point 배치)
 *    없으면 degraded 모드로 계속 구동 (개발 PC/컨테이너 대응)
 *  - 100ms timerfd tick 에서 heartbeat(PING/300ms link_dead) + 모듈 on_tick
 *  - (pan,tilt,d) -> (x,y,z) 스트리밍 변환 후 포인트클라우드(.pcd) 파일 append
 *
 *  언어: C11 (systems/epoll 은 C API. fd 는 raw int, shutdown 에서 명시 close)
 * ==========================================================================*/
#define PROTO_WANT_IOCTL 1          /* protocol.h 의 ioctl 인터페이스 노출 */

#include "daemon_module.h"
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
#define SCAN_OUT_DIR  "/var/lib/adts/scans"   /* 실패 시 ./scans 로 폴백 */
#define SCAN_BATCH    64            /* read() 1회에 받을 최대 점 수 */

/* 0.1도 -> 라디안. ★ ANGLE_SCALE(=10) 나눗셈을 빠뜨리면 좌표가 통째로 틀어진다. */
#define DDEG2RAD(x)   (((double)(x) / (double)ANGLE_SCALE) * (M_PI / 180.0))

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

    /* 스캔 출력 (포인트클라우드) */
    FILE    *pc_fp;            /* 열려 있으면 스캔 중                */
    char     pc_path[256];
    uint32_t pc_written;       /* 파일에 기록한 점 수                */
};

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
 *  포인트클라우드 출력 (.pcd ascii)
 *
 *   헤더의 POINTS/WIDTH 는 스캔이 끝나야 확정되므로, 자리를 고정폭으로 미리
 *   잡아두고 완료 시 그 자리에 덮어쓴다(파일 재작성 불필요).
 * ------------------------------------------------------------------------- */
#define PCD_COUNT_FIELD_W  10          /* "%-10u" 자리폭 (덮어쓰기용) */

static long pcd_width_off;             /* WIDTH  숫자 시작 오프셋 */
static long pcd_points_off;            /* POINTS 숫자 시작 오프셋 */

static bool pc_open(struct core *c)
{
    /* 출력 디렉토리 준비: 시스템 경로 실패 시 현재 디렉토리로 폴백 */
    const char *dir = SCAN_OUT_DIR;
    if (mkdir(dir, 0755) < 0 && errno != EEXIST) {
        dir = "./scans";
        if (mkdir(dir, 0755) < 0 && errno != EEXIST) {
            core_log(c, "SCAN", "출력 디렉토리 생성 실패: %s", strerror(errno));
            return false;
        }
    }

    time_t     now = time(NULL);
    struct tm  tmv;
    memset(&tmv, 0, sizeof(tmv));
    (void)localtime_r(&now, &tmv);
    char stamp[32];
    (void)strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &tmv);
    (void)snprintf(c->pc_path, sizeof(c->pc_path), "%s/scan_%s.pcd", dir, stamp);

    c->pc_fp = fopen(c->pc_path, "w+");
    if (c->pc_fp == NULL) {
        core_log(c, "SCAN", "파일 생성 실패 %s: %s", c->pc_path, strerror(errno));
        return false;
    }

    /* PCD ascii 헤더. 좌표계: 원점=천장 팬/틸트 축 교점, 단위=mm (ICD) */
    (void)fprintf(c->pc_fp,
        "# .PCD v0.7 - adts scan\n"
        "# origin = pan/tilt axis intersection (ceiling), unit = mm\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n");
    (void)fflush(c->pc_fp);
    (void)fputs("WIDTH ", c->pc_fp);
    pcd_width_off = ftell(c->pc_fp);
    (void)fprintf(c->pc_fp, "%-*u\n", PCD_COUNT_FIELD_W, 0u);
    (void)fprintf(c->pc_fp, "HEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n");
    (void)fputs("POINTS ", c->pc_fp);
    pcd_points_off = ftell(c->pc_fp);
    (void)fprintf(c->pc_fp, "%-*u\n", PCD_COUNT_FIELD_W, 0u);
    (void)fprintf(c->pc_fp, "DATA ascii\n");

    c->pc_written = 0u;
    core_log(c, "SCAN", "포인트클라우드 열기: %s", c->pc_path);
    return true;
}

/* (pan,tilt,d) -> (x,y,z) 변환 후 1점 기록.
 *   x = d·cos(tilt)·cos(pan),  y = d·cos(tilt)·sin(pan),  z = d·sin(tilt)
 *   각 점이 서로 독립이라 버퍼 누적 없이 받는 즉시 스트리밍 변환한다. */
static void pc_write_point(struct core *c, const struct proto_scan_point *p)
{
    if (c->pc_fp == NULL) {
        return;
    }
    const double pan  = DDEG2RAD(p->pan_ddeg);
    const double tilt = DDEG2RAD(p->tilt_ddeg);
    const double d    = (double)p->d_mm;
    const double ct   = cos(tilt);

    const double x = d * ct * cos(pan);
    const double y = d * ct * sin(pan);
    const double z = d * sin(tilt);

    (void)fprintf(c->pc_fp, "%.1f %.1f %.1f\n", x, y, z);
    c->pc_written++;
}

/* 헤더의 WIDTH/POINTS 를 실제 점 수로 덮어쓰고 파일을 닫는다. */
static void pc_close(struct core *c)
{
    if (c->pc_fp == NULL) {
        return;
    }
    (void)fflush(c->pc_fp);
    if (fseek(c->pc_fp, pcd_width_off, SEEK_SET) == 0) {
        (void)fprintf(c->pc_fp, "%-*u", PCD_COUNT_FIELD_W, c->pc_written);
    }
    if (fseek(c->pc_fp, pcd_points_off, SEEK_SET) == 0) {
        (void)fprintf(c->pc_fp, "%-*u", PCD_COUNT_FIELD_W, c->pc_written);
    }
    (void)fclose(c->pc_fp);
    c->pc_fp = NULL;
    core_log(c, "SCAN", "포인트클라우드 마감: %s (%u점)", c->pc_path, c->pc_written);
}

/* ---------------------------------------------------------------------------
 *  스캔 제어 (/dev/turret ioctl)
 * ------------------------------------------------------------------------- */

/* 요청 범위·격자로 예상 점 수 산출 (진행률 표시용) */
static uint32_t scan_expected_points(const struct scan_request *r)
{
    if (r->step_ddeg == 0u) {
        return 0u;
    }
    const int32_t pan_span  = (int32_t)r->pan_end_ddeg  - (int32_t)r->pan_start_ddeg;
    const int32_t tilt_span = (int32_t)r->tilt_end_ddeg - (int32_t)r->tilt_start_ddeg;
    const int32_t pan_n  = (pan_span  < 0 ? -pan_span  : pan_span)  / (int32_t)r->step_ddeg + 1;
    const int32_t tilt_n = (tilt_span < 0 ? -tilt_span : tilt_span) / (int32_t)r->step_ddeg + 1;
    return (uint32_t)(pan_n * tilt_n);
}

/* 요청값이 protocol.h 의 각도 규약 안에 있는지 검사 (드라이버도 재검증한다) */
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

/* SCAN_START 하달: 수평 게이트 -> 파일 열기 -> ioctl. 실패 시 false */
static bool core_scan_begin(struct core *c)
{
    const struct scan_request *r = &c->ctx.req;

    if (!scan_request_valid(r)) {
        core_log(c, "SCAN", "잘못된 스캔 요청 (범위/격자) — 거부");
        return false;
    }
    if (!level_gate_ok(c)) {
        return false;
    }
    if (!pc_open(c)) {
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
            pc_close(c);
            return false;
        }
    } else {
        core_log(c, "SCAN", "degraded — turret 없이 파일만 준비");
    }

    memset(&c->ctx.progress, 0, sizeof(c->ctx.progress));
    c->ctx.progress.expected = scan_expected_points(r);
    memset(&c->ctx.result, 0, sizeof(c->ctx.result));

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
            pc_write_point(c, &batch[i]);
        }
        c->ctx.progress.points = c->pc_written;
        if (c->ctx.progress.expected > 0u) {
            const uint64_t pct = (uint64_t)c->pc_written * 100u / c->ctx.progress.expected;
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
static void core_poll_link(struct core *c)
{
    if (c->turret_fd < 0) {
        return;                          /* degraded: STM 링크 없음 */
    }

    (void)ioctl(c->turret_fd, TURRET_PING);   /* 실패는 아래 타임아웃이 흡수 */

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
    if (st.last_err != ERR_NONE && c->ctx.state == ST_SCANNING) {
        core_log(c, "STM", "CMD_ERROR code=%u -> DISARM", st.last_err);
        core_transition(c, ST_DISARM);
    }
}

/* ---------------------------------------------------------------------------
 *  상태 평가 / 전이
 * ------------------------------------------------------------------------- */
static void core_eval_state(struct core *c)
{
    switch (c->ctx.state) {
    case ST_IDLE:
        if (c->ctx.req.valid != 0u) {                 /* MQTT scan/start 수신 */
            c->ctx.req.valid = 0u;                    /* 요청 소비 */
            core_transition(c, ST_SCANNING);
        }
        break;

    case ST_SCANNING:
        /* STM 이 스캔을 끝냈으면(STF_SCANNING 내려감) EXPORT 로.
         * degraded(turret 없음)에서는 자동 전이하지 않는다. */
        if (c->turret_fd >= 0 && c->ctx.link.scanning == 0u &&
            c->ctx.progress.points > 0u) {
            core_transition(c, ST_EXPORT);
        }
        break;

    case ST_EXPORT:
        /* 파일 마감은 on_state 진입 시 처리됨. 곧바로 IDLE 로 복귀. */
        core_transition(c, ST_IDLE);
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
        pc_close(c);
        (void)snprintf(c->ctx.result.path, sizeof(c->ctx.result.path), "%s", c->pc_path);
        c->ctx.result.point_count = c->pc_written;
        c->ctx.result.valid       = 1u;
        core_log(c, "EXPORT", "%s (%u점) — 카메라 단 전달 대기",
                 c->ctx.result.path, c->ctx.result.point_count);
    } else if (want == ST_DISARM) {
        if (c->turret_fd >= 0) {
            (void)ioctl(c->turret_fd, TURRET_DISARM);
        }
        pc_close(c);                          /* 스캔 중이었으면 파일 마감 */
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

static void core_tick(struct core *c)
{
    uint64_t expirations = 0;
    if (read(c->timer_fd, &expirations, sizeof(expirations)) < 0) {
        /* EAGAIN 등은 무시 */
    }

    core_poll_link(c);           /* STM 링크 캐시 + heartbeat 판정 */

    if (c->ctx.req_disarm != 0u) {          /* 모듈이 정지 요청 */
        c->ctx.req_disarm = 0u;
        core_transition(c, ST_DISARM);
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
    core_poll_link(c);               /* POLLERR(link_dead) 포함 상태 재확인 */
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
    if (c->turret_fd >= 0) {
        core_transition(c, ST_DISARM);        /* 안전 정지 */
    }
    pc_close(c);                              /* 스캔 중이었으면 파일 보존 */
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
 *  진입점
 * ------------------------------------------------------------------------- */
int main(void)
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

    if (!core_setup(&core)) {
        core_shutdown(&core);
        return 1;
    }
    core_run(&core);
    core_shutdown(&core);
    return 0;
}
