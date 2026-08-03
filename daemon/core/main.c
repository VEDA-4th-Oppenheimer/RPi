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

/* JSON 인터페이스 계약 버전 (PAN_TILT_LIDAR_JSON_INTERFACE.md) */
#define JSON_IFACE_VERSION  "1.0"

/* ---------------------------------------------------------------------------
 *  좌표계 (PAN_TILT_LIDAR_JSON_INTERFACE.md 확정, 2026-07-29)
 *
 *    +x: right   +y: down   +z: forward
 *    pan positive: right    tilt positive: up
 *
 *      x =  range · cos(tilt) · sin(pan)
 *      y = -range · sin(tilt)
 *      z =  range · cos(tilt) · cos(pan)
 *
 *  ⚠️ 이전 ICD(z-up: x=d·cosφ·cosθ, y=d·cosφ·sinθ, z=d·sinφ)와 **축이 다르다**.
 *    2026-07-29 이전에 생성된 .pcd 는 옛 축이므로 섞어 쓰지 말 것.
 *  ⚠️ 단위도 mm → **meter** 로 변경(문서 01/02 계약).
 *  ⚠️ 센서 높이(z_offset)는 좌표에 **적용하지 않는다** — frame 이 lidar_scan
 *    (원점=센서)이기 때문. 높이는 scan.sensor_height_m 메타데이터로만 전달.
 * ------------------------------------------------------------------------- */

/* organized 격자 한 칸. filled=false 면 미측정(JSON null / PCD NaN). */
struct scan_cell {
    uint32_t seq;              /* sweep 내 수신 순번        */
    uint64_t rx_ns;            /* RPi 수신 시각(mono ns)    */
    int16_t  pan_ddeg;
    int16_t  tilt_ddeg;
    uint16_t d_mm;
    /* v5 원시 품질 필드 (라이다 프레임 원본 그대로) */
    uint16_t signal_strength;
    uint32_t device_time_ms;
    uint32_t stm_ts_ms;
    uint8_t  dis_status;
    uint8_t  range_precision;
    bool     filled;
};

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

    /* 스캔 출력 — organized 격자 버퍼링 후 JSON + PCD 동시 산출.
     *
     * 스트리밍 append 가 아니라 격자에 담는 이유:
     *   ① (row,column) 중복 없이 organized 로 내려면 셀 단위 배치가 필요
     *   ② 미측정 셀을 "구멍"이 아니라 null/NaN 으로 명시해야 캘리브 쪽이
     *      "벽 없음"과 "측정 실패"를 구분할 수 있다
     *   ③ PCD 도 organized(WIDTH=columns, HEIGHT=rows)가 되어 문서 01/02 충족
     * 메모리: 1축 360셀≈9KB / 2축 180×360=64,800셀≈1.6MB (RPi4 여유) */
    struct scan_cell *grid;    /* NULL 이면 스캔 중 아님 */
    uint32_t grid_rows;
    uint32_t grid_cols;
    uint32_t pc_written;       /* 격자에 채운 유효 점 수 */
    uint32_t drop_dup;         /* 같은 셀에 중복 도착 (버려짐) */
    uint32_t drop_range;       /* 격자 범위 밖 각도 */
    /* dis_status 분포 — Datasheet(0=invalid,1=valid) 와 User Manual 예제가
     * 정반대라 실측으로 판별해야 한다. 인덱스 = status 값(0~3), 그 외는 [3]. */
    uint32_t status_hist[4];
    uint64_t scan_start_ns;
    uint64_t scan_end_ns;
    char     session_id[32];
    char     scan_id[32];
    char     pc_path[256];     /* .pcd 경로  */
    char     js_path[256];     /* .json 경로 */

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

    /* 무입력 타임아웃 (SCAN_DONE 유실 대비 안전망) */
    uint64_t last_point_ms;        /* 마지막 점 수신 시각 */
};

/* SCAN_DONE 을 놓쳤을 때를 대비한 안전망: 이 시간 동안 점이 하나도 안 오면 종료.
 * 라이다 100Hz(10ms) 대비 충분히 길고, 줄 끝 방향전환보다도 길게 잡는다. */
#define SCAN_IDLE_TIMEOUT_MS   3000u

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
static uint64_t mono_ns(void)
{
    struct timespec ts;
    memset(&ts, 0, sizeof(ts));
    (void)clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000u + (uint64_t)ts.tv_nsec;
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
/* 격자 크기 산출.
 *   columns = 팬 스윕 폭 / 격자간격,  rows = 틸트 줄 수
 *   ordering: row 0 = tilt_max(마지막 row = tilt_min), column 0 = pan_min */
static void grid_dims(const struct scan_request *r, uint32_t *rows, uint32_t *cols)
{
    const int32_t step = (r->step_ddeg > 0u) ? (int32_t)r->step_ddeg : 10;

    int32_t pan_span = (int32_t)r->pan_end_ddeg - (int32_t)r->pan_start_ddeg;
    if (pan_span <= 0) {
        pan_span += 3600;                       /* 0→0 은 한 바퀴 */
    }
    int32_t tilt_span = (int32_t)r->tilt_end_ddeg - (int32_t)r->tilt_start_ddeg;
    if (tilt_span < 0) {
        tilt_span = -tilt_span;
    }

    *cols = (uint32_t)(pan_span / step) + 1u;
    *rows = (uint32_t)(tilt_span / step) + 1u;
}

/* 측정 각도 → 격자 셀. 범위를 벗어나면 false. */
static bool grid_index(const struct core *c, const struct proto_scan_point *p,
                       uint32_t *row, uint32_t *col)
{
    const struct scan_request *r = &c->ctx.req;
    const int32_t step = (r->step_ddeg > 0u) ? (int32_t)r->step_ddeg : 10;

    int32_t dpan = (int32_t)p->pan_ddeg - (int32_t)r->pan_start_ddeg;
    if (dpan < 0) {
        dpan += 3600;                           /* 랩어라운드 */
    }
    const int32_t ci = (dpan + (step / 2)) / step;   /* 반올림 */

    /* row 0 = tilt_max 이므로 위에서부터 센다. */
    const int32_t tmax = (r->tilt_end_ddeg > r->tilt_start_ddeg)
                       ? r->tilt_end_ddeg : r->tilt_start_ddeg;
    const int32_t ri = ((int32_t)tmax - (int32_t)p->tilt_ddeg + (step / 2)) / step;

    bool ok = false;
    if ((ci >= 0) && ((uint32_t)ci < c->grid_cols) &&
        (ri >= 0) && ((uint32_t)ri < c->grid_rows)) {
        *col = (uint32_t)ci;
        *row = (uint32_t)ri;
        ok = true;
    }
    return ok;
}

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
    char stamp[24];
    (void)strftime(stamp, sizeof(stamp), "%Y%m%d-%H%M%S", &tmv);

    (void)snprintf(c->session_id, sizeof(c->session_id), "calib-%s", stamp);
    (void)snprintf(c->scan_id,    sizeof(c->scan_id),    "sweep-000001");
    (void)snprintf(c->pc_path, sizeof(c->pc_path), "%s/%s_%s.pcd",
                   dir, c->session_id, c->scan_id);
    (void)snprintf(c->js_path, sizeof(c->js_path), "%s/%s_%s_pan_tilt_lidar.json",
                   dir, c->session_id, c->scan_id);

    grid_dims(&c->ctx.req, &c->grid_rows, &c->grid_cols);

    const size_t n = (size_t)c->grid_rows * (size_t)c->grid_cols;
    c->grid = calloc(n, sizeof(struct scan_cell));
    if (c->grid == NULL) {
        core_log(c, "SCAN", "격자 할당 실패 (%ux%u)", c->grid_rows, c->grid_cols);
        return false;
    }

    c->pc_written    = 0u;
    c->drop_dup      = 0u;
    c->drop_range    = 0u;
    memset(c->status_hist, 0, sizeof(c->status_hist));
    c->scan_start_ns = mono_ns();
    c->scan_end_ns   = c->scan_start_ns;

    core_log(c, "SCAN", "격자 %ux%u (%zu셀) — %s", c->grid_rows, c->grid_cols, n,
             c->session_id);
    return true;
}

/* (pan,tilt,d) -> (x,y,z) 변환 후 1점 기록.
 *   x = d·cos(tilt)·cos(pan)
 *   y = d·cos(tilt)·sin(pan)
 *   z = d·sin(tilt) + z_offset_mm
 *
 *   각 점이 서로 독립이라 버퍼 누적 없이 받는 즉시 스트리밍 변환한다.
 *   z_offset 은 1축 임시 스캔(책 쌓기)에서 레이어 높이를 담는다.
 *   2축에서는 0 이므로 원래 공식과 동일하다. */
static void pc_write_point(struct core *c, const struct proto_scan_point *p)
{
    uint32_t row = 0u;
    uint32_t col = 0u;

    if (c->grid == NULL) {
        return;
    }
    if (!grid_index(c, p, &row, &col)) {
        c->drop_range++;                 /* 요청 범위 밖 각도 */
        return;
    }

    struct scan_cell *cell = &c->grid[((size_t)row * c->grid_cols) + col];
    if (cell->filled) {
        /* 같은 셀에 두 번 도착 — 먼저 온 값을 유지한다.
         * (스윕 속도가 격자보다 조밀하면 정상적으로 발생) */
        c->drop_dup++;
        return;
    }

    cell->seq             = c->pc_written;
    cell->rx_ns           = mono_ns();
    cell->pan_ddeg        = p->pan_ddeg;
    cell->tilt_ddeg       = p->tilt_ddeg;
    cell->d_mm            = p->d_mm;
    cell->signal_strength = p->signal_strength;
    cell->device_time_ms  = p->device_time_ms;
    cell->stm_ts_ms       = p->stm_ts_ms;
    cell->dis_status      = p->dis_status;
    cell->range_precision = p->range_precision;
    cell->filled          = true;
    c->status_hist[(p->dis_status < 3u) ? p->dis_status : 3u]++;

    c->scan_end_ns = cell->rx_ns;
    c->pc_written++;
}

/* organized PCD 출력 (변환 후 x/y/z, meter).
 * 미측정 셀은 nan 으로 남겨 구멍과 실패를 구분 가능하게 한다. */
static void write_pcd(struct core *c)
{
    FILE *fp = fopen(c->pc_path, "w");
    if (fp == NULL) {
        core_log(c, "SCAN", "PCD 생성 실패 %s: %s", c->pc_path, strerror(errno));
        return;
    }

    const size_t n = (size_t)c->grid_rows * (size_t)c->grid_cols;

    /* ⚠️ 센서 높이(z_offset)를 좌표에 **적용하지 않는다**.
     *   frame 이름이 lidar_scan 이면 원점은 라이다 자신이므로, tilt=0 인 점의
     *   y 는 0 이어야 한다. 예전에 높이를 빼서 모든 y 가 -1.2m 로 찍혔는데,
     *   그건 사실상 actuator_base 계열 좌표라 라벨과 불일치였다(2026-07-29 수정).
     *   높이는 scan.sensor_height_m 메타데이터로만 전달하고, 레이어 적층 같은
     *   변환은 소비자(또는 json2pcd 도구의 옵션)가 수행한다. */

    (void)fprintf(fp,
        "# .PCD v0.7 - adts scan (organized)\n"
        "# frame = lidar_scan (origin = sensor)  +x right +y down +z forward  unit = meter\n"
        "# sensor_height_m = %.4f (좌표에 미적용 — 메타데이터)\n"
        "# session=%s scan=%s\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        "WIDTH %u\n"
        "HEIGHT %u\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        "POINTS %zu\n"
        "DATA ascii\n",
        (double)c->ctx.req.z_offset_mm / 1000.0,
        c->session_id, c->scan_id, c->grid_cols, c->grid_rows, n);

    for (size_t i = 0; i < n; ++i) {
        const struct scan_cell *cell = &c->grid[i];
        if (!cell->filled) {
            (void)fputs("nan nan nan\n", fp);
        } else {
            const double pan  = DDEG2RAD(cell->pan_ddeg);
            const double tilt = DDEG2RAD(cell->tilt_ddeg);
            const double r    = (double)cell->d_mm / 1000.0;    /* mm → m */
            const double ct   = cos(tilt);

            (void)fprintf(fp, "%.4f %.4f %.4f\n",
                          r * ct * sin(pan),
                          -r * sin(tilt),
                          r * ct * cos(pan));
        }
    }
    (void)fclose(fp);
}

/* 원시 측정 JSON 출력 (변환 전 — 계약상 golden reference).
 * x/y/z 는 넣지 않는다. 캘리브 adapter 가 distance/pan/tilt 로 직접 계산한다. */
static void write_json(struct core *c)
{
    FILE *fp = fopen(c->js_path, "w");
    if (fp == NULL) {
        core_log(c, "SCAN", "JSON 생성 실패 %s: %s", c->js_path, strerror(errno));
        return;
    }

    const struct scan_request *rq = &c->ctx.req;
    const size_t n = (size_t)c->grid_rows * (size_t)c->grid_cols;

    (void)fprintf(fp,
        "{\n"
        "  \"interface_version\": \"%s\",\n"
        "  \"schema_version\": \"1.1\",\n"
        "  \"session_id\": \"%s\",\n"
        "  \"scan_id\": \"%s\",\n"
        "  \"producer\": { \"software\": \"adts_daemon\", \"protocol_version\": %u },\n"
        "  \"sensor\": { \"model\": \"TOFSense-F2P\", \"lidar_rate_hz\": 100 },\n"
        "  \"frame\": {\n"
        "    \"name\": \"lidar_scan\",\n"
        "    \"handedness\": \"right\",\n"
        "    \"convention\": \"+x right, +y down, +z forward; pan+ right, tilt+ up\"\n"
        "  },\n"
        "  \"units\": { \"distance\": \"meter\", \"angle\": \"radian\", \"timestamp\": \"nanosecond\" },\n"
        "  \"scan\": {\n"
        "    \"mode\": \"continuous_pan_sweep\",\n"
        "    \"rows\": %u,\n"
        "    \"columns\": %u,\n"
        "    \"pan_min_rad\": %.6f,\n"
        "    \"pan_max_rad\": %.6f,\n"
        "    \"tilt_min_rad\": %.6f,\n"
        "    \"tilt_max_rad\": %.6f,\n"
        "    \"grid_step_rad\": %.6f,\n"
        "    \"sensor_height_m\": %.4f,\n"
        "    \"sample_count\": %zu,\n"
        "    \"valid_count\": %u,\n"
        "    \"started_at_ns\": %llu,\n"
        "    \"ended_at_ns\": %llu\n"
        "  },\n"
        "  \"diagnostics\": {\n"
        "    \"checksum_error_count\": 0,\n"
        "    \"duplicate_cell_count\": %u,\n"
        "    \"out_of_range_angle_count\": %u,\n"
        "    \"encoder_gap_count\": 0,\n"
        "    \"dis_status_histogram\": "
        "{ \"0\": %u, \"1\": %u, \"2\": %u, \"other\": %u }\n"
        "  },\n"
        "  \"measurements\": [\n",
        JSON_IFACE_VERSION, c->session_id, c->scan_id, (unsigned)PROTO_VERSION,
        c->grid_rows, c->grid_cols,
        DDEG2RAD(rq->pan_start_ddeg),  DDEG2RAD(rq->pan_end_ddeg),
        DDEG2RAD(rq->tilt_start_ddeg), DDEG2RAD(rq->tilt_end_ddeg),
        DDEG2RAD((int)rq->step_ddeg),
        (double)rq->z_offset_mm / 1000.0,
        n, c->pc_written,
        (unsigned long long)c->scan_start_ns,
        (unsigned long long)c->scan_end_ns,
        c->drop_dup, c->drop_range,
        c->status_hist[0], c->status_hist[1],
        c->status_hist[2], c->status_hist[3]);

    for (size_t i = 0; i < n; ++i) {
        const struct scan_cell *cell = &c->grid[i];
        const uint32_t row = (uint32_t)(i / c->grid_cols);
        const uint32_t col = (uint32_t)(i % c->grid_cols);
        const char    *sep = (i + 1u < n) ? "," : "";

        if (!cell->filled) {
            /* 미측정 셀 — 구멍이 아니라 명시적 null 로 남긴다. */
            (void)fprintf(fp,
                "    { \"sequence\": null, \"row\": %u, \"column\": %u,"
                " \"timestamp_ns\": null, \"device_time_ms\": null,"
                " \"stm32_time_ms\": null,"
                " \"encoder_timestamp_ns\": null,"
                " \"pan_rad\": null, \"tilt_rad\": null,"
                " \"pan_encoder_count\": null, \"tilt_encoder_count\": null,"
                " \"distance_m\": null, \"distance_status\": null,"
                " \"signal_strength\": null, \"range_precision_raw\": null,"
                " \"range_precision_m\": null,"
                " \"checksum_valid\": null,"
                " \"angle_source\": null,"
                " \"timestamp_source\": null,"
                " \"encoder_interpolation_valid\": null,"
                " \"valid\": false, \"quality_flags\": [\"NO_MEASUREMENT\"] }%s\n",
                row, col, sep);
        } else {
            /* timestamp_ns = STM32 래치 시각(ms 해상도)을 ns 로 확장.
             *   RPi 수신 시각보다 정확하다(UART 큐잉 지연이 안 섞임).
             *   ⚠️ 단 STM32 clock domain 이라 host 와 offset 미보정 →
             *      quality_flags 에 TIMESTAMP_STM_CLOCK 을 남긴다.
             * ⚠️ 아직 null 인 필드: encoder_count / encoder_timestamp
             *   → 강유근 MT6701 엔코더 펌웨어 구현 후 채움.
             * range_precision (User Manual IIC 0x2C): **cm 단위**,
             *   0x00 = <1cm, 0xFF = >=255cm.
             *
             * ⚠️ 실측(2026-07-29): F2 P 는 359/359 전부 **0xFF** 를 보낸다.
             *   매뉴얼 §7.3.4 "If there is no corresponding parameter in the
             *   register, the default output is 0xff" 에 따라 **이 모델이
             *   지원하지 않는 필드**로 판단. 0.7m 측정에 정밀도 >=2.55m 는
             *   스펙(±3cm)과 모순이므로 값으로 쓰면 안 된다.
             *   → 0xFF 는 range_precision_m 을 null 로 두고 flag 로 표시.
             *   → 02 문서 §7.1 의 depth edge 분모 sigma 는 이 필드 대신
             *      Datasheet 거리구간별 표준편차(<1cm@[0.05,10]m,
             *      <6cm@[10,25]m) 또는 반복측정 실측 분산을 써야 한다. */
            /* 0xFF = 미지원/포화 → m 값을 만들지 않고 flag 로 알린다. */
            char rp_m[24];
            const char *rp_flag;
            if (cell->range_precision == 0xFFu) {
                (void)snprintf(rp_m, sizeof(rp_m), "null");
                rp_flag = ",\"RANGE_PRECISION_NA\"";
            } else {
                (void)snprintf(rp_m, sizeof(rp_m), "%.2f",
                               (double)cell->range_precision / 100.0);
                rp_flag = "";
            }
            (void)fprintf(fp,
                "    { \"sequence\": %u, \"row\": %u, \"column\": %u,"
                " \"timestamp_ns\": %llu, \"device_time_ms\": %u,"
                " \"stm32_time_ms\": %u,"
                " \"encoder_timestamp_ns\": null,"
                " \"pan_rad\": %.6f, \"tilt_rad\": %.6f,"
                " \"pan_encoder_count\": null, \"tilt_encoder_count\": null,"
                " \"distance_m\": %.4f, \"distance_status\": %u,"
                " \"signal_strength\": %u, \"range_precision_raw\": %u,"
                " \"range_precision_m\": %s,"
                " \"checksum_valid\": true,"
                " \"angle_source\": \"step_count\","
                " \"timestamp_source\": \"host_rx_monotonic\","
                " \"encoder_interpolation_valid\": null,"
                " \"valid\": true,"
                " \"quality_flags\": [\"VALID_RANGE\"%s] }%s\n",
                cell->seq, row, col,
                (unsigned long long)cell->rx_ns,
                (unsigned)cell->device_time_ms,
                (unsigned)cell->stm_ts_ms,
                DDEG2RAD(cell->pan_ddeg), DDEG2RAD(cell->tilt_ddeg),
                (double)cell->d_mm / 1000.0,
                (unsigned)cell->dis_status,
                (unsigned)cell->signal_strength,
                (unsigned)cell->range_precision,
                rp_m, rp_flag, sep);
        }
    }

    (void)fprintf(fp, "  ]\n}\n");
    (void)fclose(fp);
}

/* 격자를 두 포맷으로 산출하고 해제한다. */
static void pc_close(struct core *c)
{
    if (c->grid == NULL) {
        return;
    }
    write_json(c);      /* 변환 전 원시 (계약) */
    write_pcd(c);       /* 변환 후 x/y/z (뷰어·편의) */

    core_log(c, "SCAN", "산출 완료 %ux%u — 유효 %u점 (중복 %u, 범위밖 %u)",
             c->grid_rows, c->grid_cols, c->pc_written, c->drop_dup, c->drop_range);
    core_log(c, "SCAN", "  JSON: %s", c->js_path);
    core_log(c, "SCAN", "  PCD : %s", c->pc_path);

    free(c->grid);
    c->grid = NULL;
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
            pc_write_point(c, &batch[i]);
        }
        c->last_point_ms = mono_ms();      /* 무입력 타임아웃 기준 갱신 */
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
            const bool idle_timeout =
                (c->ctx.progress.points > 0u) &&
                ((mono_ms() - c->last_point_ms) > SCAN_IDLE_TIMEOUT_MS);

            if (done_sig) {
                core_transition(c, ST_EXPORT);
            } else if (idle_timeout) {
                core_log(c, "SCAN", "무입력 %ums — SCAN_DONE 없이 마감",
                         SCAN_IDLE_TIMEOUT_MS);
                core_transition(c, ST_EXPORT);
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
 *  CLI (MQTT 없이 스캔을 트리거하기 위한 경로)
 *
 *  정식 경로는 MQTT scan/start 이지만 mqtt_module 이 아직 STUB 이라,
 *  1축 임시 스캔 데이터를 뽑기 위해 실행 인자로 바로 스캔을 건다.
 *  MQTT 가 붙으면 같은 scan_request 를 채우므로 이 경로는 그대로 남겨도 무해하다.
 * ------------------------------------------------------------------------- */
static void usage(const char *p)
{
    (void)fprintf(stderr,
        "사용법:\n"
        "  %s                                   데몬 상주 (MQTT 트리거 대기)\n"
        "  %s --scan <pan0> <pan1> <tilt0> <tilt1> <step> [--z <mm>] [--once]\n"
        "\n"
        "  각도 단위 = 0.1도.  pan 0..%d,  tilt %d..%d,  step 10 = 1.0도\n"
        "  --z    지면에서 라이다까지 높이(mm). 1축 스캔에서 레이어 z 로 쓰인다.\n"
        "  --once 스캔 1회 완료(EXPORT)되면 종료. 레이어별 실행에 편리.\n"
        "\n"
        "예) 팬 한 바퀴, 틸트 0 고정, 1도 격자, 라이다 높이 1200mm:\n"
        "  %s --scan 0 %d 0 0 10 --z 1200 --once\n",
        p, p, PAN_MAX, TILT_MIN, TILT_MAX, p, PAN_MAX);
}

/* 인자 파싱. 스캔 요청이 있으면 req 를 채우고 true. */
static bool parse_args(int argc, char **argv, struct scan_request *req, bool *once)
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
        } else if ((strcmp(argv[i], "--z") == 0) && ((i + 1) < argc)) {
            req->z_offset_mm = (int32_t)atoi(argv[i + 1]);
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
    const bool cli_scan = parse_args(argc, argv, &cli_req, &once);

    if (!core_setup(&core)) {
        core_shutdown(&core);
        return 1;
    }

    if (cli_scan) {
        core.ctx.req       = cli_req;
        core.ctx.req.valid = 1u;         /* 첫 tick 에서 FSM 이 SCANNING 으로 전이 */
        core.exit_after_scan = once;
        core_log(&core, "CLI", "스캔 요청: pan[%d..%d] tilt[%d..%d] step=%u z=%dmm%s",
                 cli_req.pan_start_ddeg, cli_req.pan_end_ddeg,
                 cli_req.tilt_start_ddeg, cli_req.tilt_end_ddeg,
                 cli_req.step_ddeg, cli_req.z_offset_mm,
                 once ? " (완료 후 종료)" : "");
    }

    core_run(&core);
    core_shutdown(&core);
    return 0;
}
