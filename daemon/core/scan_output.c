/* ============================================================================
 *  scan_output.c  --  스캔 산출물 생성 구현
 * ----------------------------------------------------------------------------
 *  담당: 이현우.  계약과 분리 근거는 scan_output.h 상단 참조.
 *
 *  주의: 여기엔 epoll / ioctl / fd / 상태머신이 없다. 그래야 호스트에서 그대로
 *    링크해 격자·좌표변환을 검증할 수 있다. 리눅스 전용 API 를 들여오지 말 것.
 * ==========================================================================*/
#include "scan_output.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <time.h>
#include <sys/stat.h>
#include <unistd.h>        /* unlink — 쓰기 가능 확인용 probe 파일 제거 */

#define SCAN_OUT_DIR  "/var/lib/adts/scans"   /* 실패 시 ./scans 로 폴백 */

/* JSON 인터페이스 계약 버전 (PAN_TILT_LIDAR_JSON_INTERFACE.md) */
#define JSON_IFACE_VERSION  "1.0"

/* 0.1도 -> 라디안. 핵심: ANGLE_SCALE(=10) 나눗셈을 빠뜨리면 좌표가 통째로 틀어진다. */
#define DDEG2RAD(x)   (((double)(x) / (double)ANGLE_SCALE) * (M_PI / 180.0))

static uint64_t mono_ns(void)
{
    struct timespec ts;
    memset(&ts, 0, sizeof(ts));
    (void)clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000u + (uint64_t)ts.tv_nsec;
}

/* organized 격자 한 칸. filled=false 면 미측정(JSON null / PCD NaN). */
struct scan_cell {
    uint32_t seq;              /* sweep 내 수신 순번        */
    uint64_t rx_ns;            /* RPi 수신 시각(mono ns)    */
    int16_t  pan_ddeg;         /* 계약 방위 (변환 후)       */
    int16_t  tilt_ddeg;        /* 계약 고각 (변환 후)       */
    int16_t  mech_pan_ddeg;    /* 기구 방위 (STM 원본)      */
    int16_t  mech_tilt_ddeg;   /* 기구 고각 (STM 원본)      */
    uint16_t d_mm;
    /* v5 원시 품질 필드 (라이다 프레임 원본 그대로) */
    uint16_t signal_strength;
    uint32_t device_time_ms;
    uint32_t stm_ts_ms;
    uint8_t  dis_status;
    uint8_t  range_precision;
    bool     filled;

    /* --- 셀 내 다중 샘플 병합 -------------------------------------------
     * 틸트 45도/s + 라이다 100Hz = 0.45도/샘플 이므로 0.9도 격자에는
     * 샘플이 2개씩 떨어진다. 위 필드들(각도·품질·시각)은 **셀 중심에
     * 가장 가까운 샘플**의 것이고, 거리만 따로 누적해 둔다. */
    uint32_t d_sum_mm;         /* 도착한 샘플 거리의 합                  */
    uint16_t d_min_mm;
    uint16_t d_max_mm;
    uint16_t best_off_ddeg;    /* 대표 샘플이 셀 중심에서 벗어난 정도    */
    uint8_t  n_samples;        /* 이 셀에 도착한 샘플 수 (255 에서 포화) */
};

/* 셀 안의 샘플들을 평균해도 되는지 판정하는 산포 허용치.
 *
 * 평평한 면에서는 두 샘플이 잡음(σ<1cm)만큼만 다르므로 평균이 이득이다
 * (2개 평균 → σ/√2). 그러나 셀이 **깊이 불연속**을 물면 두 샘플이 앞뒤
 * 서로 다른 표면을 재고, 그걸 평균하면 **허공에 없는 점이 생긴다.**
 * 우리 미션이 구조 에지 기반 캘리브라 이 가짜 점이 정확도보다 훨씬 해롭다.
 *
 * 상대 허용치를 같이 두는 이유: 스치듯 비스듬한 면(천장에서 본 바닥 등)은
 * 한 셀 안에서도 거리차가 원래 크다. 절대치만 쓰면 정작 평균이 필요한
 * 자리에서 전부 거부된다. 0.9도 × 10m = 16cm 폭이고 입사각 80도면 정상
 * 깊이차가 45cm 까지 나온다 — 반면 진짜 에지는 보통 m 단위로 뛴다. */
#define CELL_AVG_ABS_TOL_MM    30u    /* 절대 하한                        */
#define CELL_AVG_REL_TOL_PCT   15u    /* 가까운 쪽 거리 대비 허용 비율(%) */

/* ---------------------------------------------------------------------------
 *  핸들 내부 상태
 *
 *  core 의 struct 에서 떼어 온 것들이다. 예전에는 이 15개 필드가 struct core 에
 *  섞여 있어서, 산출물과 무관한 코드(heartbeat/FSM/ioctl)를 읽을 때도 계속
 *  눈에 밟혔다.
 * ------------------------------------------------------------------------- */
struct scan_out {
    struct scan_request req;      /* 요청 사본 — 격자 기하를 매번 여기서 재계산 */
    int32_t  lidar_offset_mm;     /* 축교점→발광면. 좌표에 적용된다             */
    void    *log;                 /* core_log 용 불투명 포인터 (NULL 가능)      */

    struct scan_cell *grid;       /* NULL 이면 스캔 중 아님 */
    uint32_t grid_rows;
    uint32_t grid_cols;
    uint32_t pc_written;          /* 격자에 채운 유효 점 수 */
    uint32_t merged;              /* 같은 셀에 추가 도착해 병합된 샘플 수 */
    uint32_t avg_refused;         /* 산포가 커 평균을 거부한 셀 수 (에지 추정) */
    uint32_t drop_range;          /* 격자 범위 밖 각도 */
    /* dis_status 분포 — Datasheet(0=invalid,1=valid) 와 User Manual 예제가
     * 정반대라 실측으로 판별해야 한다. 인덱스 = status 값(0~3), 그 외는 [3]. */
    uint32_t status_hist[4];
    uint64_t scan_start_ns;
    uint64_t scan_end_ns;
    char     session_id[32];
    char     scan_id[32];
    char     pc_path[256];        /* .pcd 경로  */
    char     js_path[256];        /* .json 경로 */

    /* CMD_HOMED 결과 (provenance). 엔코더 원본을 같이 남기는 이유는, 영점
     * 상수가 나중에 틀렸다고 밝혀져도 raw 로부터 각도를 재계산해 이미 찍어둔
     * 스캔을 살릴 수 있어야 해서다. */
    bool     home_valid;
    uint16_t home_pan_raw;
    uint16_t home_tilt_raw;
    int16_t  home_pan_ddeg;
    int16_t  home_tilt_ddeg;
};

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
 *  주의: 여기서 range 는 라이다가 보고한 거리가 **아니다**.
 *        range = distance(라이다 보고) + LIDAR_RANGE_OFFSET_MM
 *    원점은 라이다 발광면이 아니라 팬/틸트 회전축 교점이다. 자세한 근거는
 *    LIDAR_RANGE_OFFSET_MM 정의 위 주석 참조.
 *
 *  주의: 이전 ICD(z-up: x=d·cosφ·cosθ, y=d·cosφ·sinθ, z=d·sinφ)와 **축이 다르다**.
 *    2026-07-29 이전에 생성된 .pcd 는 옛 축이므로 섞어 쓰지 말 것.
 *  주의: 단위도 mm → **meter** 로 변경(문서 01/02 계약).
 *  주의: 센서 높이는 좌표에 **적용하지 않는다** — frame 이 lidar_scan(원점=센서)
 *    이기 때문. 높이는 scan.sensor_height_m 메타데이터로만 전달.
 * ------------------------------------------------------------------------- */

/* ---------------------------------------------------------------------------
 *  기구각 → 계약각  (2축 천장 마운트, 2026-07-30 확정)
 *
 *  프로토콜이 나르는 각도는 **기구 각도**다. 계약 좌표계로 옮기는 건 데몬 몫
 *  (protocol.h §4). 두 각도계가 1:1 이 아닌 이유는 스윕이 nadir 를 지나서다.
 *
 *  기구 배치 — 틸트 영점 = 바닥(nadir):
 *
 *        틸트 -90 ──────── 0 ──────── +90
 *         (벽 A)        (바닥)       (벽 B)
 *
 *      틸트 = 빠른 축. 한 스윕이 벽 A → 바닥 → 벽 B (기구상 180도).
 *      팬   = 느린 축. 줄마다 1도씩, **0~180도만** 이동.
 *
 *  스윕이 바닥을 넘어가는 순간 빔은 반대편 방위를 보게 된다. 그래서 기구 팬이
 *  180도만 돌아도 계약 방위는 360도가 채워진다 (= 케이블 감김 없이 전방위):
 *
 *      m <= 0 :  pan = p          tilt = -90 - m     (벽 A 쪽 반)
 *      m >  0 :  pan = p + 180    tilt = -90 + m     (벽 B 쪽 반)
 *
 *      검산:  m = -90  →  (p,       0)   벽 A 수평
 *             m =   0  →  (p,     -90)   바닥
 *             m = +90  →  (p+180,   0)   벽 B 수평
 *
 *  주의: nadir(계약 tilt=-90)는 극점이라 방위가 축퇴한다. 모든 팬 줄이 같은 점을
 *    보므로 그 행은 격자의 절반만 채워지는데, 버그가 아니라 구면 격자의 성질이다.
 * ------------------------------------------------------------------------- */
static void mech_to_contract(int16_t mech_pan_ddeg, int16_t mech_tilt_ddeg,
                             int16_t *out_pan_ddeg, int16_t *out_tilt_ddeg)
{
    int32_t pan;
    int32_t tilt;

    if (mech_tilt_ddeg <= 0) {
        pan  = (int32_t)mech_pan_ddeg;
        tilt = -900 - (int32_t)mech_tilt_ddeg;
    } else {
        pan  = (int32_t)mech_pan_ddeg + 1800;
        tilt = -900 + (int32_t)mech_tilt_ddeg;
    }

    pan %= 3600;
    if (pan < 0) {
        pan += 3600;
    }

    *out_pan_ddeg  = (int16_t)pan;
    *out_tilt_ddeg = (int16_t)tilt;
}
/* ---------------------------------------------------------------------------
 *  포인트클라우드 출력 (.pcd ascii)
 *
 *   헤더의 POINTS/WIDTH 는 스캔이 끝나야 확정되므로, 자리를 고정폭으로 미리
 *   잡아두고 완료 시 그 자리에 덮어쓴다(파일 재작성 불필요).
 * ------------------------------------------------------------------------- */
/* 격자 크기 산출.
 *   columns = 팬 스윕 폭 / 격자간격,  rows = 틸트 줄 수
 *   ordering: row 0 = tilt_max(마지막 row = tilt_min), column 0 = pan_min */
/* organized 격자의 기하. 요청(scan_request)은 **기구각**이지만 격자는
 * **계약각** 위에 놓이므로, 요청 범위를 계약각으로 옮겨 놓은 결과다.
 *
 * 요청 자체에서 매번 계산한다(상태 없음). 점당 정수 연산 몇 개라
 * 32,000점을 돌려도 무시할 비용이고, 격자 기하가 두 곳에서 어긋날 여지를 없앤다. */
struct grid_geom {
    int32_t  step;              /* 격자 간격 (0.1도)              */
    int32_t  pan_origin_ddeg;   /* col 0 의 계약 방위             */
    int32_t  tilt_top_ddeg;     /* row 0 의 계약 고각 (가장 위)   */
    uint32_t rows;
    uint32_t cols;
    bool     full_circle;       /* 방위가 360도를 다 덮는가       */
};

static void grid_geometry(const struct scan_request *r, struct grid_geom *g)
{
    const int32_t step = (r->step_ddeg > 0u) ? (int32_t)r->step_ddeg : 10;

    const int32_t t0 = (int32_t)r->tilt_start_ddeg;
    const int32_t t1 = (int32_t)r->tilt_end_ddeg;
    const int32_t tlo = (t0 < t1) ? t0 : t1;
    const int32_t thi = (t0 < t1) ? t1 : t0;

    /* 계약 고각 = -900 + |기구 틸트|. 요청 구간에서 |m| 이 갖는 범위를 구한다.
     * 구간이 0 을 품으면(= nadir 를 지나는 스윕) 최소는 0 이다. */
    const int32_t a0 = (tlo < 0) ? -tlo : tlo;
    const int32_t a1 = (thi < 0) ? -thi : thi;
    const bool crosses_nadir = (tlo <= 0) && (thi >= 0);
    const int32_t abs_min = crosses_nadir ? 0 : ((a0 < a1) ? a0 : a1);
    const int32_t abs_max = (a0 > a1) ? a0 : a1;

    g->step          = step;
    g->tilt_top_ddeg = -900 + abs_max;                  /* row 0 = 가장 위 */
    g->rows          = (uint32_t)((abs_max - abs_min) / step) + 1u;

    /* 계약 방위 축.
     *
     * 스윕이 바닥을 넘어가면 한 줄이 방위 p 와 p+180 을 함께 훑는다. 이때
     * 팬을 일부 구간만 돌리면 덮이는 방위가 **두 토막으로 갈라져** 연속된
     * 열 축에 담기지 않는다 (예: 팬 0~90 -> 방위 0~90 과 180~270).
     * 그래서 이 경우 방위 축은 언제나 한 바퀴로 잡고, 안 훑은 방위는 빈
     * 셀(JSON null / PCD NaN)로 남긴다. 열 번호가 절대 방위와 1:1 이 되어
     * 소비자 입장에서도 해석이 단순하다.
     *
     * 주의: 열 수를 "스팬/스텝 + 1" 로 잡으면 안 된다. 팬 0~179(1도)는 스팬이
     *   179 도지만 줄은 180 개고 방위는 360 개를 덮는다. 스팬 기준으로 세면
     *   359 가 나와 마지막 방위가 통째로 잘린다. */
    if (crosses_nadir && (tlo < 0) && (thi > 0)) {
        g->full_circle     = true;
        g->cols            = (uint32_t)(3600 / step);
        g->pan_origin_ddeg = 0;                 /* 열 0 = 절대 방위 0 도 */
        return;
    }

    /* 여기부터는 바닥을 넘지 않는 스윕(1축 스캔 등). 방위가 한 토막이라
     * 요청 구간을 그대로 쓴다. 틸트가 양수 쪽만이면 방위는 통째로 180도 건너편. */
    int32_t pan_span = (int32_t)r->pan_end_ddeg - (int32_t)r->pan_start_ddeg;
    if (pan_span < 0) {
        pan_span += 3600;
    }

    int32_t origin = (int32_t)r->pan_start_ddeg;
    if (tlo > 0) {
        origin += 1800;
    }
    origin %= 3600;
    if (origin < 0) {
        origin += 3600;
    }
    g->pan_origin_ddeg = origin;

    if ((pan_span + step) >= 3600) {
        /* 한 바퀴를 다 덮는 경우. +1 을 붙이면 마지막 열이 첫 열과 같은
         * 방위가 되어 영원히 비는 중복 열이 생긴다. */
        g->full_circle = true;
        g->cols        = (uint32_t)(3600 / step);
    } else {
        g->full_circle = false;
        g->cols        = (uint32_t)(pan_span / step) + 1u;
    }
}

static void grid_dims(const struct scan_request *r, uint32_t *rows, uint32_t *cols)
{
    struct grid_geom g;

    grid_geometry(r, &g);
    *rows = g.rows;
    *cols = g.cols;
}

/* 계약 각도 → 격자 셀. 범위를 벗어나면 false. */
static bool grid_index(const struct scan_out *o, int16_t pan_ddeg, int16_t tilt_ddeg,
                       uint32_t *row, uint32_t *col)
{
    struct grid_geom g;

    grid_geometry(&o->req, &g);

    int32_t dpan = (int32_t)pan_ddeg - g.pan_origin_ddeg;
    dpan %= 3600;
    if (dpan < 0) {
        dpan += 3600;                                   /* 랩어라운드 */
    }
    int32_t ci = (dpan + (g.step / 2)) / g.step;        /* 반올림 */
    if (g.full_circle && (ci == (int32_t)g.cols)) {
        ci = 0;                                         /* 360도 == 0도 */
    }

    /* row 0 = tilt_top 이므로 위에서부터 센다. */
    const int32_t ri = (g.tilt_top_ddeg - (int32_t)tilt_ddeg + (g.step / 2)) / g.step;

    bool ok = false;
    if ((ci >= 0) && ((uint32_t)ci < o->grid_cols) &&
        (ri >= 0) && ((uint32_t)ri < o->grid_rows)) {
        *col = (uint32_t)ci;
        *row = (uint32_t)ri;
        ok = true;
    }
    return ok;
}

struct scan_out *scan_out_open(const struct scan_request *req,
                               int32_t lidar_offset_mm, void *log_core)
{
    struct scan_out *o = calloc(1u, sizeof(*o));
    if (o == NULL) {
        core_log(log_core, "SCAN", "핸들 할당 실패");
        return NULL;
    }
    o->req             = *req;   /* 사본 — 호출자가 요청을 지워도 격자 기하가 산다 */
    o->lidar_offset_mm = lidar_offset_mm;
    o->log             = log_core;

    /* 출력 디렉토리 준비: 시스템 경로 실패 시 현재 디렉토리로 폴백 */
    const char *dir = SCAN_OUT_DIR;
    if (mkdir(dir, 0755) < 0 && errno != EEXIST) {
        dir = "./scans";
        if (mkdir(dir, 0755) < 0 && errno != EEXIST) {
            core_log(o->log, "SCAN", "출력 디렉토리 생성 실패: %s", strerror(errno));
            free(o);
            return NULL;
        }
    }

    time_t     now = time(NULL);
    struct tm  tmv;
    memset(&tmv, 0, sizeof(tmv));
    (void)localtime_r(&now, &tmv);
    char stamp[24];
    (void)strftime(stamp, sizeof(stamp), "%Y%m%d-%H%M%S", &tmv);

    (void)snprintf(o->session_id, sizeof(o->session_id), "calib-%s", stamp);
    (void)snprintf(o->scan_id,    sizeof(o->scan_id),    "sweep-000001");
    (void)snprintf(o->pc_path, sizeof(o->pc_path), "%s/%s_%s.pcd",
                   dir, o->session_id, o->scan_id);
    (void)snprintf(o->js_path, sizeof(o->js_path), "%s/%s_%s_pan_tilt_lidar.json",
                   dir, o->session_id, o->scan_id);

    /* 핵심: 여기서 **실제로 파일을 만들어 본다.**
     *
     * 산출물은 스캔이 다 끝난 뒤(scan_out_close)에야 기록된다 — 격자를 통째로
     * 메모리에 들고 있다가 마지막에 쓰기 때문이다. 그래서 쓰기 권한이 없으면
     * **40,000점을 10분간 모으고 나서** fopen 이 실패하고, 그 시점엔 데이터를
     * 되살릴 방법이 없다. 실기에서 그대로 당했다(scans/ 가 root 소유였는데
     * 데몬을 sudo 없이 돌림 → "Permission denied" 두 줄 남기고 전량 유실).
     *
     * 위의 mkdir 은 이걸 못 잡는다. 디렉토리가 이미 있으면 EEXIST 로 통과하는데,
     * "존재한다" 와 "쓸 수 있다" 는 다른 얘기다.
     *
     * 주의: access(W_OK) 를 쓰지 않는다. 그건 **실제 uid** 로 검사해서 데몬이
     *   권한을 떨어뜨려 실행될 때 답이 틀리고, 읽기전용 마운트·디스크 가득 같은
     *   경우도 못 잡는다. 진짜로 열어보는 것이 유일하게 믿을 만한 검사다.
     *
     * 검사 후 지운다 — 0바이트 파일을 남기면 실패한 스캔이 산출물처럼 보인다. */
    FILE *probe = fopen(o->pc_path, "w");
    if (probe == NULL) {
        core_log(o->log, "SCAN",
                 "출력 경로에 쓸 수 없음 %s: %s — 스캔을 시작하지 않는다",
                 o->pc_path, strerror(errno));
        core_log(o->log, "SCAN",
                 "  (디렉토리 소유자 확인: ls -ld %s / 고치기: sudo chown -R $USER %s)",
                 dir, dir);
        free(o);
        return NULL;
    }
    (void)fclose(probe);
    (void)unlink(o->pc_path);

    grid_dims(&o->req, &o->grid_rows, &o->grid_cols);

    const size_t n = (size_t)o->grid_rows * (size_t)o->grid_cols;
    o->grid = calloc(n, sizeof(struct scan_cell));
    if (o->grid == NULL) {
        core_log(o->log, "SCAN", "격자 할당 실패 (%ux%u)", o->grid_rows, o->grid_cols);
        free(o);
        return NULL;
    }

    o->pc_written    = 0u;
    o->merged        = 0u;
    o->avg_refused   = 0u;
    o->drop_range    = 0u;
    memset(o->status_hist, 0, sizeof(o->status_hist));
    o->scan_start_ns = mono_ns();
    o->scan_end_ns   = o->scan_start_ns;

    core_log(o->log, "SCAN", "격자 %ux%u (%zu셀) — %s", o->grid_rows, o->grid_cols, n,
             o->session_id);
    return o;
}

/* 스캔 점 1개를 격자에 배치한다.
 *
 * 들어온 각도는 **기구각**이므로 먼저 계약각으로 옮긴다(mech_to_contract).
 * 격자 인덱싱도, 셀에 저장하는 각도도, 나중의 (x,y,z) 변환도 전부 계약각
 * 기준이다 — 기구각을 그대로 쓰면 바닥 넘어간 절반이 엉뚱한 방위에 쌓인다.
 *
 * 기구각도 셀에 함께 남긴다. 산출물이 이상할 때 "변환이 틀렸나 / 모터가
 * 엉뚱한 데 있었나" 를 산출물만 보고 가를 수 있어야 하기 때문. */
/* 이 샘플이 셀 중심에서 얼마나 벗어났나 (방위+고각 이탈량의 합, 0.1도).
 * 같은 셀에 여러 샘플이 오면 이 값이 작은 쪽을 대표로 삼는다 — 격자 배정
 * 오차를 ±step/2 에서 실질적으로 절반까지 줄인다. */
static uint16_t cell_center_offset_ddeg(const struct scan_out *o,
                                        uint32_t row, uint32_t col,
                                        int16_t c_pan, int16_t c_tilt)
{
    struct grid_geom g;
    grid_geometry(&o->req, &g);

    int32_t dt = (int32_t)c_tilt - (g.tilt_top_ddeg - ((int32_t)row * g.step));
    if (dt < 0) {
        dt = -dt;
    }

    int32_t dp = (int32_t)c_pan
               - ((g.pan_origin_ddeg + ((int32_t)col * g.step)) % 3600);
    dp %= 3600;
    if (dp > 1800) {
        dp -= 3600;
    } else if (dp < -1800) {
        dp += 3600;
    } else {
        /* 이미 반바퀴 안 */
    }
    if (dp < 0) {
        dp = -dp;
    }

    return (uint16_t)(dt + dp);
}

/* 셀 안의 샘플들을 평균해도 되나. 판정 근거는 struct scan_cell 위 주석. */
static bool cell_avg_ok(const struct scan_cell *cell)
{
    bool ok = false;

    if (cell->n_samples >= 2u) {
        const uint32_t spread = (uint32_t)cell->d_max_mm - (uint32_t)cell->d_min_mm;
        uint32_t tol = ((uint32_t)cell->d_min_mm * CELL_AVG_REL_TOL_PCT) / 100u;
        if (tol < CELL_AVG_ABS_TOL_MM) {
            tol = CELL_AVG_ABS_TOL_MM;
        }
        ok = (spread <= tol);
    }
    return ok;
}

/* 산출물에 쓸 최종 거리. 평균이 안전하면 평균, 아니면 대표 샘플 그대로. */
static uint16_t cell_distance_mm(const struct scan_cell *cell)
{
    uint16_t d = cell->d_mm;

    if (cell_avg_ok(cell)) {
        d = (uint16_t)((cell->d_sum_mm + ((uint32_t)cell->n_samples / 2u))
                       / (uint32_t)cell->n_samples);
    }
    return d;
}

void scan_out_add(struct scan_out *o, const struct proto_scan_point *p)
{
    uint32_t row = 0u;
    uint32_t col = 0u;
    int16_t  c_pan  = 0;
    int16_t  c_tilt = 0;

    if (o->grid == NULL) {
        return;
    }

    mech_to_contract(p->pan_ddeg, p->tilt_ddeg, &c_pan, &c_tilt);

    if (!grid_index(o, c_pan, c_tilt, &row, &col)) {
        o->drop_range++;                 /* 요청 범위 밖 각도 */
        return;
    }

    struct scan_cell *cell = &o->grid[((size_t)row * o->grid_cols) + col];
    const uint16_t   off   = cell_center_offset_ddeg(o, row, col, c_pan, c_tilt);

    o->status_hist[(p->dis_status < 3u) ? p->dis_status : 3u]++;
    o->scan_end_ns = mono_ns();

    if (!cell->filled) {
        cell->seq           = o->pc_written;
        cell->d_sum_mm      = (uint32_t)p->d_mm;
        cell->d_min_mm      = p->d_mm;
        cell->d_max_mm      = p->d_mm;
        cell->n_samples     = 1u;
        cell->best_off_ddeg = off;
        cell->filled        = true;
        o->pc_written++;
    } else {
        /* 같은 셀에 추가 도착. 예전에는 버렸지만(drop_dup) 그러면 스윕이
         * 격자보다 조밀할 때 절반을 그냥 내다 버리는 셈이었다. 거리는
         * 누적해 평균 후보로 남기고, 나머지 필드는 셀 중심에 더 가까운
         * 샘플이 왔을 때만 교체한다. */
        o->merged++;
        if (cell->n_samples < 255u) {
            cell->n_samples++;
        }
        cell->d_sum_mm += (uint32_t)p->d_mm;
        if (p->d_mm < cell->d_min_mm) {
            cell->d_min_mm = p->d_mm;
        }
        if (p->d_mm > cell->d_max_mm) {
            cell->d_max_mm = p->d_mm;
        }
        if (off >= cell->best_off_ddeg) {
            return;                      /* 대표를 바꿀 만큼 가깝지 않다 */
        }
        cell->best_off_ddeg = off;
    }

    /* 첫 샘플이거나, 셀 중심에 더 가까운 샘플로 대표를 교체하는 경우 */
    cell->rx_ns           = o->scan_end_ns;
    cell->pan_ddeg        = c_pan;
    cell->tilt_ddeg       = c_tilt;
    cell->mech_pan_ddeg   = p->pan_ddeg;
    cell->mech_tilt_ddeg  = p->tilt_ddeg;
    cell->d_mm            = p->d_mm;
    cell->signal_strength = p->signal_strength;
    cell->device_time_ms  = p->device_time_ms;
    cell->stm_ts_ms       = p->stm_ts_ms;
    cell->dis_status      = p->dis_status;
    cell->range_precision = p->range_precision;
}

/* organized PCD 출력 (변환 후 x/y/z, meter).
 * 미측정 셀은 nan 으로 남겨 구멍과 실패를 구분 가능하게 한다. */
static bool write_pcd(struct scan_out *o)
{
    FILE *fp = fopen(o->pc_path, "w");
    if (fp == NULL) {
        core_log(o->log, "SCAN", "PCD 생성 실패 %s: %s", o->pc_path, strerror(errno));
        return false;
    }

    const size_t n = (size_t)o->grid_rows * (size_t)o->grid_cols;

    /* 주의: 센서 높이(z_offset)를 좌표에 **적용하지 않는다**.
     *   frame 이름이 lidar_scan 이면 원점은 라이다 자신이므로, tilt=0 인 점의
     *   y 는 0 이어야 한다. 예전에 높이를 빼서 모든 y 가 -1.2m 로 찍혔는데,
     *   그건 사실상 actuator_base 계열 좌표라 라벨과 불일치였다(2026-07-29 수정).
     *   높이는 scan.sensor_height_m 메타데이터로만 전달하고, 레이어 적층 같은
     *   변환은 소비자(또는 json2pcd 도구의 옵션)가 수행한다. */

    (void)fprintf(fp,
        "# .PCD v0.7 - adts scan (organized)\n"
        "# frame = lidar_scan (origin = pan/tilt axis intersection)"
        "  +x right +y down +z forward  unit = meter\n"
        "# range_offset_m = %.4f (축교점→발광면. 좌표에 **적용됨**: r = distance + offset)\n"
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
        (double)o->lidar_offset_mm / 1000.0,
        (double)o->req.sensor_height_mm / 1000.0,
        o->session_id, o->scan_id, o->grid_cols, o->grid_rows, n);

    for (size_t i = 0; i < n; ++i) {
        const struct scan_cell *cell = &o->grid[i];
        if (!cell->filled) {
            (void)fputs("nan nan nan\n", fp);
        } else {
            const double pan  = DDEG2RAD(cell->pan_ddeg);
            const double tilt = DDEG2RAD(cell->tilt_ddeg);
            /* 라이다 보고거리 + 축교점 오프셋. 원점이 발광면이 아니라 회전축
             * 교점이라 이걸 더해야 한다(LIDAR_RANGE_OFFSET_MM 주석 참조). */
            const double r    = (double)((int32_t)cell_distance_mm(cell)
                                         + o->lidar_offset_mm) / 1000.0;
            const double ct   = cos(tilt);

            (void)fprintf(fp, "%.4f %.4f %.4f\n",
                          r * ct * sin(pan),
                          -r * sin(tilt),
                          r * ct * cos(pan));
        }
    }
    /* 주의: fclose 결과까지 본다. 디스크가 가득 찼거나 마운트가 사라진 경우
     *   fprintf 는 조용히 성공하고 **버퍼가 비워지는 fclose 에서야** 실패한다.
     *   여기서 안 보면 "산출 완료" 를 찍어놓고 실제 파일은 잘려 있게 된다. */
    const bool wr_ok = (ferror(fp) == 0);
    return (fclose(fp) == 0) && wr_ok;
}

/* 원시 측정 JSON 출력 (변환 전 — 계약상 golden reference).
 * x/y/z 는 넣지 않는다. 캘리브 adapter 가 distance/pan/tilt 로 직접 계산한다. */
static bool write_json(struct scan_out *o)
{
    FILE *fp = fopen(o->js_path, "w");
    if (fp == NULL) {
        core_log(o->log, "SCAN", "JSON 생성 실패 %s: %s", o->js_path, strerror(errno));
        return false;
    }

    /* 헤더의 진단 수치를 먼저 확정한다 — 셀 루프보다 앞에 찍히기 때문. */
    o->avg_refused = 0u;
    {
        const size_t nc = (size_t)o->grid_rows * (size_t)o->grid_cols;
        for (size_t k = 0; k < nc; ++k) {
            const struct scan_cell *cl = &o->grid[k];
            if ((cl->n_samples >= 2u) && !cell_avg_ok(cl)) {
                o->avg_refused++;
            }
        }
    }

    const struct scan_request *rq = &o->req;
    const size_t n = (size_t)o->grid_rows * (size_t)o->grid_cols;

    /* 격자 범위는 **계약각** 으로 적는다. 요청(rq)은 기구각이라 그대로 실으면
     * 헤더의 pan 범위(0~180)와 measurements 의 pan(0~360)이 어긋나 소비자가
     * 데이터를 범위 밖으로 판정한다. */
    struct grid_geom g;
    grid_geometry(rq, &g);
    const int32_t pan_lo  = g.pan_origin_ddeg;
    const int32_t pan_hi  = g.pan_origin_ddeg + (int32_t)(g.cols - 1u) * g.step;
    const int32_t tilt_hi = g.tilt_top_ddeg;
    const int32_t tilt_lo = g.tilt_top_ddeg - (int32_t)(g.rows - 1u) * g.step;

    /* 홈 provenance. HOME 을 안 거쳤으면 null 로 남겨 "0 도였다" 와 구분한다. */
    char home_js[160];
    if (o->home_valid) {
        (void)snprintf(home_js, sizeof(home_js),
            "{ \"pan_encoder_raw\": %u, \"tilt_encoder_raw\": %u, "
            "\"pan_ddeg\": %d, \"tilt_ddeg\": %d, \"encoder_bits\": 14 }",
            o->home_pan_raw, o->home_tilt_raw,
            o->home_pan_ddeg, o->home_tilt_ddeg);
    } else {
        (void)snprintf(home_js, sizeof(home_js), "null");
    }

    (void)fprintf(fp,
        "{\n"
        "  \"interface_version\": \"%s\",\n"
        /* 1.1 → 1.2: sensor.range_offset_m 신설. 소비자가 distance_m 로
         * 직접 (x,y,z)를 계산하므로 **반드시 더해야 하는** 값이고, 모르고
         * 옛 방식대로 쓰면 방 전체가 84mm 수축한다. 그래서 마이너 판올림. */
        "  \"schema_version\": \"1.2\",\n"
        "  \"session_id\": \"%s\",\n"
        "  \"scan_id\": \"%s\",\n"
        "  \"producer\": { \"software\": \"adts_daemon\", \"protocol_version\": %u },\n"
        /* 핵심: range_offset_m — measurements[].distance_m 는 라이다 **발광면**
         *   기준 원거리다. 좌표 원점은 팬/틸트 회전축 교점이므로 소비자는
         *   반드시 r = distance_m + range_offset_m 으로 반경을 만든 뒤
         *   구면→직교 변환해야 한다. 빠뜨리면 장면 전체가 원점 쪽으로
         *   range_offset_m 만큼 균일하게 수축하고, 평면 잔차로는 안 잡힌다. */
        "  \"sensor\": { \"model\": \"TOFSense-F2P\", \"lidar_rate_hz\": 100,"
        " \"range_offset_m\": %.4f },\n"
        "  \"frame\": {\n"
        "    \"name\": \"lidar_scan\",\n"
        "    \"origin\": \"pan_tilt_axis_intersection\",\n"
        "    \"handedness\": \"right\",\n"
        "    \"convention\": \"+x right, +y down, +z forward; pan+ right, tilt+ up\",\n"
        "    \"range_formula\": \"r = distance_m + sensor.range_offset_m;"
        " x = r*cos(tilt)*sin(pan), y = -r*sin(tilt), z = r*cos(tilt)*cos(pan)\"\n"
        "  },\n"
        "  \"units\": { \"distance\": \"meter\", \"angle\": \"radian\", \"timestamp\": \"nanosecond\" },\n"
        "  \"scan\": {\n"
        "    \"mode\": \"continuous_tilt_sweep\",\n"
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
        /* 기구 원본. 계약각으로 환산하기 전 값이라, 산출물이 이상할 때
         * 변환 문제인지 구동 문제인지 가르는 근거가 된다. */
        "  \"mechanism\": {\n"
        "    \"sweep_axis\": \"tilt\",\n"
        "    \"index_axis\": \"pan\",\n"
        "    \"tilt_zero\": \"nadir\",\n"
        "    \"angle_source\": \"step_count\",\n"
        "    \"home_method\": \"absolute_encoder\",\n"
        "    \"pan_range_ddeg\": [%d, %d],\n"
        "    \"tilt_range_ddeg\": [%d, %d],\n"
        "    \"step_ddeg\": %u,\n"
        "    \"home\": %s\n"
        "  },\n"
        "  \"diagnostics\": {\n"
        "    \"checksum_error_count\": 0,\n"
        "    \"merged_sample_count\": %u,\n"
        "    \"avg_refused_cell_count\": %u,\n"
        "    \"out_of_range_angle_count\": %u,\n"
        /* 0 이 아니라 null 이다. STM 이 틸트 끝점 엔코더 대조 횟수를
         * 상행하는 경로가 아직 없어 데몬은 이 값을 **모른다**. 0 으로
         * 적으면 "대조에서 한 번도 안 틀어졌다" 는 거짓 주장이 된다. */
        "    \"encoder_gap_count\": null,\n"
        "    \"dis_status_histogram\": "
        "{ \"0\": %u, \"1\": %u, \"2\": %u, \"other\": %u }\n"
        "  },\n"
        "  \"measurements\": [\n",
        JSON_IFACE_VERSION, o->session_id, o->scan_id, (unsigned)PROTO_VERSION,
        (double)o->lidar_offset_mm / 1000.0,
        o->grid_rows, o->grid_cols,
        DDEG2RAD(pan_lo),  DDEG2RAD(pan_hi),
        DDEG2RAD(tilt_lo), DDEG2RAD(tilt_hi),
        DDEG2RAD((int)rq->step_ddeg),
        (double)rq->sensor_height_mm / 1000.0,
        n, o->pc_written,
        (unsigned long long)o->scan_start_ns,
        (unsigned long long)o->scan_end_ns,
        rq->pan_start_ddeg, rq->pan_end_ddeg,
        rq->tilt_start_ddeg, rq->tilt_end_ddeg,
        (unsigned)rq->step_ddeg, home_js,
        o->merged, o->avg_refused, o->drop_range,
        o->status_hist[0], o->status_hist[1],
        o->status_hist[2], o->status_hist[3]);

    for (size_t i = 0; i < n; ++i) {
        const struct scan_cell *cell = &o->grid[i];
        const uint32_t row = (uint32_t)(i / o->grid_cols);
        const uint32_t col = (uint32_t)(i % o->grid_cols);
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
                " \"samples\": 0, \"spread_mm\": null,"
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
             *   주의: 단 STM32 clock domain 이라 host 와 offset 미보정 →
             *      quality_flags 에 TIMESTAMP_STM_CLOCK 을 남긴다.
             * 주의: 아직 null 인 필드: encoder_count / encoder_timestamp
             *   → 강유근 MT6701 엔코더 펌웨어 구현 후 채움.
             * range_precision (User Manual IIC 0x2C): **cm 단위**,
             *   0x00 = <1cm, 0xFF = >=255cm.
             *
             * 주의: 실측(2026-07-29): F2 P 는 359/359 전부 **0xFF** 를 보낸다.
             *   매뉴얼 §7.3.4 "If there is no corresponding parameter in the
             *   register, the default output is 0xff" 에 따라 **이 모델이
             *   지원하지 않는 필드**로 판단. 0.7m 측정에 정밀도 >=2.55m 는
             *   스펙(±3cm)과 모순이므로 값으로 쓰면 안 된다.
             *   → 0xFF 는 range_precision_m 을 null 로 두고 flag 로 표시.
             *   → 02 문서 §7.1 의 depth edge 분모 sigma 는 이 필드 대신
             *      Datasheet 거리구간별 표준편차(<1cm@[0.05,10]m,
             *      <6cm@[10,25]m) 또는 반복측정 실측 분산을 써야 한다. */
            /* 0xFF = 미지원/포화 → m 값을 만들지 않고 flag 로 알린다. */
            /* 셀 병합 결과. 평균을 거부했다면 그 사실을 flag 로 남긴다 —
             * 산포가 컸다는 건 대개 이 셀이 깊이 에지를 물었다는 뜻이라,
             * 하류 에지 검출에 그대로 쓸 수 있는 단서다. */
            const bool     averaged = cell_avg_ok(cell);
            const uint32_t spread   = (uint32_t)cell->d_max_mm
                                    - (uint32_t)cell->d_min_mm;
            const char *avg_flag = "";
            if (cell->n_samples >= 2u) {
                avg_flag = averaged ? ",\"RANGE_AVERAGED\""
                                    : ",\"AVG_REFUSED_SPREAD\"";
            }

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
                " \"samples\": %u, \"spread_mm\": %u,"
                " \"signal_strength\": %u, \"range_precision_raw\": %u,"
                " \"range_precision_m\": %s,"
                " \"checksum_valid\": true,"
                " \"angle_source\": \"step_count\","
                " \"timestamp_source\": \"host_rx_monotonic\","
                " \"encoder_interpolation_valid\": null,"
                " \"valid\": true,"
                " \"quality_flags\": [\"VALID_RANGE\"%s%s] }%s\n",
                cell->seq, row, col,
                (unsigned long long)cell->rx_ns,
                (unsigned)cell->device_time_ms,
                (unsigned)cell->stm_ts_ms,
                DDEG2RAD(cell->pan_ddeg), DDEG2RAD(cell->tilt_ddeg),
                (double)cell_distance_mm(cell) / 1000.0,
                (unsigned)cell->dis_status,
                (unsigned)cell->n_samples, (unsigned)spread,
                (unsigned)cell->signal_strength,
                (unsigned)cell->range_precision,
                rp_m, avg_flag, rp_flag, sep);
        }
    }

    (void)fprintf(fp, "  ]\n}\n");
    const bool wr_ok = (ferror(fp) == 0);   /* write_pcd 의 주석 참조 */
    return (fclose(fp) == 0) && wr_ok;
}

/* 격자를 두 포맷으로 산출하고 해제한다. */
bool scan_out_close(struct scan_out **po)
{
    /* 중단·종료 경로에서 여러 번 불린다. 두 번째부터는 조용히 통과해야 한다. */
    if ((po == NULL) || (*po == NULL)) {
        return false;
    }
    struct scan_out *o = *po;
    const bool js_ok = write_json(o);   /* 변환 전 원시 (계약) */
    const bool pc_ok = write_pcd(o);    /* 변환 후 x/y/z (뷰어·편의) */

    /* 주의: 실패해도 "산출 완료" 를 찍고 경로를 나열하던 시절이 있었다. 로그만
     *   보면 성공으로 보여서, 파일이 없는 걸 한참 뒤에야 알아챘다. 결과를
     *   말 그대로 적는다. */
    if (js_ok && pc_ok) {
        core_log(o->log, "SCAN",
                 "산출 완료 %ux%u — 유효 %u셀 (병합 %u, 평균거부 %u, 범위밖 %u)",
                 o->grid_rows, o->grid_cols, o->pc_written,
                 o->merged, o->avg_refused, o->drop_range);
        core_log(o->log, "SCAN", "  JSON: %s", o->js_path);
        core_log(o->log, "SCAN", "  PCD : %s", o->pc_path);
    } else {
        core_log(o->log, "SCAN",
                 "핵심: 산출 실패 (JSON=%s PCD=%s) — 유효했던 %u셀은 복구 불가",
                 js_ok ? "ok" : "FAIL", pc_ok ? "ok" : "FAIL", o->pc_written);
    }

    free(o->grid);
    free(o);
    *po = NULL;
    return js_ok && pc_ok;
}

/* 요청 범위·격자로 예상 점 수 산출 (진행률 표시용) */
uint32_t scan_out_expected_points(const struct scan_request *r)
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

/* 팬 이음매(seam) 이중 스캔 경고.
 *
 * 2축 스윕은 한 줄이 방위 p 와 p+180 을 같이 훑는다. 그래서 팬을 0~180 "양끝
 * 포함" 으로 돌리면 첫 줄과 마지막 줄이 **같은 수직 평면**을 잡는다:
 *     팬 0   줄 -> 방위 0, 180
 *     팬 180 줄 -> 방위 180, 360(=0)
 * 방위 0 과 180 만 두 번 측정되고 나머지는 한 번씩이라, 그 두 평면의 점은
 * 같은 셀에 몰려 병합된다. 데이터가 틀리진 않지만 스캔 시간을 헛쓰는 것이고
 * 산출물의 병합 통계가 부풀어 원인을 오해하기 쉽다.
 *
 * 팬을 (한 바퀴 - 1스텝) 까지만 돌리면 정확히 0 이 된다. 예) 1도 격자면 0~179.
 * (실측: 0~180 = 중복 180건 / 0~179 = 0건) */
void scan_out_warn_seam(const struct scan_request *r, void *log_core)
{
    const int32_t step = (int32_t)r->step_ddeg;
    int32_t pan_span = (int32_t)r->pan_end_ddeg - (int32_t)r->pan_start_ddeg;
    if (pan_span < 0) {
        pan_span += 3600;
    }

    /* nadir 를 지나는 스윕에서만 방위가 2배로 펼쳐진다 */
    const bool crosses_nadir = (r->tilt_start_ddeg < 0) && (r->tilt_end_ddeg > 0);
    const bool seam = crosses_nadir && (step > 0) && ((pan_span * 2) >= 3600);

    if (seam) {
        core_log(log_core, "SCAN",
                 "주의: 팬 %d..%d 은 양끝이 같은 평면 — 방위 %d/%d 가 두 번 스캔된다. "
                 "%d 까지만 돌리면 중복 0",
                 r->pan_start_ddeg, r->pan_end_ddeg,
                 r->pan_start_ddeg, (r->pan_start_ddeg + 1800) % 3600,
                 (int)(r->pan_start_ddeg + 1800 - step));
    }
}

/* SCAN_START 하달: 수평 게이트 -> 파일 열기 -> ioctl. 실패 시 false */

/* ---------------------------------------------------------------------------
 *  관측
 * ------------------------------------------------------------------------- */
uint32_t scan_out_point_count(const struct scan_out *o)
{
    return (o != NULL) ? o->pc_written : 0u;
}

const char *scan_out_path(const struct scan_out *o)
{
    return (o != NULL) ? o->pc_path : "";
}

const char *scan_out_json_path(const struct scan_out *o)
{
    return (o != NULL) ? o->js_path : "";
}

void scan_out_set_home(struct scan_out *o,
                       uint16_t pan_raw, uint16_t tilt_raw,
                       int16_t pan_ddeg, int16_t tilt_ddeg)
{
    if (o != NULL) {
        o->home_valid     = true;
        o->home_pan_raw   = pan_raw;
        o->home_tilt_raw  = tilt_raw;
        o->home_pan_ddeg  = pan_ddeg;
        o->home_tilt_ddeg = tilt_ddeg;
    }
}
