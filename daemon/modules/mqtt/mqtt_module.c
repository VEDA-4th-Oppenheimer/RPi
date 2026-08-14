/* ============================================================================
 *  mqtt_module.c  --  Mosquitto 브로커 클라이언트 (제어·상태 통신)
 * ----------------------------------------------------------------------------
 *  담당: 이현우(모듈·코어 통합) / 이광진(브로커 운영·인증서 발급)
 *
 *  토픽 계약: Confluence "MQTT 토픽 계약 — Qt 관제 ↔ RPi 스캐너 데몬" (31162383)
 *  이 파일은 그 문서에 적힌 것만 발행·구독한다. 바꾸려면 문서를 먼저 고칠 것.
 *
 *  ★ epoll 통합 (단일 스레드 유지)
 *      get_fd()   -> mosquitto_socket()  을 코어 epoll 에 등록
 *      on_event() -> mosquitto_loop_read/write()
 *      on_tick()  -> mosquitto_loop_misc()  (keepalive·재접속)
 *    ⚠️ mosquitto_loop_start() (자체 스레드) 금지 — 콜백이 다른 스레드에서 불려
 *      shared_ctx/FSM 에 락이 필요해지고 데몬의 무락 설계가 깨진다.
 *
 *  ★ 재연결 시 소켓 fd 가 바뀐다
 *    끊기면 get_fd() 가 -1 을 돌려주고, 다시 붙으면 새 fd 를 돌려준다.
 *    코어가 매 tick get_fd() 를 확인해 epoll 등록을 갱신한다.
 *
 *  ★ OpenSSL 고정 요건 = MQTT-over-TLS(8883) + mTLS 로 충족한다.
 *    브로커가 클라이언트 인증서를 요구하므로(require_certificate true) 인증서
 *    3종이 없으면 접속 자체가 안 된다. 없으면 모듈은 degraded 로 계속 구동한다
 *    — MQTT 가 안 붙어도 CLI(--scan)로는 스캔이 되어야 하기 때문.
 * ==========================================================================*/
#include "daemon_module.h"

/* ---------------------------------------------------------------------------
 *  ADTS_NO_MQTT — libmosquitto/libcjson 이 없는 환경용 컴파일아웃
 *
 *  맥 크로스 툴체인 sysroot 에는 aarch64 용 패키지가 없다. 그때도 데몬이
 *  빌드돼야 CLion 인덱싱과 나머지 코드 검증이 가능하므로, MQTT 만 빼고
 *  같은 인터페이스의 빈 모듈로 대체한다(파일 끝의 #else 블록).
 *  배포 바이너리는 Docker(대상과 같은 glibc)에서 굽는다.
 * ------------------------------------------------------------------------- */
#ifndef ADTS_NO_MQTT

#include <mosquitto.h>
#include <cjson/cJSON.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

/* ---------------------------------------------------------------------------
 *  토픽 (Confluence 31162383)
 *
 *  kit id 계층은 두지 않는다. 브로커가 킷마다 상주하므로 그 브로커 위의 토픽은
 *  이미 그 킷 것이라 중복 정보다. 나중에 중앙 브로커로 모으더라도 Mosquitto
 *  브리지가 접두사를 붙여줄 수 있어 클라이언트를 고칠 필요가 없다.
 * ------------------------------------------------------------------------- */
#define T_CMD_ALL        "adts/cmd/#"
#define T_CMD_SCAN       "adts/cmd/scan"
#define T_CMD_STOP       "adts/cmd/stop"
#define T_CMD_HOME       "adts/cmd/home"
#define T_CMD_DISARM     "adts/cmd/disarm"
/* ⚠️ rearm 은 아직 계약 문서(Confluence 31162383)에 **없다**. 계약 §5 표는
 *   DISARM 상태에서 "복구" 버튼을 규정하는데 대응 토픽이 비어 있어, Qt 는
 *   지금 로컬 상태만 되돌리고 데몬은 DISARM 에 남는 불일치가 있다.
 *   여기서 먼저 구현해 두고 계약에 반영한다(이현우 협의 필요).
 *   ACL 변경은 불필요 — 양쪽 다 adts/cmd/# 로 잡혀 있다. */
#define T_CMD_REARM      "adts/cmd/rearm"
#define T_ST_DAEMON      "adts/state/daemon"
#define T_ST_SCAN        "adts/state/scan"
#define T_EV_PROGRESS    "adts/event/progress"
#define T_EV_ERROR       "adts/event/error"

/* 접속 파라미터 — 환경변수로 덮어쓸 수 있다(운영 배포 시 유용) */
#define ENV_HOST         "ADTS_MQTT_HOST"
#define ENV_PORT         "ADTS_MQTT_PORT"
#define ENV_CA           "ADTS_MQTT_CA"
#define ENV_CERT         "ADTS_MQTT_CERT"
#define ENV_KEY          "ADTS_MQTT_KEY"

#define DEF_HOST         "127.0.0.1"
#define DEF_PORT         8883
#define DEF_CA           "/etc/adts/certs/ca.crt"
#define DEF_CERT         "/etc/adts/certs/daemon.crt"
#define DEF_KEY          "/etc/adts/certs/daemon.key"

#define MQTT_KEEPALIVE_S     30
#define MQTT_CLIENT_ID       "adts-daemon"
#define PROGRESS_PERIOD_MS   500u      /* 진행률 발행 주기 (약 2Hz)        */
#define STATE_HEARTBEAT_MS  5000u      /* 상태 무변화 시에도 주기 발행     */
#define REQ_ID_LEN            33u      /* 32자 + NUL                        */

/* ⚠️ 여기 있던 ERRC_LINK_DEAD/ERRC_HOME_TIMEOUT 을 지웠다. daemon_module.h 의
 *   NOTICE_* 와 **값은 같은데 이름만 다른 상수 두 벌**이 되어, 한쪽만 고치면
 *   조용히 어긋나는 상태였다. 계약 헤더 쪽 하나만 쓴다. */

/* ---------------------------------------------------------------------------
 *  모듈 상태
 * ------------------------------------------------------------------------- */
static struct mosquitto  *s_mosq;
static struct shared_ctx *s_ctx;          /* init 에서 받아둔다 (콜백용)     */
static bool      s_connected;
static bool      s_tls_ready;             /* 인증서 3종이 다 있었나          */

static char      s_req_id[REQ_ID_LEN];    /* 현재 처리 중인 요청             */
static char      s_last_req_id[REQ_ID_LEN];/* 중복 배달 판정용               */

static uint64_t  s_last_progress_ms;
static uint64_t  s_last_state_ms;
static daemon_state_t s_last_state = ST_IDLE;
static uint8_t   s_last_err_sent;

/* 수평 게이트 판정의 직전 결과. 0=미측정 / 1=OK / 2=NG.
 * roll·pitch 숫자가 아니라 **판정**이 뒤집힐 때만 즉시 발행하기 위한 것이다.
 * 숫자로 비교하면 손떨림 수준의 잡음에도 5초 주기가 무의미해질 만큼 발행이
 * 잦아진다. */
static uint8_t   s_last_level_verdict;

/* 방금 우리가 cmd/disarm 을 코어에 넘겼나.
 *
 * ★ 스캔이 정상 완료되면 코어가 되감기 유예(15초) 뒤 **스스로** DISARM 으로
 *   내려간다. 그런데 예전에는 DISARM 전이를 무조건 오류로 발행해서, 성공한
 *   스캔마다 "안전정지 진입"(code 100)이 하나씩 쌓였다. Qt 오류 로그가
 *   정상 동작으로 가득 차면 진짜 오류를 못 찾는다.
 *
 *   전이 시점에 사유를 알려주는 필드가 shared_ctx 에 없어서, **우리가 보낸
 *   요청은 우리가 기억한다.** 나머지 판정(링크 두절 / STM 오류)은 그 시점의
 *   ctx 로 충분히 가려진다. */
static bool      s_user_disarm;

/* 마지막으로 발행한 코어 통지의 seq. 코어/다른 모듈이 notice_post() 로
 * 올린 것을 여기서 event/error 로 내보낸다. seq 로 에지를 잡는 이유는
 * **같은 코드가 다시 나도 새 사건**이기 때문이다(값 비교로는 못 잡는다). */
static uint32_t  s_notice_seq;

/* ---------------------------------------------------------------------------
 *  보조
 * ------------------------------------------------------------------------- */
static uint64_t now_ms(void)
{
    struct timespec ts;
    (void)clock_gettime(CLOCK_MONOTONIC, &ts);
    return ((uint64_t)ts.tv_sec * 1000u) + ((uint64_t)ts.tv_nsec / 1000000u);
}

static long unix_ts(void)
{
    return (long)time(NULL);
}

static const char *env_or(const char *name, const char *fallback)
{
    const char *v = getenv(name);
    return ((v != NULL) && (v[0] != '\0')) ? v : fallback;
}

static const char *state_name(daemon_state_t s)
{
    return daemon_state_str(s);
}

/* JSON 문자열을 발행하고 해제한다. root 는 항상 소비된다. */
static void publish_json(const char *topic, cJSON *root, int qos, bool retain)
{
    char *txt;

    if (root == NULL) {
        return;
    }
    txt = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    if (txt == NULL) {
        return;
    }
    if ((s_mosq != NULL) && s_connected) {
        (void)mosquitto_publish(s_mosq, NULL, topic,
                                (int)strlen(txt), txt, qos, retain);
    }
    free(txt);
}

/* ---------------------------------------------------------------------------
 *  발행
 * ------------------------------------------------------------------------- */
static cJSON *build_state(const struct shared_ctx *ctx)
{
    cJSON *o = cJSON_CreateObject();
    cJSON *lv;

    if (o == NULL) {
        return NULL;
    }
    (void)cJSON_AddStringToObject(o, "state", state_name(ctx->state));
    (void)cJSON_AddBoolToObject  (o, "online", true);
    (void)cJSON_AddBoolToObject  (o, "link_alive", ctx->link.link_alive != 0u);
    (void)cJSON_AddBoolToObject  (o, "homed",      ctx->link.homed != 0u);
    (void)cJSON_AddBoolToObject  (o, "scanning",   ctx->link.scanning != 0u);
    (void)cJSON_AddNumberToObject(o, "cur_pan_ddeg",  ctx->link.cur_pan_ddeg);
    (void)cJSON_AddNumberToObject(o, "cur_tilt_ddeg", ctx->link.cur_tilt_ddeg);
    (void)cJSON_AddNumberToObject(o, "last_err",      ctx->link.last_err);

    lv = cJSON_AddObjectToObject(o, "level");
    if (lv != NULL) {
        (void)cJSON_AddBoolToObject  (lv, "valid", ctx->level.valid != 0u);
        /* roll/pitch 는 설치각을 뺀 **이탈** 이다(게이트가 보는 값과 동일).
         * raw_* 는 중력벡터 각 그대로 — 둘을 같이 실어야 Qt 에서 "리그가
         * 기울었나" 와 "IMU 마운트가 틀어졌나" 를 구분할 수 있다. */
        (void)cJSON_AddNumberToObject(lv, "roll_deg",  ctx->level.roll_deg);
        (void)cJSON_AddNumberToObject(lv, "pitch_deg", ctx->level.pitch_deg);
        (void)cJSON_AddNumberToObject(lv, "raw_roll_deg",  ctx->level.raw_roll_deg);
        (void)cJSON_AddNumberToObject(lv, "raw_pitch_deg", ctx->level.raw_pitch_deg);
    }
    (void)cJSON_AddNumberToObject(o, "ts", (double)unix_ts());
    return o;
}

/* 코어 level_gate_ok() 와 같은 기준으로 현재 수평 판정을 낸다.
 * (판정 자체는 코어 소관이고 여기서는 "발행할 만한 변화인가" 만 본다) */
static uint8_t level_verdict(const struct shared_ctx *ctx)
{
    uint8_t v = 0u;                     /* 미측정 */

    if (ctx->level.valid != 0u) {
        const float ar = (ctx->level.roll_deg  < 0.0f)
                       ? -ctx->level.roll_deg  : ctx->level.roll_deg;
        const float ap = (ctx->level.pitch_deg < 0.0f)
                       ? -ctx->level.pitch_deg : ctx->level.pitch_deg;
        v = ((ar > LEVEL_GATE_MAX_DEG) || (ap > LEVEL_GATE_MAX_DEG)) ? 2u : 1u;
    }
    return v;
}

static void publish_state(const struct shared_ctx *ctx)
{
    publish_json(T_ST_DAEMON, build_state(ctx), 1, true);   /* retained */
    s_last_state_ms = now_ms();
    s_last_state    = ctx->state;
}

static void publish_progress(const struct shared_ctx *ctx)
{
    cJSON *o = cJSON_CreateObject();

    if (o == NULL) {
        return;
    }
    (void)cJSON_AddStringToObject(o, "req_id", s_req_id);
    (void)cJSON_AddNumberToObject(o, "points",   ctx->progress.points);
    (void)cJSON_AddNumberToObject(o, "expected", ctx->progress.expected);
    (void)cJSON_AddNumberToObject(o, "percent",  ctx->progress.percent);
    (void)cJSON_AddNumberToObject(o, "ts", (double)unix_ts());
    publish_json(T_EV_PROGRESS, o, 0, false);               /* QoS0, 유실 허용 */
}

static void publish_scan_result(const struct shared_ctx *ctx)
{
    cJSON *o = cJSON_CreateObject();

    if (o == NULL) {
        return;
    }
    (void)cJSON_AddStringToObject(o, "req_id", s_req_id);
    (void)cJSON_AddBoolToObject  (o, "ok", ctx->result.valid != 0u);
    /* 파일 자체는 싣지 않는다 — JSON 이 15MB 라 브로커에 부담이고 페이로드
     * 한계에도 걸린다. 경로만 알리고 소비자가 파일공유로 가져간다. */
    (void)cJSON_AddStringToObject(o, "pcd", ctx->result.path);
    (void)cJSON_AddNumberToObject(o, "points",       ctx->result.point_count);
    (void)cJSON_AddNumberToObject(o, "stm_reported", ctx->result.stm_reported);
    (void)cJSON_AddNumberToObject(o, "ts", (double)unix_ts());
    publish_json(T_ST_SCAN, o, 1, true);                    /* retained */
}

/* fatal = **작업이 중단됐고 사용자가 개입해야 다시 나간다.**
 *
 * "하드웨어가 고장났나" 가 아니라 **화면을 어떻게 그릴까**로 정의한다 —
 * 그게 이 플래그를 쓰는 쪽(Qt)이 실제로 물어보는 질문이기 때문이다:
 *     fatal=true   배너·모달로 크게. 조작을 막고 사용자를 부른다
 *     fatal=false  로그 한 줄. 스캔은 계속되거나 다시 시도하면 된다
 *
 * (Qt 는 이 필드를 이미 파싱하고 있었는데 데몬이 안 채워 항상 false 였다) */
static void publish_error(int code, const char *name, const char *msg, bool fatal)
{
    cJSON *o = cJSON_CreateObject();

    if (o == NULL) {
        return;
    }
    (void)cJSON_AddStringToObject(o, "req_id", s_req_id);
    (void)cJSON_AddNumberToObject(o, "code", code);
    (void)cJSON_AddStringToObject(o, "name", name);
    (void)cJSON_AddStringToObject(o, "msg",  msg);
    (void)cJSON_AddBoolToObject  (o, "fatal", fatal);
    (void)cJSON_AddNumberToObject(o, "ts", (double)unix_ts());
    publish_json(T_EV_ERROR, o, 1, false);
}

/* STM 오류코드별 fatal 판정. 표는 MQTT 계약 문서 §3.5 와 같아야 한다. */
static bool stm_err_is_fatal(uint8_t code)
{
    bool fatal;

    switch (code) {
    case 1:  /* ERR_BAD_CRC      — 프레임 하나가 깨진 것. 다음 프레임에 복구된다 */
    case 2:  /* ERR_BAD_LEN                                                    */
    case 4:  /* ERR_OUT_OF_RANGE — 입력값만 고치면 된다. 장비는 멀쩡하다        */
        fatal = false;
        break;
    default: /* 3 NOT_HOMED / 5 STALL / 6 LIDAR — 스캔이 못 나간다 */
        fatal = true;
        break;
    }
    return fatal;
}

/* ---------------------------------------------------------------------------
 *  수신 — 명령 파싱
 * ------------------------------------------------------------------------- */
static bool get_int(const cJSON *o, const char *key, int *out)
{
    const cJSON *v = cJSON_GetObjectItemCaseSensitive(o, key);
    bool ok = cJSON_IsNumber(v);

    if (ok) {
        *out = v->valueint;
    }
    return ok;
}

/* [a, b] 형태의 2원소 정수 배열 */
static bool get_pair(const cJSON *o, const char *key, int *a, int *b)
{
    const cJSON *arr = cJSON_GetObjectItemCaseSensitive(o, key);
    bool ok = false;

    if (cJSON_IsArray(arr) && (cJSON_GetArraySize(arr) == 2)) {
        const cJSON *x = cJSON_GetArrayItem(arr, 0);
        const cJSON *y = cJSON_GetArrayItem(arr, 1);
        if (cJSON_IsNumber(x) && cJSON_IsNumber(y)) {
            *a = x->valueint;
            *b = y->valueint;
            ok = true;
        }
    }
    return ok;
}

/* req_id 를 s_req_id 로 복사. 없으면 "-" 로 둔다.
 * 반환 false = 직전과 같은 id (QoS1 중복 배달) → 호출자가 무시할 것 */
static bool take_req_id(const cJSON *o)
{
    const cJSON *v = cJSON_GetObjectItemCaseSensitive(o, "req_id");
    const char  *s = cJSON_IsString(v) ? v->valuestring : "-";
    bool fresh;

    fresh = (strncmp(s, s_last_req_id, REQ_ID_LEN - 1u) != 0);
    (void)snprintf(s_req_id, sizeof(s_req_id), "%s", s);
    (void)snprintf(s_last_req_id, sizeof(s_last_req_id), "%s", s);
    return fresh;
}

static void handle_cmd_scan(struct shared_ctx *ctx, const cJSON *o)
{
    int p0 = 0, p1 = 0, t0 = 0, t1 = 0, step = 0, h = 0;

    if (!get_pair(o, "pan_ddeg", &p0, &p1) ||
        !get_pair(o, "tilt_ddeg", &t0, &t1) ||
        !get_int (o, "step_ddeg", &step)) {
        /* ⚠️ 예전에는 4(ERR_OUT_OF_RANGE)를 빌려 썼다. 그건 STM32 가 "스캔
         *   범위 밖" 에 쓰는 코드라, Qt 가 code=4 를 받고도 STM 이 거절한
         *   건지 데몬이 페이로드를 못 읽은 건지 알 수 없었다. 데몬 자신의
         *   판정이므로 100번대를 쓴다. */
        publish_error((int)NOTICE_BAD_REQUEST, "ERR_BAD_REQUEST",
                      "필수 필드 누락/형식 오류", false);
        core_log(ctx->core, "MQTT", "cmd/scan 파싱 실패");
        return;
    }
    (void)get_int(o, "sensor_height_mm", &h);   /* 선택 */

    /* 값 검증은 코어(scan_request_valid)가 다시 한다. 여기선 채우기만. */
    ctx->req.pan_start_ddeg   = (int16_t)p0;
    ctx->req.pan_end_ddeg     = (int16_t)p1;
    ctx->req.tilt_start_ddeg  = (int16_t)t0;
    ctx->req.tilt_end_ddeg    = (int16_t)t1;
    ctx->req.step_ddeg        = (uint16_t)step;
    ctx->req.sensor_height_mm = h;
    ctx->req.valid            = 1u;             /* ← 코어 FSM 트리거 */

    core_log(ctx->core, "MQTT", "scan 요청 [%s] pan[%d..%d] tilt[%d..%d] step=%d",
             s_req_id, p0, p1, t0, t1, step);
}

static void on_message(struct mosquitto *m, void *user,
                       const struct mosquitto_message *msg)
{
    cJSON *o;
    struct shared_ctx *ctx = s_ctx;

    (void)m;
    (void)user;

    if ((ctx == NULL) || (msg == NULL) || (msg->topic == NULL)) {
        return;
    }

    /* payload 가 없거나 JSON 이 아니어도 명령 자체는 유효할 수 있다
     * (stop/home/disarm 은 req_id 만 있으면 되고 그마저 선택). */
    o = (msg->payloadlen > 0)
        ? cJSON_ParseWithLength((const char *)msg->payload, (size_t)msg->payloadlen)
        : cJSON_CreateObject();
    if (o == NULL) {
        core_log(ctx->core, "MQTT", "%s: JSON 파싱 실패 — 무시", msg->topic);
        return;
    }

    if (!take_req_id(o)) {
        /* QoS 1 은 at-least-once 라 같은 명령이 두 번 올 수 있다.
         * 스캔이 5분이라 중복 시작은 실제로 사고가 된다. */
        core_log(ctx->core, "MQTT", "%s: 중복 req_id[%s] — 무시",
                 msg->topic, s_req_id);
        cJSON_Delete(o);
        return;
    }

    if (strcmp(msg->topic, T_CMD_SCAN) == 0) {
        handle_cmd_scan(ctx, o);
    } else if (strcmp(msg->topic, T_CMD_STOP) == 0) {
        ctx->req_scan_stop = 1u;
        core_log(ctx->core, "MQTT", "stop 요청 [%s]", s_req_id);
    } else if (strcmp(msg->topic, T_CMD_DISARM) == 0) {
        ctx->req_disarm = 1u;
        s_user_disarm   = true;         /* 아래 mqtt_on_state 에서 사유 판정에 쓴다 */
        core_log(ctx->core, "MQTT", "disarm 요청 [%s]", s_req_id);
    } else if (strcmp(msg->topic, T_CMD_REARM) == 0) {
        /* 복구 가능 여부(링크 생존·현재 상태)는 코어가 판정한다. 여기서
         * 미리 걸러내면 판정 기준이 두 곳으로 갈라진다. */
        ctx->req_rearm = 1u;
        core_log(ctx->core, "MQTT", "rearm 요청 [%s]", s_req_id);
    } else if (strcmp(msg->topic, T_CMD_HOME) == 0) {
        /* 스캔 없이 홈만 세운다. 코어는 스캔 직전에 어차피 홈을 다시 잡으므로
         * 스캔 전에는 불필요하지만, 설치·정비 때 축을 홈 자세로 보내 눈으로
         * 확인하는 용도로 쓴다. 수용 여부(IDLE 인가, 중복인가)는 코어가 본다. */
        ctx->req_home = 1u;
        core_log(ctx->core, "MQTT", "home 요청 [%s]", s_req_id);
    } else {
        core_log(ctx->core, "MQTT", "알 수 없는 토픽: %s", msg->topic);
    }
    cJSON_Delete(o);
}

/* ---------------------------------------------------------------------------
 *  접속 콜백
 * ------------------------------------------------------------------------- */
static void on_connect(struct mosquitto *m, void *user, int rc)
{
    (void)user;
    if (rc == 0) {
        s_connected = true;
        (void)mosquitto_subscribe(m, NULL, T_CMD_ALL, 1);
        if (s_ctx != NULL) {
            core_log(s_ctx->core, "MQTT", "브로커 접속 — %s 구독", T_CMD_ALL);
            publish_state(s_ctx);           /* retained 로 현재 상태 즉시 게시 */
        }
    } else {
        s_connected = false;
        if (s_ctx != NULL) {
            core_log(s_ctx->core, "MQTT", "접속 거부: %s",
                     mosquitto_connack_string(rc));
        }
    }
}

static void on_disconnect(struct mosquitto *m, void *user, int rc)
{
    (void)m;
    (void)user;
    s_connected = false;
    if (s_ctx != NULL) {
        core_log(s_ctx->core, "MQTT", "브로커 연결 끊김 (rc=%d) — 재접속 대기", rc);
    }
}

/* ---------------------------------------------------------------------------
 *  모듈 인터페이스
 * ------------------------------------------------------------------------- */
static int mqtt_init(struct shared_ctx *ctx)
{
    const char *host = env_or(ENV_HOST, DEF_HOST);
    const int   port = atoi(env_or(ENV_PORT, "8883"));
    const char *ca   = env_or(ENV_CA,   DEF_CA);
    const char *cert = env_or(ENV_CERT, DEF_CERT);
    const char *key  = env_or(ENV_KEY,  DEF_KEY);
    char will[128];
    int  rc;

    s_ctx = ctx;
    (void)snprintf(s_req_id,      sizeof(s_req_id),      "-");
    s_last_req_id[0] = '\0';

    mosquitto_lib_init();
    s_mosq = mosquitto_new(MQTT_CLIENT_ID, true, NULL);
    if (s_mosq == NULL) {
        core_log(ctx->core, "MQTT", "mosquitto_new 실패 — MQTT 없이 계속");
        return 0;                      /* degraded — 데몬은 계속 살아야 한다 */
    }

    mosquitto_connect_callback_set(s_mosq, on_connect);
    mosquitto_disconnect_callback_set(s_mosq, on_disconnect);
    mosquitto_message_callback_set(s_mosq, on_message);

    /* LWT: 데몬이 죽으면 브로커가 대신 OFFLINE 을 뿌린다.
     * 이게 없으면 Qt 화면에 낡은 IDLE 이 남아 조작자가 "준비됨" 으로 오인한다. */
    (void)snprintf(will, sizeof(will),
                   "{\"state\":\"OFFLINE\",\"online\":false,\"ts\":%ld}",
                   unix_ts());
    (void)mosquitto_will_set(s_mosq, T_ST_DAEMON,
                             (int)strlen(will), will, 1, true);

    /* mTLS. 3·4번 인자(내 인증서·개인키)를 채우는 것이 단방향 TLS 와의 차이다.
     * 파일이 없으면 tls_set 이 실패하는데, 그래도 데몬은 계속 구동한다 —
     * MQTT 가 안 붙어도 CLI(--scan)로 스캔이 되어야 하기 때문. */
    rc = mosquitto_tls_set(s_mosq, ca, NULL, cert, key, NULL);
    if (rc != MOSQ_ERR_SUCCESS) {
        core_log(ctx->core, "MQTT",
                 "TLS 설정 실패 (%s) — 인증서 확인: %s / %s / %s",
                 mosquitto_strerror(rc), ca, cert, key);
        s_tls_ready = false;
    } else {
        s_tls_ready = true;
        (void)mosquitto_tls_opts_set(s_mosq, 1 /*SSL_VERIFY_PEER*/,
                                     "tlsv1.2", NULL);
    }

    /* 비동기 접속 — 여기서 블로킹하면 부팅이 브로커에 묶인다.
     * 실패해도 loop_misc 가 주기적으로 재시도한다. */
    rc = mosquitto_connect_async(s_mosq, host, port, MQTT_KEEPALIVE_S);
    if (rc != MOSQ_ERR_SUCCESS) {
        core_log(ctx->core, "MQTT", "%s:%d 접속 시도 실패 (%s) — 재시도 예정",
                 host, port, mosquitto_strerror(rc));
    } else {
        core_log(ctx->core, "MQTT", "%s:%d 접속 시도 (mTLS %s)",
                 host, port, s_tls_ready ? "on" : "off");
    }
    (void)mosquitto_reconnect_delay_set(s_mosq, 1, 30, true);
    return 0;
}

static int mqtt_get_fd(void)
{
    /* 재연결 시 fd 가 바뀐다. 코어가 매 tick 이 값을 확인해 epoll 을 갱신한다. */
    return (s_mosq != NULL) ? mosquitto_socket(s_mosq) : -1;
}

/* cppcheck-suppress constParameterCallback ; 콜백 ABI 고정 */
static void mqtt_on_event(struct shared_ctx *ctx)
{
    (void)ctx;
    if (s_mosq == NULL) {
        return;
    }
    (void)mosquitto_loop_read(s_mosq, 1);
    if (mosquitto_want_write(s_mosq)) {
        (void)mosquitto_loop_write(s_mosq, 1);
    }
}

/* cppcheck-suppress constParameterCallback ; 콜백 ABI 고정 */
static void mqtt_on_tick(struct shared_ctx *ctx, daemon_state_t state)
{
    const uint64_t now = now_ms();

    if (s_mosq == NULL) {
        return;
    }
    /* keepalive + 재접속. 접속이 끊긴 동안에도 반드시 호출해야 한다. */
    (void)mosquitto_loop_misc(s_mosq);
    if (mosquitto_want_write(s_mosq)) {
        (void)mosquitto_loop_write(s_mosq, 1);
    }
    if (!s_connected) {
        return;
    }

    /* 스캔 중에는 진행률을 주기 발행 (QoS0 — 유실돼도 무방) */
    if ((state == ST_SCANNING) &&
        ((now - s_last_progress_ms) >= PROGRESS_PERIOD_MS)) {
        publish_progress(ctx);
        s_last_progress_ms = now;
    }

    /* 상태는 변화 시 즉시 + 무변화여도 주기적으로 (retained 갱신 겸 생존 신호) */
    {
        const uint8_t verdict = level_verdict(ctx);

        /* ⚠️ 수평 판정이 뒤집히면 하트비트를 기다리지 않고 바로 알린다.
         *   "초록 뜰 때까지 각도를 맞춘다" 는 조작 UX 인데 5초 지연이면
         *   나사를 돌리고 다섯을 세야 해서 쓸 수가 없다. */
        if ((verdict != s_last_level_verdict) ||
            ((now - s_last_state_ms) >= STATE_HEARTBEAT_MS)) {
            publish_state(ctx);
            s_last_level_verdict = verdict;
        }
    }

    /* 코어/모듈이 스스로 판단한 사건(수평 게이트 거부·홈 타임아웃·업로드 실패).
     * STM 이 올린 오류와 경로가 다르다 — 그쪽은 아래 link.last_err 다. */
    if (ctx->notice.seq != s_notice_seq) {
        s_notice_seq = ctx->notice.seq;
        publish_error((int)ctx->notice.code, ctx->notice.name,
                      ctx->notice.msg, ctx->notice.fatal != 0u);
    }

    /* STM 오류가 새로 뜨면 이벤트로 알린다 (같은 코드 반복 발행 방지) */
    if ((ctx->link.last_err != 0u) && (ctx->link.last_err != s_last_err_sent)) {
        publish_error(ctx->link.last_err, "STM_ERROR", "STM32 오류 통지",
                      stm_err_is_fatal(ctx->link.last_err));
        s_last_err_sent = ctx->link.last_err;
    } else if (ctx->link.last_err == 0u) {
        s_last_err_sent = 0u;
    } else {
        /* 같은 오류 지속 — 재발행하지 않는다 */
    }
}

/* cppcheck-suppress constParameterCallback ; 콜백 ABI 고정 */
static void mqtt_on_state(struct shared_ctx *ctx,
                          daemon_state_t old_st, daemon_state_t new_st)
{
    (void)old_st;

    if ((s_mosq == NULL) || !s_connected) {
        return;
    }
    publish_state(ctx);                 /* 전이는 즉시 알린다 */

    if (new_st == ST_EXPORT) {
        /* 파일 마감은 코어가 EXPORT 진입 시 수행하므로 result 가 채워져 있다. */
        publish_scan_result(ctx);
    } else if (new_st == ST_DISARM) {
        /* ⚠️ 예전에는 여기서 무조건 오류를 발행했다. 스캔 후 자동 DISARM 이
         *   생기면서 **성공한 스캔마다 오류가 하나씩** 나가게 됐다.
         *   사유를 가려서, 알릴 가치가 있을 때만 보낸다. */
        const bool user = s_user_disarm;
        s_user_disarm = false;

        if (ctx->link.link_alive == 0u) {
            publish_error((int)NOTICE_DISARM, "ERR_DISARM",
                          "링크 두절 — 배선/전원 확인", true);
        } else if (ctx->link.last_err != 0u) {
            publish_error((int)NOTICE_DISARM, "ERR_DISARM",
                          "STM32 오류로 안전정지", true);
        } else if (user) {
            /* 사용자가 직접 눌렀다. 장비가 고장난 건 아니지만 **REARM 전까지
             * 스캔이 안 나가므로** fatal 이다(위 정의 참조). 여러 콘솔이 붙어
             * 있을 수 있어 "누가 세웠다" 도 남겨야 한다.
             * 이름은 Qt 데모 브리지와 맞춘다 — 같은 사건이 실물/데모에서
             * 다르게 보이면 UI 를 두 번 만들게 된다. */
            publish_error((int)NOTICE_DISARM, "USER_DISARM",
                          "사용자 안전정지", true);
        } else {
            /* 스캔 후 되감기 유예 뒤의 자동 DISARM = 정상 종료.
             * state/daemon 전이만으로 충분하고 오류가 아니다. */
            core_log(ctx->core, "MQTT", "자동 DISARM (정상) — 오류 발행 안 함");
        }
    } else {
        /* 그 외 전이는 상태 발행으로 충분 */
        if (new_st == ST_IDLE) {
            s_user_disarm = false;      /* rearm 등으로 빠져나왔다 — 표식 정리 */
        }
    }
}

static void mqtt_deinit(struct shared_ctx *ctx)
{
    if (s_mosq != NULL) {
        /* 정상 종료는 LWT 가 발동하지 않으므로 OFFLINE 을 직접 남긴다. */
        if (s_connected) {
            cJSON *o = cJSON_CreateObject();
            if (o != NULL) {
                (void)cJSON_AddStringToObject(o, "state", "OFFLINE");
                (void)cJSON_AddBoolToObject  (o, "online", false);
                (void)cJSON_AddNumberToObject(o, "ts", (double)unix_ts());
                publish_json(T_ST_DAEMON, o, 1, true);
            }
            (void)mosquitto_loop_write(s_mosq, 1);
            (void)mosquitto_disconnect(s_mosq);
        }
        mosquitto_destroy(s_mosq);
        s_mosq = NULL;
    }
    mosquitto_lib_cleanup();
    s_connected = false;
    s_ctx = NULL;
    if (ctx != NULL) {
        core_log(ctx->core, "MQTT", "정리 완료");
    }
}

#else  /* ADTS_NO_MQTT — 라이브러리 없이 빌드 */

#include <stdio.h>

static int mqtt_init(struct shared_ctx *ctx)
{
    core_log(ctx->core, "MQTT",
             "비활성 빌드 (libmosquitto/libcjson 없음) — CLI --scan 만 가능");
    return 0;
}
static int  mqtt_get_fd(void) { return -1; }
static void mqtt_on_event(struct shared_ctx *ctx) { (void)ctx; }
static void mqtt_on_tick(struct shared_ctx *ctx, daemon_state_t state)
{ (void)ctx; (void)state; }
static void mqtt_on_state(struct shared_ctx *ctx,
                          daemon_state_t old_st, daemon_state_t new_st)
{ (void)ctx; (void)old_st; (void)new_st; }
static void mqtt_deinit(struct shared_ctx *ctx) { (void)ctx; }

#endif /* ADTS_NO_MQTT */

static const struct daemon_module k_mqtt = {
    "mqtt",
    mqtt_init,
    mqtt_get_fd,
    mqtt_on_event,
    mqtt_on_tick,
    mqtt_on_state,
    mqtt_deinit,
};

const struct daemon_module *mqtt_module_get(void)
{
    return &k_mqtt;
}
