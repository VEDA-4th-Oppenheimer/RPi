/* ============================================================================
 *  mqtt_module.c  --  RPi Mosquitto 브로커 클라이언트 (제어·상태 통신)
 *  담당(계약): 데몬=이현우 · Qt=송영빈 · 브로커/인증서=이광진
 *
 *  ★ 이 파일이 구현해야 하는 계약은 ../../docs/MQTT_INTERFACE_CONTRACT.md (v1.0) 다.
 *    토픽/페이로드/QoS/retain 을 바꿔야 하면 그 문서를 먼저 고친다.
 *
 *  연결: 데몬은 RPi 자신에 상주한 브로커에 loopback 으로 붙는다. 계약서 §1의
 *  "평문 1883 은 외부에 열지 않음"은 브로커가 8883(mTLS)만 외부에 노출한다는
 *  뜻으로 해석했다 — RPi 안에서 데몬↔브로커 구간까지 TLS 를 씌울 이유는 없어
 *  이 모듈은 localhost:1883 평문으로 접속한다. Qt(외부)는 8883/mTLS 로 접속한다.
 *
 *  토픽(계약 §2): adts/kit1/cmd/{scan,stop,home,disarm} 구독,
 *                 adts/kit1/state/{daemon,scan}, adts/kit1/event/{progress,error} 발행.
 *
 *  ★ epoll 통합 (단일 스레드 유지):
 *      get_fd()  -> mosquitto_socket(mosq)  를 코어 epoll 에 등록
 *      on_event()-> mosquitto_loop_read/write()
 *      on_tick() -> mosquitto_loop_misc()   (keepalive) + 무조건 loop_write 플러시
 *    ⚠️ mosquitto_loop_start() (자체 스레드) 금지 — 콜백이 다른 스레드에서 불려
 *       shared_ctx/FSM 에 락이 필요해지고 무락 설계가 깨진다.
 *    ⚠️ 코어가 모듈 fd 를 EPOLLIN 으로만 등록해(main.c core_setup) EPOLLOUT 이벤트를
 *       못 받는다. 성공적인 non-blocking connect 직후엔 소켓이 "쓰기 가능"으로만
 *       깨어나므로, on_event 의 loop_write 호출만 믿으면 CONNECT 패킷이 영영 안
 *       나갈 수 있다. 그래서 on_tick(100ms 마다, epoll 상태와 무관하게 항상 호출)
 *       에서도 mosquitto_want_write()/loop_write() 를 무조건 확인한다.
 *    ⚠️ 재연결 시 소켓 fd 가 바뀔 수 있는데, 코어에 fd 재등록 API가 아직 없어
 *       지금은 처리하지 못한다 (TODO — core_* API 확장 필요, 이현우 협의).
 *       대신 fd 가 바뀌면 로그로 크게 남겨 무음 장애를 막는다.
 * ==========================================================================*/
#include "daemon_module.h"

#include <mosquitto.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define BROKER_HOST      "localhost"
#define BROKER_PORT      1883
#define BROKER_KEEPALIVE 30
#define MQTT_QOS_RELIABLE 1
#define MQTT_QOS_BESTEFFORT 0
#define KIT_ID "kit1"

#define TOPIC_CMD_SCAN       "adts/" KIT_ID "/cmd/scan"
#define TOPIC_CMD_STOP       "adts/" KIT_ID "/cmd/stop"
#define TOPIC_CMD_HOME       "adts/" KIT_ID "/cmd/home"
#define TOPIC_CMD_DISARM     "adts/" KIT_ID "/cmd/disarm"
#define TOPIC_STATE_DAEMON   "adts/" KIT_ID "/state/daemon"
#define TOPIC_STATE_SCAN     "adts/" KIT_ID "/state/scan"
#define TOPIC_EVENT_PROGRESS "adts/" KIT_ID "/event/progress"
#define TOPIC_EVENT_ERROR    "adts/" KIT_ID "/event/error"

#define REQ_ID_MAX          32u
#define PAYLOAD_COPY_MAX    511u   /* CWE-120: 고정 버퍼 + 명시적 절단 */

#define RECONNECT_EVERY_TICKS  10u  /* on_tick 100ms 기준 * 10 = ~1초 */
#define HEARTBEAT_EVERY_TICKS  50u  /* ~5초 — 계약 §3.3 "최소 5초에 한 번" */
#define PROGRESS_EVERY_TICKS    5u  /* ~0.5초 = 2Hz — 계약 §3.6 */

static struct mosquitto *g_mosq = NULL;
static bool     g_connected = false;
static unsigned g_tick_count = 0u;
static int      g_registered_fd = -1;   /* mqtt_get_fd() 가 코어에 처음 돌려준 fd */

static char    g_req_id[REQ_ID_MAX + 1u] = "";           /* 최근 명령의 req_id (상관용) */
static char    g_last_scan_req_id[REQ_ID_MAX + 1u] = ""; /* 중복 cmd/scan 무시용 */
static uint8_t g_last_reported_err = 0u;
static bool    g_last_link_alive = true;

/* ---------------------------------------------------------------------------
 *  최소 JSON 추출 — 외부 라이브러리 없이(이 저장소 관례) 필요한 키만 뽑는다.
 * ------------------------------------------------------------------------- */
static bool json_find_long(const char *json, const char *key, long *out)
{
    char needle[40];
    (void)snprintf(needle, sizeof(needle), "\"%s\"", key);
    const char *k = strstr(json, needle);
    if (k == NULL) {
        return false;
    }
    const char *colon = strchr(k, ':');
    if (colon == NULL) {
        return false;
    }
    char *end = NULL;
    const long v = strtol(colon + 1, &end, 10);
    if (end == (colon + 1)) {
        return false;
    }
    *out = v;
    return true;
}

/* "key":[a,b] 형태의 정수 쌍 (pan_ddeg/tilt_ddeg) */
static bool json_find_int_pair(const char *json, const char *key, long *a, long *b)
{
    char needle[40];
    (void)snprintf(needle, sizeof(needle), "\"%s\"", key);
    const char *k = strstr(json, needle);
    if (k == NULL) {
        return false;
    }
    const char *bracket = strchr(k, '[');
    if (bracket == NULL) {
        return false;
    }
    char *end_a = NULL;
    const long va = strtol(bracket + 1, &end_a, 10);
    if (end_a == (bracket + 1)) {
        return false;
    }
    const char *comma = strchr(end_a, ',');
    if (comma == NULL) {
        return false;
    }
    char *end_b = NULL;
    const long vb = strtol(comma + 1, &end_b, 10);
    if (end_b == (comma + 1)) {
        return false;
    }
    *a = va;
    *b = vb;
    return true;
}

/* "key":"value" 형태의 문자열 (req_id) — outsz 는 NUL 포함 버퍼 크기 */
static bool json_find_str(const char *json, const char *key, char *out, size_t outsz)
{
    out[0] = '\0';
    char needle[40];
    (void)snprintf(needle, sizeof(needle), "\"%s\"", key);
    const char *k = strstr(json, needle);
    if (k == NULL) {
        return false;
    }
    const char *colon = strchr(k, ':');
    if (colon == NULL) {
        return false;
    }
    const char *p = colon + 1;
    while ((*p == ' ') || (*p == '\t')) {
        ++p;
    }
    if (*p != '"') {
        return false;
    }
    ++p;
    size_t i = 0u;
    while ((p[i] != '\0') && (p[i] != '"') && (i < (outsz - 1u))) {
        out[i] = p[i];
        ++i;
    }
    out[i] = '\0';
    return i > 0u;
}

/* ---------------------------------------------------------------------------
 *  cmd/scan payload -> struct scan_request (계약 §3.1)
 * ------------------------------------------------------------------------- */
static void parse_scan_cmd(const char *json, struct scan_request *req)
{
    long a = 0, b = 0, v = 0;
    memset(req, 0, sizeof(*req));
    req->step_ddeg = 10u;   /* 기본값(1.0도) — 파싱 실패 필드도 안전값 유지 */

    if (json_find_int_pair(json, "pan_ddeg", &a, &b)) {
        req->pan_start_ddeg = (int16_t)a;
        req->pan_end_ddeg   = (int16_t)b;
    }
    if (json_find_int_pair(json, "tilt_ddeg", &a, &b)) {
        req->tilt_start_ddeg = (int16_t)a;
        req->tilt_end_ddeg   = (int16_t)b;
    }
    if (json_find_long(json, "step_ddeg", &v) && (v > 0)) {
        req->step_ddeg = (uint16_t)v;
    }
    if (json_find_long(json, "sensor_height_mm", &v)) {
        req->z_offset_mm = (int32_t)v;
    }
    req->valid = 1u;

    /* 계약 §3.1: 팬이 180도(1800dd) 이상이면 방위 0/180 중복 측정 — 거부하지 않고 경고만. */
    const int32_t pan_span = (int32_t)req->pan_end_ddeg - (int32_t)req->pan_start_ddeg;
    if (pan_span >= 1800) {
        (void)fprintf(stderr,
            "[mqtt    ] 경고: pan 범위가 180도 이상(%d) — 방위 0/180 중복 측정 가능 "
            "(0~1790 권장)\n", pan_span);
    }
}

/* ---------------------------------------------------------------------------
 *  발행 헬퍼
 * ------------------------------------------------------------------------- */
static void publish_state_daemon(const struct shared_ctx *ctx)
{
    char payload[320];
    (void)snprintf(payload, sizeof(payload),
        "{\"state\":\"%s\",\"online\":true,\"link_alive\":%s,\"homed\":%s,"
        "\"scanning\":%s,\"cur_pan_ddeg\":%d,\"cur_tilt_ddeg\":%d,\"last_err\":%u,"
        "\"level\":{\"valid\":%s,\"roll_deg\":%.2f,\"pitch_deg\":%.2f},\"ts\":%ld}",
        daemon_state_str(ctx->state),
        ctx->link.link_alive ? "true" : "false",
        ctx->link.homed      ? "true" : "false",
        ctx->link.scanning   ? "true" : "false",
        ctx->link.cur_pan_ddeg, ctx->link.cur_tilt_ddeg,
        (unsigned)ctx->link.last_err,
        ctx->level.valid ? "true" : "false",
        (double)ctx->level.roll_deg, (double)ctx->level.pitch_deg,
        (long)time(NULL));
    (void)mosquitto_publish(g_mosq, NULL, TOPIC_STATE_DAEMON,
                             (int)strlen(payload), payload, MQTT_QOS_RELIABLE, true);
}

static void publish_state_scan(const struct shared_ctx *ctx)
{
    char payload[768];
    (void)snprintf(payload, sizeof(payload),
        "{\"req_id\":\"%s\",\"ok\":%s,\"session_id\":\"%s\",\"scan_id\":\"%s\","
        "\"pcd\":\"%s\",\"json\":\"%s\",\"rows\":%u,\"columns\":%u,"
        "\"points\":%u,\"expected\":%u,\"duration_s\":%.1f,\"ts\":%ld}",
        g_req_id, ctx->result.valid ? "true" : "false",
        ctx->result.session_id, ctx->result.scan_id,
        ctx->result.path, ctx->result.json_path,
        (unsigned)ctx->result.rows, (unsigned)ctx->result.columns,
        (unsigned)ctx->result.point_count, (unsigned)ctx->progress.expected,
        ctx->result.duration_s, (long)time(NULL));
    (void)mosquitto_publish(g_mosq, NULL, TOPIC_STATE_SCAN,
                             (int)strlen(payload), payload, MQTT_QOS_RELIABLE, true);
}

static void publish_progress(const struct shared_ctx *ctx)
{
    char payload[160];
    (void)snprintf(payload, sizeof(payload),
        "{\"req_id\":\"%s\",\"points\":%u,\"expected\":%u,\"percent\":%u,\"ts\":%ld}",
        g_req_id, (unsigned)ctx->progress.points, (unsigned)ctx->progress.expected,
        (unsigned)ctx->progress.percent, (long)time(NULL));
    (void)mosquitto_publish(g_mosq, NULL, TOPIC_EVENT_PROGRESS,
                             (int)strlen(payload), payload, MQTT_QOS_BESTEFFORT, false);
}

static const char *err_name(uint8_t code)
{
    switch (code) {
        case 1:   return "ERR_BAD_CRC";
        case 2:   return "ERR_BAD_LEN";
        case 3:   return "ERR_NOT_HOMED";
        case 4:   return "ERR_OUT_OF_RANGE";
        case 5:   return "ERR_STALL";
        case 6:   return "ERR_LIDAR";
        case 100: return "ERR_LINK_DEAD";
        case 101: return "ERR_HOME_TIMEOUT";
        default:  return "ERR_UNKNOWN";
    }
}

static void publish_error(uint8_t code, bool fatal)
{
    char payload[224];
    (void)snprintf(payload, sizeof(payload),
        "{\"req_id\":\"%s\",\"code\":%u,\"name\":\"%s\",\"msg\":\"\",\"fatal\":%s,\"ts\":%ld}",
        g_req_id, (unsigned)code, err_name(code), fatal ? "true" : "false", (long)time(NULL));
    (void)mosquitto_publish(g_mosq, NULL, TOPIC_EVENT_ERROR,
                             (int)strlen(payload), payload, MQTT_QOS_RELIABLE, false);
}

/* last_err/link_alive 변화를 감지해 event/error 를 낸다 (계약 §3.5) */
static void check_and_publish_errors(const struct shared_ctx *ctx)
{
    if ((ctx->link.last_err != 0u) && (ctx->link.last_err != g_last_reported_err)) {
        publish_error(ctx->link.last_err, true);
    }
    g_last_reported_err = ctx->link.last_err;

    if (g_last_link_alive && !ctx->link.link_alive) {
        publish_error(100u, true);   /* ERR_LINK_DEAD */
    }
    g_last_link_alive = ctx->link.link_alive;
}

/* ---------------------------------------------------------------------------
 *  libmosquitto 콜백 — mosquitto_loop_read() 안에서 동기 호출된다(별도 스레드 아님).
 * ------------------------------------------------------------------------- */
static void on_connect(struct mosquitto *mosq, void *obj, int rc)
{
    (void)obj;
    if (rc != 0) {
        (void)fprintf(stderr, "[mqtt    ] connect 거부: %s\n", mosquitto_connack_string(rc));
        return;
    }
    g_connected = true;
    (void)mosquitto_subscribe(mosq, NULL, TOPIC_CMD_SCAN, MQTT_QOS_RELIABLE);
    (void)mosquitto_subscribe(mosq, NULL, TOPIC_CMD_STOP, MQTT_QOS_RELIABLE);
    (void)mosquitto_subscribe(mosq, NULL, TOPIC_CMD_HOME, MQTT_QOS_RELIABLE);
    (void)mosquitto_subscribe(mosq, NULL, TOPIC_CMD_DISARM, MQTT_QOS_RELIABLE);
    (void)fprintf(stderr, "[mqtt    ] 브로커 연결됨 (%s:%d), cmd/* 4개 구독\n",
                  BROKER_HOST, BROKER_PORT);
}

static void on_disconnect(struct mosquitto *mosq, void *obj, int rc)
{
    (void)mosq;
    (void)obj;
    g_connected = false;
    (void)fprintf(stderr, "[mqtt    ] 브로커 연결 끊김 (rc=%d) — 다음 tick 에 재연결 시도\n", rc);
}

static void on_message(struct mosquitto *mosq, void *obj, const struct mosquitto_message *msg)
{
    (void)mosq;
    struct shared_ctx *ctx = (struct shared_ctx *)obj;
    if ((msg->payload == NULL) || (msg->payloadlen <= 0)) {
        return;
    }

    /* CWE-120: payload 는 NUL 종단이 보장되지 않는다 — 고정 버퍼 + 명시적 절단. */
    char buf[PAYLOAD_COPY_MAX + 1u];
    const size_t len = ((size_t)msg->payloadlen < PAYLOAD_COPY_MAX)
                            ? (size_t)msg->payloadlen : PAYLOAD_COPY_MAX;
    memcpy(buf, msg->payload, len);
    buf[len] = '\0';

    char incoming_req_id[REQ_ID_MAX + 1u] = "";
    (void)json_find_str(buf, "req_id", incoming_req_id, sizeof(incoming_req_id));
    if (incoming_req_id[0] != '\0') {
        (void)snprintf(g_req_id, sizeof(g_req_id), "%s", incoming_req_id);
    }

    if (strcmp(msg->topic, TOPIC_CMD_SCAN) == 0) {
        /* 계약 §4: 같은 req_id 가 연속으로 오면 뒤엣것은 무시(중복 배달 대비). */
        if ((incoming_req_id[0] != '\0')
            && (strcmp(incoming_req_id, g_last_scan_req_id) == 0)) {
            (void)fprintf(stderr, "[mqtt    ] 중복 req_id(%s) — cmd/scan 무시\n", incoming_req_id);
            return;
        }
        parse_scan_cmd(buf, &ctx->req);
        (void)snprintf(g_last_scan_req_id, sizeof(g_last_scan_req_id), "%s", incoming_req_id);
        (void)fprintf(stderr,
            "[mqtt    ] cmd/scan 수신 (req_id=%s) — pan[%d..%d] tilt[%d..%d] step=%u\n",
            incoming_req_id, ctx->req.pan_start_ddeg, ctx->req.pan_end_ddeg,
            ctx->req.tilt_start_ddeg, ctx->req.tilt_end_ddeg, (unsigned)ctx->req.step_ddeg);
    } else if (strcmp(msg->topic, TOPIC_CMD_STOP) == 0) {
        ctx->req_scan_stop = 1u;
        (void)fprintf(stderr, "[mqtt    ] cmd/stop 수신 (req_id=%s)\n", incoming_req_id);
    } else if (strcmp(msg->topic, TOPIC_CMD_HOME) == 0) {
        /* TODO: 코어에 "스캔 없이 홈만" 트리거하는 API가 아직 없다(SCAN_START 진입 시
         * 자동으로 홈을 겸하는 경로만 존재). 계약 §3.2 에도 "보통 불필요"라 당장은
         * 로그만 남기고 무시한다 — 필요해지면 core_request_home() 류 API를 이현우와 협의. */
        (void)fprintf(stderr,
            "[mqtt    ] cmd/home 수신 (req_id=%s) — TODO: 단독 홈 API 미구현, 무시\n",
            incoming_req_id);
    } else if (strcmp(msg->topic, TOPIC_CMD_DISARM) == 0) {
        ctx->req_disarm = 1u;
        (void)fprintf(stderr, "[mqtt    ] cmd/disarm 수신 (req_id=%s)\n", incoming_req_id);
    }
}

/* ---------------------------------------------------------------------------
 *  daemon_module 인터페이스
 * ------------------------------------------------------------------------- */
static int mqtt_init(struct shared_ctx *ctx)
{
    mosquitto_lib_init();

    g_mosq = mosquitto_new(NULL /* 자동 client id */, true, ctx);
    if (g_mosq == NULL) {
        (void)fprintf(stderr, "[mqtt    ] mosquitto_new 실패\n");
        return -1;
    }

    /* LWT (계약 §5.2): 데몬이 죽거나 링크가 끊기면 브로커가 대신 발행한다. */
    static const char *kLastWill = "{\"state\":\"OFFLINE\",\"online\":false}";
    (void)mosquitto_will_set(g_mosq, TOPIC_STATE_DAEMON, (int)strlen(kLastWill), kLastWill,
                              MQTT_QOS_RELIABLE, true);

    mosquitto_connect_callback_set(g_mosq, on_connect);
    mosquitto_disconnect_callback_set(g_mosq, on_disconnect);
    mosquitto_message_callback_set(g_mosq, on_message);

    if (mosquitto_connect_async(g_mosq, BROKER_HOST, BROKER_PORT, BROKER_KEEPALIVE)
        != MOSQ_ERR_SUCCESS) {
        (void)fprintf(stderr,
            "[mqtt    ] connect_async 실패 — degraded 로 계속(재시도는 on_tick)\n");
    }

    (void)fprintf(stderr, "[mqtt    ] init 완료 (%s:%d loopback; 외부는 8883/mTLS, 계약서 §1)\n",
                  BROKER_HOST, BROKER_PORT);
    return 0;
}

static int mqtt_get_fd(void)
{
    if (g_mosq == NULL) {
        return -1;
    }
    const int fd = mosquitto_socket(g_mosq);
    if (g_registered_fd < 0) {
        g_registered_fd = fd;   /* 코어가 이 첫 값을 epoll 에 등록한다 */
    }
    return fd;
}

/* cppcheck-suppress constParameterCallback ; on_event 는 daemon_module 콜백 ABI
 * (void(*)(struct shared_ctx*)) 라 ctx 를 const 로 못 바꾼다. */
static void mqtt_on_event(struct shared_ctx *ctx)
{
    (void)ctx;
    if (g_mosq == NULL) {
        return;
    }
    if (mosquitto_loop_read(g_mosq, 1) != MOSQ_ERR_SUCCESS) {
        return;   /* 다음 tick 의 재연결 로직이 처리한다 */
    }
    if (mosquitto_want_write(g_mosq)) {
        (void)mosquitto_loop_write(g_mosq, 1);
    }
}

/* cppcheck-suppress constParameterCallback ; on_tick 은 daemon_module 콜백 ABI
 * (void(*)(struct shared_ctx*, daemon_state_t)) 라 ctx 를 const 로 못 바꾼다. */
static void mqtt_on_tick(struct shared_ctx *ctx, daemon_state_t state)
{
    if (g_mosq == NULL) {
        return;
    }
    ++g_tick_count;

    if (!g_connected) {
        if ((g_tick_count % RECONNECT_EVERY_TICKS) == 0u) {
            (void)mosquitto_reconnect_async(g_mosq);
            /* ⚠️ 재연결로 소켓 fd 가 바뀔 수 있다. 코어에 fd 재등록 API가 없어
             *   현재는 처리하지 못한다(TODO, 상단 주석 참고). */
        }
    }

    (void)mosquitto_loop_misc(g_mosq);   /* keepalive PINGREQ */

    /* EPOLLOUT 미등록 보완: epoll 이벤트와 무관하게 매 tick 마다 쓰기 대기를 비운다.
     * (코어가 EPOLLIN 만 등록해, 연결 직후처럼 "쓰기 가능"으로만 깨어나는 소켓 상태를
     *  on_event 만으로는 못 잡는다 — 상단 주석 참고.) */
    if (mosquitto_want_write(g_mosq)) {
        (void)mosquitto_loop_write(g_mosq, 1);
    }

    if (g_connected) {
        const int live_fd = mosquitto_socket(g_mosq);
        if ((g_registered_fd >= 0) && (live_fd != g_registered_fd)) {
            (void)fprintf(stderr,
                "[mqtt    ] 경고: 소켓 fd 변경 감지(%d -> %d) — 코어 epoll 재등록 API가 "
                "없어 이 연결의 이벤트를 못 받을 수 있음. 데몬 재시작 권장(TODO)\n",
                g_registered_fd, live_fd);
        }

        if ((g_tick_count % HEARTBEAT_EVERY_TICKS) == 0u) {
            publish_state_daemon(ctx);   /* 계약 §3.3: 최소 5초 heartbeat */
        }
        if ((state == ST_SCANNING) && ((g_tick_count % PROGRESS_EVERY_TICKS) == 0u)) {
            publish_progress(ctx);
        }
        check_and_publish_errors(ctx);
    }
}

/* cppcheck-suppress constParameterCallback ; 위와 동일(콜백 ABI 고정) */
static void mqtt_on_state(struct shared_ctx *ctx, daemon_state_t old_st, daemon_state_t new_st)
{
    (void)old_st;
    if ((g_mosq == NULL) || !g_connected) {
        return;
    }
    publish_state_daemon(ctx);          /* 계약 §3.3: 상태 바뀔 때마다 */
    if (new_st == ST_EXPORT) {
        publish_state_scan(ctx);        /* 계약 §3.4 */
    }
}

static void mqtt_deinit(struct shared_ctx *ctx)
{
    (void)ctx;
    if (g_mosq != NULL) {
        (void)mosquitto_disconnect(g_mosq);
        mosquitto_destroy(g_mosq);
        g_mosq = NULL;
    }
    mosquitto_lib_cleanup();
}

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
