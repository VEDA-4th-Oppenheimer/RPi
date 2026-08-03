/* ============================================================================
 *  mqtt_module.c  --  RPi Mosquitto 브로커 클라이언트 (제어·상태 통신)
 *  담당: 이현우(코어 통합) + 이광진(브로커 운영·토픽 스키마·TLS 인증서)
 *
 *  브로커는 RPi 에 상주(Mosquitto). Qt·카메라·데몬이 모두 이 브로커의 클라이언트.
 *    수신: scan/start  -> ctx->req 채우고 valid=1  (코어가 FSM 트리거)
 *          scan/stop   -> ctx->req_scan_stop = 1
 *    발행: scan/status -> ctx->progress (진행률, SCANNING 중 ~1초 간격)
 *          scan/done   -> ctx->result   (파일 경로·점 수), ST_EXPORT 진입 시 1회
 *
 *  ★ epoll 통합 (단일 스레드 유지):
 *      get_fd()  -> mosquitto_socket(mosq)  를 코어 epoll 에 등록
 *      on_event()-> mosquitto_loop_read/write()
 *      on_tick() -> mosquitto_loop_misc()   (keepalive)
 *    ⚠️ mosquitto_loop_start() (자체 스레드) 금지 — 콜백이 다른 스레드에서 불려
 *       shared_ctx/FSM 에 락이 필요해지고 무락 설계가 깨진다.
 *    ⚠️ 재연결 시 소켓 fd 가 바뀔 수 있는데, 코어에 fd 재등록 API가 아직 없어
 *       지금은 처리하지 못한다 (TODO — core_* API 확장 필요, 이현우 협의).
 *
 *  ★ MQTT-over-TLS(8883)는 인증서 발급 전까지 보류. 지금은 평문 1883으로
 *    localhost 브로커에 붙는다 — mosquitto_tls_set() 호출부만 미리 마련해 둔다.
 *
 *  ★ 토픽 스키마: "Device 파트 아키텍처 및 역할 분담 V2" 문서 기준 scan 토픽 4개.
 *    calib/result·calib/objects·imu/level 등 카메라 단 발행 토픽은 아직 팀
 *    협의 중이라 이 모듈에서는 다루지 않는다(수신 전용 상대가 없음).
 * ==========================================================================*/
#include "daemon_module.h"

#include <mosquitto.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BROKER_HOST      "localhost"
#define BROKER_PORT      1883
#define BROKER_KEEPALIVE 30
#define MQTT_QOS         1

#define TOPIC_SCAN_START  "scan/start"
#define TOPIC_SCAN_STOP   "scan/stop"
#define TOPIC_SCAN_STATUS "scan/status"
#define TOPIC_SCAN_DONE   "scan/done"

#define STATUS_PUBLISH_EVERY_TICKS  10u   /* on_tick 100ms 기준 * 10 = 약 1초 간격 */
#define PAYLOAD_COPY_MAX            511u  /* CWE-120: 고정 버퍼 + 명시적 절단 */

static struct mosquitto *g_mosq = NULL;
static bool     g_connected = false;
static unsigned g_tick_count = 0u;

/* ---------------------------------------------------------------------------
 *  scan/start payload 파싱 — 외부 JSON 라이브러리 없이(이 저장소 관례) 필요한
 *  키만 뽑아낸다. "key":value 형태를 가정하며 키 순서는 무관하다.
 *    {"pan_start_ddeg":0,"pan_end_ddeg":1800,"tilt_start_ddeg":0,
 *     "tilt_end_ddeg":0,"step_ddeg":10,"z_offset_mm":0}
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
        return false;                 /* 숫자를 못 읽음 */
    }
    *out = v;
    return true;
}

static void parse_scan_start(const char *json, struct scan_request *req)
{
    long v = 0;
    memset(req, 0, sizeof(*req));
    req->step_ddeg = 10u;             /* 기본값(1.0도) — 파싱 실패 필드도 안전값 유지 */

    if (json_find_long(json, "pan_start_ddeg", &v))     req->pan_start_ddeg  = (int16_t)v;
    if (json_find_long(json, "pan_end_ddeg", &v))       req->pan_end_ddeg    = (int16_t)v;
    if (json_find_long(json, "tilt_start_ddeg", &v))    req->tilt_start_ddeg = (int16_t)v;
    if (json_find_long(json, "tilt_end_ddeg", &v))      req->tilt_end_ddeg   = (int16_t)v;
    if (json_find_long(json, "step_ddeg", &v) && v > 0) req->step_ddeg       = (uint16_t)v;
    if (json_find_long(json, "z_offset_mm", &v))        req->z_offset_mm     = (int32_t)v;

    req->valid = 1u;
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
    (void)mosquitto_subscribe(mosq, NULL, TOPIC_SCAN_START, MQTT_QOS);
    (void)mosquitto_subscribe(mosq, NULL, TOPIC_SCAN_STOP, MQTT_QOS);
    (void)fprintf(stderr, "[mqtt    ] 브로커 연결됨 (%s:%d), scan/start·scan/stop 구독\n",
                  BROKER_HOST, BROKER_PORT);
}

static void on_disconnect(struct mosquitto *mosq, void *obj, int rc)
{
    (void)mosq;
    (void)obj;
    g_connected = false;
    (void)fprintf(stderr, "[mqtt    ] 브로커 연결 끊김 (rc=%d) — 다음 tick 에 재연결 시도\n", rc);
}

/* ctx 는 mqtt_init() 에서 mosquitto_new() 의 userdata 로 넘긴 shared_ctx* 다. */
static void on_message(struct mosquitto *mosq, void *obj, const struct mosquitto_message *msg)
{
    (void)mosq;
    struct shared_ctx *ctx = (struct shared_ctx *)obj;
    if ((msg->payload == NULL) || (msg->payloadlen <= 0)) {
        return;
    }

    if (strcmp(msg->topic, TOPIC_SCAN_STOP) == 0) {
        ctx->req_scan_stop = 1u;
        (void)fprintf(stderr, "[mqtt    ] scan/stop 수신\n");
        return;
    }
    if (strcmp(msg->topic, TOPIC_SCAN_START) != 0) {
        return;
    }

    /* CWE-120: payload 는 NUL 종단이 보장되지 않는다 — 고정 버퍼 + 명시적 절단. */
    char buf[PAYLOAD_COPY_MAX + 1u];
    const size_t len = ((size_t)msg->payloadlen < PAYLOAD_COPY_MAX)
                            ? (size_t)msg->payloadlen : PAYLOAD_COPY_MAX;
    memcpy(buf, msg->payload, len);
    buf[len] = '\0';

    parse_scan_start(buf, &ctx->req);
    (void)fprintf(stderr, "[mqtt    ] scan/start 수신 — pan[%d..%d] tilt[%d..%d] step=%u\n",
                  ctx->req.pan_start_ddeg, ctx->req.pan_end_ddeg,
                  ctx->req.tilt_start_ddeg, ctx->req.tilt_end_ddeg, ctx->req.step_ddeg);
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

    mosquitto_connect_callback_set(g_mosq, on_connect);
    mosquitto_disconnect_callback_set(g_mosq, on_disconnect);
    mosquitto_message_callback_set(g_mosq, on_message);

    /* TODO(이광진): 인증서 준비되면 MQTT-over-TLS(8883) 로 전환.
     *   mosquitto_tls_set(g_mosq, ca_path, NULL, cert_path, key_path, NULL); */

    if (mosquitto_connect_async(g_mosq, BROKER_HOST, BROKER_PORT, BROKER_KEEPALIVE)
        != MOSQ_ERR_SUCCESS) {
        (void)fprintf(stderr,
            "[mqtt    ] connect_async 실패 — degraded 로 계속(재시도는 on_tick)\n");
        /* 브로커가 아직 안 떠 있을 수 있으므로 실패해도 데몬 전체를 죽이지 않는다. */
    }

    (void)fprintf(stderr, "[mqtt    ] init 완료 (%s:%d, TLS 비활성 — 인증서 대기)\n",
                  BROKER_HOST, BROKER_PORT);
    return 0;
}

static int mqtt_get_fd(void)
{
    if (g_mosq == NULL) {
        return -1;
    }
    return mosquitto_socket(g_mosq);
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

static void publish_scan_status(const struct shared_ctx *ctx)
{
    char payload[160];
    (void)snprintf(payload, sizeof(payload),
        "{\"state\":\"%s\",\"percent\":%u,\"points\":%u,\"expected\":%u}",
        daemon_state_str(ctx->state), (unsigned)ctx->progress.percent,
        (unsigned)ctx->progress.points, (unsigned)ctx->progress.expected);
    (void)mosquitto_publish(g_mosq, NULL, TOPIC_SCAN_STATUS,
                             (int)strlen(payload), payload, MQTT_QOS, false);
}

/* cppcheck-suppress constParameterCallback ; 위와 동일(콜백 ABI 고정) */
static void mqtt_on_tick(struct shared_ctx *ctx, daemon_state_t state)
{
    if (g_mosq == NULL) {
        return;
    }

    if (!g_connected) {
        /* 매 tick 재시도하면 브로커/네트워크를 압박하므로 ~1초 간격으로만. */
        if ((g_tick_count % STATUS_PUBLISH_EVERY_TICKS) == 0u) {
            (void)mosquitto_reconnect_async(g_mosq);
            /* ⚠️ 재연결로 소켓 fd 가 바뀔 수 있다. 코어에 fd 재등록 API가 없어
             *   현재는 처리하지 못한다(TODO, 상단 주석 참고). */
        }
    }

    (void)mosquitto_loop_misc(g_mosq);   /* keepalive PINGREQ */

    if ((state == ST_SCANNING) && g_connected
        && ((g_tick_count % STATUS_PUBLISH_EVERY_TICKS) == 0u)) {
        publish_scan_status(ctx);
    }
    ++g_tick_count;
}

/* cppcheck-suppress constParameterCallback ; 위와 동일(콜백 ABI 고정) */
static void mqtt_on_state(struct shared_ctx *ctx, daemon_state_t old_st, daemon_state_t new_st)
{
    (void)old_st;
    if ((g_mosq == NULL) || !g_connected) {
        return;
    }

    if (new_st == ST_EXPORT) {
        char payload[352];
        (void)snprintf(payload, sizeof(payload),
            "{\"path\":\"%s\",\"point_count\":%u,\"stm_reported\":%u}",
            ctx->result.path, (unsigned)ctx->result.point_count,
            (unsigned)ctx->result.stm_reported);
        (void)mosquitto_publish(g_mosq, NULL, TOPIC_SCAN_DONE,
                                 (int)strlen(payload), payload, MQTT_QOS, false);
    } else if (new_st == ST_DISARM) {
        char payload[96];
        (void)snprintf(payload, sizeof(payload),
            "{\"state\":\"error\",\"last_err\":%u}", (unsigned)ctx->link.last_err);
        (void)mosquitto_publish(g_mosq, NULL, TOPIC_SCAN_STATUS,
                                 (int)strlen(payload), payload, MQTT_QOS, false);
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
