/* ============================================================================
 *  mqtt_module.c  --  RPi Mosquitto 브로커 클라이언트 (제어·상태 통신)  [STUB]
 *  담당: 이현우(코어 통합) + 이광진(브로커 운영·토픽 스키마·TLS 인증서)
 *
 *  브로커는 RPi 에 상주(Mosquitto). Qt·카메라·데몬이 모두 이 브로커의 클라이언트.
 *    수신: scan/start  -> ctx->req 채우고 valid=1  (코어가 FSM 트리거)
 *          scan/stop   -> ctx->req_scan_stop = 1
 *    발행: scan/status -> ctx->progress (진행률)
 *          scan/done   -> ctx->result   (파일 경로·점 수)
 *
 *  ★ epoll 통합 (단일 스레드 유지):
 *      get_fd()  -> mosquitto_socket(mosq)  를 코어 epoll 에 등록
 *      on_event()-> mosquitto_loop_read/write()
 *      on_tick() -> mosquitto_loop_misc()   (keepalive)
 *    ⚠️ mosquitto_loop_start() (자체 스레드) 금지 — 콜백이 다른 스레드에서 불려
 *       shared_ctx/FSM 에 락이 필요해지고 무락 설계가 깨진다.
 *    ⚠️ 재연결 시 소켓 fd 가 바뀌므로 코어에 재등록을 요청해야 한다(TODO).
 *
 *  ★ OpenSSL 고정 요건은 MQTT-over-TLS(8883)로 충족한다(별도 TLS 소켓 없음).
 * ==========================================================================*/
#include "daemon_module.h"
#include <stdio.h>

/* TODO(이광진 협의): 토픽 스키마 확정 후 상수화
 *   #define TOPIC_SCAN_START  "adts/scan/start"
 *   #define TOPIC_SCAN_STOP   "adts/scan/stop"
 *   #define TOPIC_SCAN_STATUS "adts/scan/status"
 *   #define TOPIC_SCAN_DONE   "adts/scan/done"          */

static int mqtt_init(struct shared_ctx *ctx)
{
    (void)ctx;
    /* TODO: mosquitto_lib_init / mosquitto_new / tls_set(8883)
     *       / connect(localhost) / subscribe(scan/start, scan/stop)
     *       콜백에서 payload 파싱 -> ctx->req 채우고 valid=1 */
    (void)fprintf(stderr,
        "[mqtt    ] init (STUB — 브로커 연결·구독 미구현. 이광진 협업)\n");
    return 0;
}

static int mqtt_get_fd(void)
{
    return -1;   /* TODO: return mosquitto_socket(mosq); (연결 수립 후) */
}

/* cppcheck-suppress constParameterCallback ; on_event 는 daemon_module 콜백 ABI
 * (void(*)(struct shared_ctx*)) 라 ctx 를 const 로 못 바꾼다. */
static void mqtt_on_event(struct shared_ctx *ctx)
{
    (void)ctx;
    /* TODO: mosquitto_loop_read(mosq,1); 쓰기 대기 있으면 loop_write */
}

/* cppcheck-suppress constParameterCallback ; 위와 동일(콜백 ABI 고정) */
static void mqtt_on_tick(struct shared_ctx *ctx, daemon_state_t state)
{
    (void)ctx;
    (void)state;
    /* TODO: mosquitto_loop_misc(mosq);            // keepalive
     *       SCANNING 중이면 ctx->progress 를 scan/status 로 발행(1초 간격 등) */
}

/* cppcheck-suppress constParameterCallback ; 위와 동일(콜백 ABI 고정) */
static void mqtt_on_state(struct shared_ctx *ctx,
                          daemon_state_t old_st, daemon_state_t new_st)
{
    (void)ctx;
    (void)old_st;
    (void)new_st;
    /* TODO: 상태 전이 발행.
     *   -> ST_EXPORT  : ctx->result (경로·점수) 를 scan/done 으로 발행
     *   -> ST_DISARM  : 에러 상태를 scan/status 로 발행
     *   대용량 포인트클라우드는 payload 로 싣지 않는다(파일공유 + 경로 알림만). */
}

static void mqtt_deinit(struct shared_ctx *ctx)
{
    (void)ctx;
    /* TODO: mosquitto_disconnect / destroy / lib_cleanup */
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
