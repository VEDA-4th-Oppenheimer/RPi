/* ============================================================================
 *  daemon_module.h  --  통합 데몬 "코어 <-> 모듈" 계약 (공용 헤더)  [v2]
 * ----------------------------------------------------------------------------
 *  통합 데몬은 "코어 + 모듈" 플러그인 구조로 공동 구축한다.
 *    - 코어 (이현우): epoll 이벤트 루프 + 상태머신 + 모듈 등록/호출 + 공유 컨텍스트
 *                     + /dev/turret ioctl 전담 + heartbeat(100ms PING/300ms link_dead)
 *    - 모듈 (각 담당): 아래 daemon_module 인터페이스를 구현해 코어에 등록
 *
 *  ★ 모듈은 서로를 직접 호출하지 않는다. 모든 데이터 교환은
 *    코어가 들고 있는 shared_ctx 를 통해서만 이루어진다.
 *    → 파일 단위 소유가 분리되어 머지 충돌이 없고 결합도가 낮다.
 *
 *  모듈 담당 (v2):
 *    - tracking_module : 이현우(칼만 예측/기하변환) + 송영빈(비주얼 서보잉)
 *    - meta_module     : 이영민  (카메라 AI 메타데이터 → 픽셀오차 dx,dy + 채널)
 *    - tls_module      : 팀장    (융합 (x,y,z) → Qt 관제 TLS push)
 *
 *  ※ v2 아키텍처: 레이더(radar)·조도(illum) 센서 제거됨. 카메라 4채널이 상시
 *    감시하다 드론 탐지 시 해당 채널 방위로 조준·추적 시작.
 *  ※ 라이다는 STM32 경유. 거리는 코어가 /dev/turret CMD_DISTANCE 로 받아
 *    shared_ctx.range 에 채운다(모듈이 /dev/lidar 직접 read 하지 않음).
 *  ※ /dev/turret 은 모듈이 아니라 "코어가 직접" ioctl 로 다룬다(protocol.h).
 *
 *  담당: 이현우 (데몬 코어 + 이 계약 관리)
 * ==========================================================================*/
#ifndef DAEMON_MODULE_H
#define DAEMON_MODULE_H

#include <stdint.h>
#include <stddef.h>

#define DAEMON_MODULE_VERSION   2u

/* ---------------------------------------------------------------------------
 *  1. 시스템 상태머신 (v2)
 *
 *     IDLE ──카메라 탐지(채널)──> TRACK ──정렬 완료──> LOCK_ON
 *       ^                           │                    │
 *       │                       표적 상실           (측거+융합 유지)
 *       │                           v                    │
 *     DISARM <──링크단절/에러/사용자 정지────────────────┘
 *
 *   - 코어가 상태를 소유하고 전이시킨다.
 *   - 모듈은 on_tick(state) 로 "현재 상태"를 통보받고, 필요 시 shared_ctx 에
 *     이벤트(req_disarm)를 쓰거나 core_request_state() 로 전이를 요청한다.
 * ------------------------------------------------------------------------- */
typedef enum {
    ST_IDLE = 0,   /* 광역 감시: 카메라 4채널 상시 탐지 대기            */
    ST_TRACK,      /* 표적 포착: 해당 채널 방위 조준 + 비주얼 서보잉    */
    ST_LOCK_ON,    /* 정렬 완료: 라이다 측거(STM32 경유) + 좌표 융합    */
    ST_DISARM,     /* 안전정지: 링크단절/에러/사용자 정지 (모터 disable)*/
} daemon_state_t;

/* ---------------------------------------------------------------------------
 *  2. 공유 컨텍스트 (shared_ctx)
 *
 *   코어가 소유하는 단일 구조체. 모듈은 이 포인터를 통해서만 데이터를
 *   읽고 쓴다. (모듈 간 직접 호출 금지)
 *
 *   쓰기 주체를 주석으로 못박아 경합을 예방한다. 데몬은 단일 스레드
 *   epoll 루프이므로 락은 불필요하나, "누가 쓰는 필드인지"는 계약이다.
 * ------------------------------------------------------------------------- */

/* 표적 조대 방위. 코어가 카메라 채널→방위 매핑으로 채움 (탐지 채널 기준) */
struct target_bearing {
    int16_t  theta_ddeg;    /* 조대 방위 절대각 (0.1도)                     */
    uint8_t  valid;         /* 1=유효                                       */
};

/* 카메라 픽셀 오차 (화면중심 대비). meta_module 이 씀 */
struct pixel_error {
    int16_t  dx;            /* +오른쪽                                      */
    int16_t  dy;            /* +아래                                        */
    uint8_t  valid;         /* 1=이번 프레임 표적 있음                      */
    uint8_t  channel;       /* 4채널 중 표적이 잡힌 채널 (0..3)             */
};

/* 라이다 거리. 코어가 /dev/turret CMD_DISTANCE 로 받아 씀. tracking 이 읽음 */
struct range_meas {
    uint16_t d_mm;          /* 거리 (mm)                                    */
    uint8_t  valid;         /* 1=유효 측거                                  */
};

/* 최종 융합 결과 (직교좌표). tracking_module 이 씀, tls_module 이 읽음 */
struct target_xyz {
    float x, y, z;          /* m 단위 (좌표계 원점/축은 코어가 정의)        */
    uint8_t valid;
};

/* 현재 조준각 (코어가 /dev/turret STATUS 에서 캐시). 여러 모듈이 읽음 */
struct aim_state {
    int16_t cur_theta_ddeg; /* 현재 방위 (스텝)                             */
    int16_t cur_phi_ddeg;   /* 현재 고각 (스텝)                             */
    uint8_t homed;          /* 1=홈 완료                                    */
    uint8_t aligned;        /* 1=정렬 완료(lock-on 가능)                    */
    uint8_t link_alive;     /* 1=STM 링크 정상 (heartbeat)                  */
};

struct shared_ctx {
    daemon_state_t         state;      /* 코어 소유. 모듈은 읽기만          */
    struct target_bearing  bearing;    /* 코어(카메라 채널→방위) ->        */
    struct pixel_error     pixerr;     /* meta_module ->                   */
    struct range_meas      range;      /* 코어(/dev/turret CMD_DISTANCE) -> */
    struct target_xyz      fused;      /* tracking_module ->               */
    struct aim_state       aim;        /* 코어 -> (모듈 읽기)              */

    /* 모듈 -> 코어 이벤트 요청 플래그 (코어가 매 tick 소비 후 클리어) */
    uint8_t  req_disarm;    /* 임의 모듈: 정지 요청                         */

    void    *core;          /* 코어 핸들 (core_* API 호출용, 아래 참조)     */
};

/* ---------------------------------------------------------------------------
 *  3. 모듈 인터페이스
 *
 *   각 모듈은 이 구조체를 채운 인스턴스를 하나 노출한다.
 *   콜백은 모두 "코어의 단일 epoll 스레드"에서 호출된다(재진입 없음).
 *
 *   반환값 규약: 0=성공, <0=에러(-errno 권장).
 * ------------------------------------------------------------------------- */
struct daemon_module {
    const char *name;               /* 로그/디버그용 이름                   */

    /* 1회 초기화. 디바이스 open, 리소스 확보. ctx 저장 가능.
     * 실패(<0) 시 코어는 데몬을 중단한다. */
    int  (*init)(struct shared_ctx *ctx);

    /* epoll 에 등록할 파일 디스크립터.
     *   >=0 : 코어가 epoll 에 등록, 이벤트 시 on_event 호출
     *   < 0 : fd 없는 모듈(순수 계산형). on_event 미호출          */
    int  (*get_fd)(void);

    /* get_fd() 가 준 fd 에 I/O 이벤트가 왔을 때 호출.
     * 모듈은 여기서 fd 를 읽고 shared_ctx 를 갱신한다. */
    void (*on_event)(struct shared_ctx *ctx);

    /* 매 루프 주기 tick(기본 100ms) 1회 호출. 현재 상태를 인자로 받는다.
     * fd 없는 계산 모듈(tracking 등)은 여기서 일한다. */
    void (*on_tick)(struct shared_ctx *ctx, daemon_state_t state);

    /* 상태 전이 알림(선택). 코어가 상태를 바꾼 직후 호출.
     * NULL 이면 무시. lock-on 진입 시 측거 트리거 등에 사용. */
    void (*on_state)(struct shared_ctx *ctx,
                     daemon_state_t old_st, daemon_state_t new_st);

    /* 종료 정리(선택). 디바이스 close 등. NULL 가능. */
    void (*deinit)(struct shared_ctx *ctx);
};

/* ---------------------------------------------------------------------------
 *  4. 모듈 등록 (각 모듈 .c/.cpp 파일이 노출하는 심볼)
 *
 *   구현 예)  const struct daemon_module *meta_module_get(void);
 *   코어는 시작 시 아래 목록을 순회하며 register 한다.
 * ------------------------------------------------------------------------- */
#ifdef __cplusplus
extern "C" {
#endif

const struct daemon_module *tracking_module_get(void);   /* 이현우+송영빈 */
const struct daemon_module *meta_module_get(void);       /* 이영민       */
const struct daemon_module *tls_module_get(void);        /* 팀장         */

/* ---------------------------------------------------------------------------
 *  5. 코어 API  (모듈이 코어에 요청할 때 호출)
 *
 *   모듈은 ctx->core 를 첫 인자로 넘겨 호출한다.
 *   구현은 코어(이현우)가 제공. 모듈은 선언만 사용.
 * ------------------------------------------------------------------------- */

/* /dev/turret 로 목표각 전송을 코어에 요청 (코어가 ioctl 수행).
 * tracking_module 이 비주얼서보잉/예측 결과 각도를 넘길 때 사용. */
int core_aim(void *core, int16_t theta_ddeg, int16_t phi_ddeg);

/* 상태 전이 요청 (코어가 유효성 검사 후 전이). */
int core_request_state(void *core, daemon_state_t want);

/* 구조화 로그 (감사/이벤트 로그). 코어가 stderr(추후 파일+syslog) 로 남김. */
void core_log(void *core, const char *event, const char *fmt, ...);

#ifdef __cplusplus
}  /* extern "C" */
#endif

/* ---------------------------------------------------------------------------
 *  6. 상태 문자열 (로그/디버그 공용)
 * ------------------------------------------------------------------------- */
static inline const char *daemon_state_str(daemon_state_t s)
{
    switch (s) {
    case ST_IDLE:    return "IDLE";
    case ST_TRACK:   return "TRACK";
    case ST_LOCK_ON: return "LOCK_ON";
    case ST_DISARM:  return "DISARM";
    default:         return "?";
    }
}

#endif /* DAEMON_MODULE_H */
