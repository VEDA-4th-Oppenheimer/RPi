/* ============================================================================
 *  daemon_module.h  --  통합 데몬 "코어 <-> 모듈" 계약 (공용 헤더)  [v3 스캐너]
 * ----------------------------------------------------------------------------
 *  통합 데몬은 "코어 + 모듈" 플러그인 구조로 공동 구축한다.
 *    - 코어 (이현우): epoll 이벤트 루프 + 상태머신 + 모듈 등록/호출 + 공유 컨텍스트
 *                     + /dev/turret ioctl·read 전담 + heartbeat(100ms PING/300ms link_dead)
 *                     + (pan,tilt,d) -> (x,y,z) 스트리밍 변환 + 포인트클라우드 파일화
 *    - 모듈 (각 담당): 아래 daemon_module 인터페이스를 구현해 코어에 등록
 *
 *  ★ 모듈은 서로를 직접 호출하지 않는다. 모든 데이터 교환은
 *    코어가 들고 있는 shared_ctx 를 통해서만 이루어진다.
 *
 *  ★ v3 (2026-07-22 주제 전환): 안티드론 조준 -> 1D LiDAR 스캐너·자동 캘리브레이션.
 *    - FSM: IDLE->TRACK->LOCK_ON  =>  IDLE->SCANNING->EXPORT->IDLE
 *    - 칼만/비주얼서보잉/조준(core_aim) 제거. 정적 스캔이라 부적합.
 *    - tls_module -> mqtt_module (MQTT-over-TLS 로 OpenSSL 요건 동시 충족)
 *    - 캘리브 연산은 카메라 단(앱+OpenCV). RPi 는 스캔 수집·변환·파일화까지.
 *
 *  모듈 담당 (v3):
 *    - mqtt_module : 이현우 코어 + 이광진 협업 (RPi Mosquitto 브로커 클라이언트.
 *                    scan/start·stop 수신 -> FSM 트리거, scan/status·done 발행)
 *    - imu_module  : 송영빈 (/dev/imu ICM-20948 read -> roll/pitch 제공.
 *                    수평 게이트 "판정"은 코어가 한다)
 *    - led_module  : 이현우 (/dev/led = 상태 LED x3 + 액티브 부저 x1)
 *
 *  ※ /dev/turret 은 모듈이 아니라 "코어가 직접" 다룬다(protocol.h):
 *    ioctl 로 HOME/SCAN_START/STOP/DISARM, read()+poll() 로 스캔 점 스트림 수신.
 *  ※ 단일 스레드 epoll 이므로 락 없음. 콜백에서 블로킹 금지.
 *
 *  담당: 이현우 (데몬 코어 + 이 계약 관리)
 * ==========================================================================*/
#ifndef DAEMON_MODULE_H
#define DAEMON_MODULE_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>   /* notice_post 의 strncpy */

/* v3 -> v4 (2026-08-13)
 *   · scan_result 에 json_path 추가 — 카메라 업로드가 .pcd 경로에서 확장자를
 *     조작해 추론하지 않도록. 규칙이 두 곳에 흩어지면 파일명 규칙을 바꿀 때
 *     한쪽만 고쳐진다.
 *   · daemon_notice 신설 — 코어/모듈이 "자기가 판단한 것"을 Qt 에 알리는 통로.
 *     STM 이 올린 오류는 link.last_err 로 흐르는데, 코어 자신의 판정(수평 게이트
 *     거부·홈 타임아웃·업로드 실패)은 전달 경로가 없어 로그로만 끝났다. */
#define DAEMON_MODULE_VERSION   4u

/* ---------------------------------------------------------------------------
 *  1. 시스템 상태머신 (v3 스캐너)
 *
 *     IDLE ──(MQTT scan/start + 수평게이트 OK)──> SCANNING ──(SCAN_DONE)──> EXPORT
 *       ^                                            │                        │
 *       │                                    (스캔 점 스트림                  │
 *       │                                     -> 변환 -> 파일 append)         │
 *       └──────────────────(파일 마감·전달 완료)───────────────────────────────┘
 *
 *     DISARM <── 링크단절 / STM 에러(CMD_ERROR) / 사용자 정지 ── (어느 상태에서든)
 *     DISARM ──(cmd/rearm, 링크가 살아있을 때만)──> IDLE
 *
 *   - 코어가 상태를 소유하고 전이시킨다.
 *   - 모듈은 on_tick(state) 로 현재 상태를 통보받고, core_request_state() 로
 *     전이를 요청한다(코어가 유효성 검사).
 * ------------------------------------------------------------------------- */
typedef enum {
    ST_IDLE = 0,   /* 대기: MQTT scan/start 수신 대기. heartbeat 만 유지       */
    ST_SCANNING,   /* 스캔 중: (pan,tilt,d) 스트림 -> (x,y,z) 변환 -> 파일     */
    ST_EXPORT,     /* 내보내기: 포인트클라우드 파일 마감 + 카메라 단 전달      */
    ST_DISARM,     /* 안전정지: 링크단절/에러/사용자 정지 (모터 disable)       */
} daemon_state_t;

/* ---------------------------------------------------------------------------
 *  2. 공유 컨텍스트 (shared_ctx)
 *
 *   코어가 소유하는 단일 구조체. 모듈은 이 포인터를 통해서만 데이터를
 *   읽고 쓴다. 쓰기 주체를 주석으로 못박아 경합을 예방한다.
 * ------------------------------------------------------------------------- */

/* 스캔 요청 파라미터. mqtt_module 이 scan/start 페이로드를 파싱해 채운다.
 * 코어가 protocol.h 의 struct proto_scan_start 로 변환해 ioctl 로 내린다.
 * 단위는 protocol.h 와 동일하게 0.1도(deci-degree). */
struct scan_request {
    int16_t  pan_start_ddeg;    /* 팬 시작각   (PAN_MIN..PAN_MAX)             */
    int16_t  pan_end_ddeg;      /* 팬 끝각                                    */
    int16_t  tilt_start_ddeg;   /* 틸트 시작각 (TILT_MIN..TILT_MAX, 부호)     */
    int16_t  tilt_end_ddeg;     /* 틸트 끝각                                  */
    uint16_t step_ddeg;         /* 격자 간격 (10 = 1.0도, 빔 FOV 상 1도 권장) */
    int32_t  sensor_height_mm;  /* 지면→라이다 높이 (아래 설명)               */
    uint8_t  valid;             /* 1=요청 있음 (코어가 소비 후 0 으로 클리어) */
};

/* sensor_height_mm — 지면에서 라이다 회전축까지의 높이. 설치 시 실측해 넣는다.
 *
 *   ⚠️ 이 값은 좌표 계산에 **들어가지 않는다.** 산출물의 frame 은 lidar_scan,
 *     즉 원점이 라이다 자신이므로 tilt=0 인 점의 y 는 0 이어야 한다. 높이를
 *     좌표에 반영하면 이름과 내용이 어긋난다(2026-07-29 실제로 발생한 버그 —
 *     모든 y 가 -1.2m 로 찍혔고 그건 사실상 actuator_base 좌표였다).
 *
 *   용도는 메타데이터다. 산출물 헤더에 scan.sensor_height_m 으로 실려 나가고,
 *   소비자(카메라 단)가 바닥평면을 잡거나 다른 좌표계로 옮길 때 쓴다.
 *   0 이면 "모름"으로 간주된다. */

/* 스캔 진행 상황. 코어가 씀, mqtt_module 이 읽어 scan/status 로 발행 */
struct scan_progress {
    uint32_t points;            /* 지금까지 수신·변환한 점 수                 */
    uint32_t expected;          /* 예상 총 점 수 (범위/격자로 코어가 산출)    */
    uint8_t  percent;           /* 0..100 (expected==0 이면 0)                */
};

/* 스캔 결과 요약. 코어가 EXPORT 진입 시 씀, mqtt_module 이 scan/done 발행 */
struct scan_result {
    char     path[256];         /* 생성된 포인트클라우드 파일 경로 (.pcd)     */
    /* 원시 측정 JSON 경로. 카메라 단이 받는 것은 **이쪽**이다(.pcd 아님).
     * 코어가 채운다 — 소비자가 .pcd 경로에서 확장자를 갈아끼워 추론하면
     * 파일명 규칙이 두 곳에 살게 되어 한쪽만 바뀔 때 조용히 깨진다. */
    char     json_path[256];
    uint32_t point_count;       /* 파일에 기록된 점 수                        */
    uint32_t stm_reported;      /* STM 이 CMD_SCAN_DONE 으로 보고한 점 수     */
    uint8_t  valid;             /* 1=파일 준비 완료                           */
};

/* 코어/모듈이 스스로 판단한 사건을 Qt 에 알리는 통로 (v4 신설).
 *
 * ★ 왜 필요한가: STM 이 올린 오류는 link.last_err 로 흐르지만, **데몬 자신의
 *   판정**은 갈 곳이 없었다. 수평 게이트가 스캔을 거부해도, 홈이 20초 무응답으로
 *   취소돼도, 카메라 업로드가 실패해도 로그에만 남아서 Qt 는 "명령을 보냈는데
 *   아무 일도 안 일어난" 것으로 보였다.
 *
 * 사용법: 쓰는 쪽이 code/name/msg 를 채우고 seq 를 **증가**시킨다. mqtt_module 이
 * seq 변화를 감지해 adts/event/error 로 발행한다. 같은 사건이 반복될 때도
 * 구분되도록 값이 아니라 seq 로 에지를 잡는다. */
struct daemon_notice {
    uint32_t seq;               /* 증가 = 새 통지. 0 = 아직 없음              */
    uint16_t code;              /* 100+ = 데몬 자체 판정 (STM 코드와 겹침 회피) */
    uint8_t  fatal;             /* 1 = 작업 중단, 사용자 개입 필요             */
    char     name[32];          /* 예: "ERR_NOT_LEVEL"                        */
    char     msg[128];
};

/* 데몬 자체 판정 코드 (protocol.h 의 proto_err_code 와 겹치지 않게 100 부터) */
/* ★ 100 부터 시작하는 이유 = Qt 가 출처를 코드 하나로 가릴 수 있게:
 *      code <  100  STM32 가 CMD_ERROR 로 올린 것 (protocol.h proto_err_code)
 *      code >= 100  데몬이 스스로 판단한 것 (여기)
 *   ⚠️ 이 규칙을 깨고 데몬이 1~6 을 빌려 쓰면 안 된다. 실제로 페이로드 파싱
 *     오류가 4(ERR_OUT_OF_RANGE)를 빌려 쓰고 있었는데, Qt 는 그게 STM32 의
 *     "스캔 범위 밖" 인지 데몬의 "필드 누락" 인지 구분할 수 없었다. */
#define NOTICE_DISARM        100u   /* 안전정지 (링크 두절 / STM 오류 / 사용자) */
#define NOTICE_HOME_TIMEOUT  101u   /* 홈 무응답으로 요청 취소                  */
#define NOTICE_NOT_LEVEL     102u   /* 수평 게이트가 스캔을 거부                */
#define NOTICE_UPLOAD_FAIL   103u   /* 카메라 업로드 실패 (파일은 로컬에 남음)  */
#define NOTICE_BAD_REQUEST   104u   /* cmd 페이로드 필드 누락/형식 오류         */

/* 거치 수평 상태. imu_module 이 중력벡터에서 산출해 씀, 코어가 게이트 판정.
 * ※ 방식 A(게이트): 임계값 초과면 SCANNING 진입 거부. 좌표 보정 아님.
 *   roll/pitch 원본은 향후 보정(방식 B) 확장 대비로 로그에 보존한다. */
struct level_state {
    float    roll_deg;          /* 좌우 기울기                                */
    float    pitch_deg;         /* 앞뒤 기울기                                */
    uint8_t  valid;             /* 1=최근 측정값 유효 (IMU 없으면 0)          */
};

/* STM32 링크/축 상태. 코어가 /dev/turret GET_STATE 로 캐시. 모듈은 읽기만 */
struct link_status {
    int16_t  cur_pan_ddeg;      /* 현재 팬 각  (스텝카운트)                   */
    int16_t  cur_tilt_ddeg;     /* 현재 틸트 각(엔코더, 부호)                 */
    uint8_t  homed;             /* 1=홈 완료 (STF_HOMED)                      */
    uint8_t  scanning;          /* 1=STM 이 스캔 중 (STF_SCANNING)            */
    uint8_t  link_alive;        /* 1=heartbeat 정상                           */
    uint8_t  last_err;          /* 최근 CMD_ERROR code (enum proto_err_code)  */
};

struct shared_ctx {
    daemon_state_t       state;     /* 코어 소유. 모듈은 읽기만               */
    struct scan_request  req;       /* mqtt_module ->  (코어가 소비)          */
    struct scan_progress progress;  /* 코어 ->  (mqtt 읽음)                   */
    struct scan_result   result;    /* 코어 ->  (mqtt 읽음)                   */
    struct level_state   level;     /* imu_module ->  (코어가 게이트 판정)    */
    struct link_status   link;      /* 코어 ->  (모듈 읽기)                   */

    /* 모듈 -> 코어 이벤트 요청 플래그 (코어가 매 tick 소비 후 클리어) */
    uint8_t  req_scan_stop;         /* mqtt: scan/stop 수신                   */
    uint8_t  req_disarm;            /* 임의 모듈: 안전정지 요청               */
    /* DISARM 해제 요청. 코어가 복구 가능 여부를 판정한다(링크가 죽어 있으면
     * 거부) — 모듈은 "사용자가 눌렀다" 만 전달하고 판단은 하지 않는다.
     * 같은 tick 에 req_disarm 과 함께 서면 안전정지가 이긴다. */
    uint8_t  req_rearm;             /* mqtt: cmd/rearm 수신                   */
    /* 스캔 없이 홈만 세우는 요청. 코어는 스캔 직전에 어차피 홈을 다시 잡으므로
     * 필수는 아니지만, 설치·정비 때 축을 홈 자세로 보내 확인하는 용도다.
     * IDLE 에서만 받는다. 진행 상황은 link.homed 가 0 -> 1 로 알린다. */
    uint8_t  req_home;              /* mqtt: cmd/home 수신                    */

    /* 코어/모듈 -> mqtt (event/error 발행). 위 daemon_notice 주석 참조.
     * 쓰는 쪽은 notice_post() 를 쓴다 — seq 증가를 빠뜨리면 조용히 안 나간다. */
    struct daemon_notice notice;

    void    *core;                  /* 코어 핸들 (core_* API 호출용)          */
};

/* 수평 게이트 임계값 (deg). 이 값을 넘으면 코어가 SCANNING 진입을 거부한다.
 *
 * ⚠️ **지금 이 값은 게이트를 사실상 무력화한 브링업 설정이다 (2026-08-13).**
 *
 *   IMU 가 킷 축에 대해 약 2.7도 비뚤게 붙어 있어 실제 기울기를 그만큼 작게
 *   보고한다. 실측 대조:
 *       스캔 바닥평면으로 잰 실제 기울기   3.6도  (6회 반복, 편차 0.05도)
 *       IMU 가 보고한 값                  1.1도
 *   그래서 임계를 6.0 으로 둔 것은 "3.6도 기울어진 킷을 통과시키기 위해서"이지
 *   6도까지 허용해도 된다는 뜻이 아니다. 지금 이 게이트는 아무것도 막지 않는다.
 *
 * ⚠️ **캘리브 데이터를 뽑기 전에 반드시 되돌릴 것.** 기울어진 채 찍으면 산출물의
 *   바닥이 통째로 3.6도 돌아간 상태로 카메라 단에 넘어간다.
 *
 *   되돌리는 순서:
 *     ① 킷 거치 자세 교정 — IMU 말고 **바닥평면 기울기**를 보고 맞춘다.
 *        내리막 방위 232도 쪽이 낮다(받침 지름 200mm 기준 심 약 12.6mm).
 *     ② IMU 장착 오프셋 상수를 imu_module 에 넣어 level 값을 실제와 맞춘다.
 *     ③ 이 값을 3.0f 이하로 되돌린다.
 *   ①~③ 전까지 이 게이트의 통과는 "수평이 확인됐다"는 뜻이 아니다. */
#define LEVEL_GATE_MAX_DEG   6.0f

/* ---------------------------------------------------------------------------
 *  3. 모듈 인터페이스
 *
 *   각 모듈은 이 구조체를 채운 인스턴스를 하나 노출한다.
 *   콜백은 모두 "코어의 단일 epoll 스레드"에서 호출된다(재진입 없음).
 *
 *   ★ 콜백에서 블로킹 금지: 단일 스레드라 한 모듈이 멈추면 전체가 멈춘다.
 *     (예: libmosquitto 는 loop_start() 스레드 방식이 아니라 소켓 fd 를
 *      get_fd() 로 넘겨 코어 epoll 에 태운다.)
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
     *   < 0 : fd 없는 모듈(순수 계산형). on_event 미호출
     * ※ MQTT 재연결 시 소켓 fd 가 바뀌므로 코어는 필요 시 재등록한다. */
    int  (*get_fd)(void);

    /* get_fd() 가 준 fd 에 I/O 이벤트가 왔을 때 호출.
     * 모듈은 여기서 fd 를 읽고 shared_ctx 를 갱신한다. */
    void (*on_event)(struct shared_ctx *ctx);

    /* 매 루프 주기 tick(기본 100ms) 1회 호출. 현재 상태를 인자로 받는다.
     * fd 없는 모듈(led 점멸/비프 패턴 등)은 여기서 일한다. */
    void (*on_tick)(struct shared_ctx *ctx, daemon_state_t state);

    /* 상태 전이 알림(선택). 코어가 상태를 바꾼 직후 호출. NULL 이면 무시.
     * (예: led=색 변경/비프, mqtt=상태 발행) */
    void (*on_state)(struct shared_ctx *ctx,
                     daemon_state_t old_st, daemon_state_t new_st);

    /* 종료 정리(선택). 디바이스 close 등. NULL 가능. */
    void (*deinit)(struct shared_ctx *ctx);
};

/* ---------------------------------------------------------------------------
 *  4. 모듈 등록 (각 모듈 .c 파일이 노출하는 심볼)
 *
 *   구현 예)  const struct daemon_module *mqtt_module_get(void);
 *   코어는 시작 시 아래 목록을 순회하며 등록한다.
 * ------------------------------------------------------------------------- */
#ifdef __cplusplus
extern "C" {
#endif

const struct daemon_module *mqtt_module_get(void);   /* 이현우 + 이광진 협업 */
const struct daemon_module *imu_module_get(void);    /* 송영빈               */
const struct daemon_module *led_module_get(void);    /* 이현우               */
const struct daemon_module *camera_module_get(void); /* 이현우 + 이영민      */

/* ---------------------------------------------------------------------------
 *  5. 코어 API  (모듈이 코어에 요청할 때 호출)
 *
 *   모듈은 ctx->core 를 첫 인자로 넘겨 호출한다.
 *   구현은 코어(이현우)가 제공. 모듈은 선언만 사용.
 * ------------------------------------------------------------------------- */

/* 상태 전이 요청 (코어가 유효성 검사 후 전이).
 * 0=전이됨, <0=거부(잘못된 전이 / 수평 게이트 미통과 등). */
int core_request_state(void *core, daemon_state_t want);

/* 구조화 로그 (감사/이벤트 로그). 코어가 stderr(추후 파일+syslog) 로 남김. */
void core_log(void *core, const char *event, const char *fmt, ...);

/* 블로킹 작업(수십 초짜리 파일 업로드 등)을 마친 뒤 heartbeat 기준선을 지금으로
 * 되맞춘다.
 *
 * ★ 없으면: 데몬은 자기 CLOCK_MONOTONIC 으로 "300ms 넘게 PONG 없음 = link_dead"
 *   를 판정한다. 루프가 업로드 때문에 30초 멈췄다 깨어나면 그 간격을 통신 두절로
 *   오판해 **매번 DISARM** 이 걸린다. 링크는 멀쩡한데 성공한 스캔마다 Qt 에
 *   빨간 오류가 뜬다. "내가 바빴던 것이지 링크가 죽은 게 아니다" 를 명시한다.
 *
 * ⚠️ 진짜 두절을 가릴 수 있으므로, **자기가 오래 블로킹했다는 것을 아는 쪽**만
 *   부를 것. 주기 코드에서 습관적으로 부르면 heartbeat 가 무력해진다. */
void core_hb_reprime(void *core);

/* 통지 게시. seq 를 증가시켜 mqtt_module 이 에지로 잡게 한다.
 * 헤더 인라인인 이유 = 코어와 모듈 양쪽에서 쓰는데 링크 대상이 갈리지 않게. */
static inline void notice_post(struct shared_ctx *ctx, uint16_t code,
                               uint8_t fatal, const char *name, const char *msg)
{
    if (ctx == NULL) {
        return;
    }
    ctx->notice.code  = code;
    ctx->notice.fatal = fatal;
    /* ⚠️ strncpy 는 소스가 길면 NUL 을 안 붙인다. 마지막 바이트를 직접
     *   0 으로 눌러 잘림이 항상 종료되게 한다. */
    strncpy(ctx->notice.name, name, sizeof(ctx->notice.name) - 1u);
    ctx->notice.name[sizeof(ctx->notice.name) - 1u] = '\0';
    strncpy(ctx->notice.msg, msg, sizeof(ctx->notice.msg) - 1u);
    ctx->notice.msg[sizeof(ctx->notice.msg) - 1u] = '\0';
    ctx->notice.seq++;              /* ★ 마지막에 — 위 필드가 다 채워진 뒤 */
}

#ifdef __cplusplus
}  /* extern "C" */
#endif

/* ---------------------------------------------------------------------------
 *  6. 상태 문자열 (로그/디버그 공용)
 * ------------------------------------------------------------------------- */
static inline const char *daemon_state_str(daemon_state_t s)
{
    switch (s) {
    case ST_IDLE:     return "IDLE";
    case ST_SCANNING: return "SCANNING";
    case ST_EXPORT:   return "EXPORT";
    case ST_DISARM:   return "DISARM";
    default:          return "?";
    }
}

#endif /* DAEMON_MODULE_H */
