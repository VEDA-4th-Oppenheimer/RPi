/* ============================================================================
 *  protocol.h  --  RPi <-> STM32 UART 통신 계약 (공용 헤더)
 * ----------------------------------------------------------------------------
 *  이 파일은 세 곳에서 "그대로 동일하게" 사용된다:
 *    1) RPi 커널 드라이버 (/dev/turret)   - 프레임 조립/파싱
 *    2) STM32 펌웨어                        - 프레임 파싱/조립
 *    3) RPi 유저 데몬                       - ioctl 호출
 *
 *  ★ 이 헤더는 단일 진실 소스(single source of truth)다.
 *    수정 시 반드시 3자 모두 재빌드하고, VERSION을 올린다.
 *
 *  담당: 이현우 (RPi↔STM32 프로토콜 관리)
 *  버전 규칙: 프레임/CMD/구조체가 바뀌면 PROTO_VERSION +1
 * ==========================================================================*/
#ifndef PROTOCOL_H
#define PROTOCOL_H

/* 커널/펌웨어/유저 어디서 include 되어도 고정폭 정수를 얻도록 분기 */
#ifdef __KERNEL__
  #include <linux/types.h>      /* u8, s16 ... (커널) */
  typedef __u8  proto_u8;
  typedef __u16 proto_u16;
  typedef __s16 proto_s16;
  typedef __u32 proto_u32;
#else
  #include <stdint.h>           /* uint8_t ... (유저/STM) */
  typedef uint8_t  proto_u8;
  typedef uint16_t proto_u16;
  typedef int16_t  proto_s16;
  typedef uint32_t proto_u32;
#endif

/* ---------------------------------------------------------------------------
 *  0. 버전
 * ------------------------------------------------------------------------- */
#define PROTO_VERSION   3u

/* ---------------------------------------------------------------------------
 *  1. 프레임 구조
 *
 *     +------+------+------+---------------+--------+
 *     | SOF  | CMD  | LEN  | PAYLOAD(0..N) | CRC16  |
 *     +------+------+------+---------------+--------+
 *       1B     1B     1B      LEN B           2B
 *
 *   - SOF   : 프레임 시작 표식. 바이트 스트림에서 프레임 경계를 찾는 기준.
 *   - CMD   : 명령 코드 (enum proto_cmd).
 *   - LEN   : PAYLOAD 바이트 수. ★ 반드시 PROTO_MAX_PAYLOAD 이하인지 검사 (CWE-120).
 *   - CRC16 : SOF~PAYLOAD 까지의 CRC-16/CCITT-FALSE. 리틀엔디언 2바이트.
 *             불일치 시 해당 프레임 폐기.
 * ------------------------------------------------------------------------- */
#define PROTO_SOF            0xAAu   /* 프레임 시작 바이트            */
#define PROTO_MAX_PAYLOAD    16u     /* PAYLOAD 최대 크기 (경계검사)  */
#define PROTO_HEADER_LEN     3u      /* SOF + CMD + LEN               */
#define PROTO_CRC_LEN        2u      /* CRC16                         */
#define PROTO_MAX_FRAME      (PROTO_HEADER_LEN + PROTO_MAX_PAYLOAD + PROTO_CRC_LEN)

/* ---------------------------------------------------------------------------
 *  2. 명령 코드 (CMD)
 *
 *   방향 표기:  R->S = RPi가 STM으로,  S->R = STM이 RPi로.
 *
 *   ※ 이 프로토콜은 STM32 에 붙은 액추에이터(방위 스텝 + 고각 스텝)와
 *     라이다(TOFSense-F2 P) 를 다룬다. 라이다는 STM32 경유이므로 거리(d)가
 *     CMD_DISTANCE 로 이 프로토콜에 포함된다. (레이더/조도는 설계에서 제거됨)
 * ------------------------------------------------------------------------- */
enum proto_cmd {
    /* --- 링크 감시 (heartbeat) --- */
    CMD_PING        = 0x01,   /* R->S : 살아있나? (100ms 주기)            */
    CMD_PONG        = 0x02,   /* S->R : 살아있음                          */

    /* --- 조준 명령 --- */
    CMD_SET_TARGET  = 0x10,   /* R->S : 목표 절대각 (payload: proto_target)*/
    CMD_HOME        = 0x11,   /* R->S : 홈 포지션 복귀 시작 (양축)         */
    CMD_SET_MODE    = 0x12,   /* R->S : 모드 전환   (payload: proto_mode)  */
    CMD_DISARM      = 0x13,   /* R->S : 즉시 안전정지 (스텝모터 2축 disable)*/
    CMD_QUERY_DIST  = 0x14,   /* R->S : 라이다 거리 측정 요청              */

    /* --- 상태 통지 (STM -> RPi) --- */
    CMD_HOMED       = 0x20,   /* S->R : 양축 홈 완료. 이제 SET_TARGET 유효 */
    CMD_ALIGNED     = 0x21,   /* S->R : 목표각 도달(lock-on 가능)          */
    CMD_STATUS      = 0x22,   /* S->R : 현재 상태 보고 (payload: proto_status) */
    CMD_DISTANCE    = 0x23,   /* S->R : 라이다 측정 거리 보고 (payload: proto_distance) */
    CMD_ERROR       = 0x2F,   /* S->R : 오류 통지 (payload: proto_err)     */
};

/* ---------------------------------------------------------------------------
 *  3. 모드 / 에러 코드
 * ------------------------------------------------------------------------- */
enum proto_mode_val {
    MODE_MANUAL = 0,          /* 수동: 외부 SET_TARGET 무시, 정지 유지     */
    MODE_TRACK  = 1,          /* 추적: SET_TARGET 수신 시 즉시 조준        */
};

enum proto_err_code {
    ERR_NONE        = 0,
    ERR_BAD_CRC     = 1,      /* 수신 프레임 CRC 불일치                    */
    ERR_BAD_LEN     = 2,      /* LEN 초과 (CWE-120 방어 발동)              */
    ERR_NOT_HOMED   = 3,      /* 홈 전에 SET_TARGET 수신                   */
    ERR_OUT_OF_RANGE= 4,      /* 목표각 범위 밖                           */
    ERR_STALL       = 5,      /* 스텝모터 탈조 감지                       */
};

/* ---------------------------------------------------------------------------
 *  4. 각도 규약  ★ 중요
 *
 *   - 단위: 0.1도 (deci-degree).  예) 1234 => 123.4도
 *   - theta (방위, 스텝모터): 홈 기준 "절대각". 범위 0 ~ 3599 (0.0~359.9도).
 *       STM 내부에서 (목표절대각 - 현재스텝카운트) 오차만큼만 펄스를 생성한다.
 *       프로토콜은 절대각을 주고받고, 상대 펄스 변환은 STM 펌웨어 책임.
 *   - phi (고각, 스텝모터): 방위/고각 모두 스텝모터 구동(서보 아님).
 *       범위 PHI_MIN ~ PHI_MAX (기구 가동범위). 절대각. theta 와 동일하게
 *       홈 캘리브레이션 + 스텝 카운트로 절대각 추적.
 *
 *   ※ 부팅 후 CMD_HOMED(양축 홈 완료) 를 받기 전의 SET_TARGET 은 STM이
 *     무시하고 CMD_ERROR(ERR_NOT_HOMED) 로 응답한다.
 * ------------------------------------------------------------------------- */
#define ANGLE_SCALE       10        /* 1도 = 10 단위 (0.1도 해상도)        */
#define THETA_MIN         0         /* 방위 최소 (0.0도)                   */
#define THETA_MAX         3599      /* 방위 최대 (359.9도)                 */
#define PHI_MIN           0         /* 고각 최소 (0.0도)  ※ 기구에 맞게 조정 */
#define PHI_MAX           900       /* 고각 최대 (90.0도) ※ 기구에 맞게 조정 */

/* ---------------------------------------------------------------------------
 *  5. PAYLOAD 구조체  (모두 고정 크기 + __packed)
 *
 *   ★ 패딩이 끼면 RPi/STM 간 바이트 정렬이 어긋난다. 반드시 packed.
 *   ★ 멀티바이트 필드는 리틀엔디언으로 직렬화한다(양측 합의).
 * ------------------------------------------------------------------------- */
#if defined(_MSC_VER)
  #define PROTO_PACKED_BEGIN __pragma(pack(push,1))
  #define PROTO_PACKED_END   __pragma(pack(pop))
  #define PROTO_PACKED
#else
  #define PROTO_PACKED_BEGIN
  #define PROTO_PACKED_END
  #define PROTO_PACKED __attribute__((packed))
#endif

PROTO_PACKED_BEGIN

/* CMD_SET_TARGET payload : 목표 절대각 */
struct proto_target {
    proto_s16 theta_ddeg;   /* 방위 절대각 (0.1도), 0..3599  */
    proto_s16 phi_ddeg;     /* 고각 절대각 (0.1도)           */
} PROTO_PACKED;

/* CMD_SET_MODE payload */
struct proto_mode {
    proto_u8 mode;          /* enum proto_mode_val           */
} PROTO_PACKED;

/* CMD_STATUS payload : STM -> RPi 현재 상태 */
struct proto_status {
    proto_s16 cur_theta_ddeg;  /* 현재 방위 절대각 (스텝 카운트 기반) */
    proto_s16 cur_phi_ddeg;    /* 현재 고각 (스텝 카운트 기반)       */
    proto_u8  flags;           /* bit0=homed, bit1=aligned, bit2=moving */
} PROTO_PACKED;

/* CMD_ERROR payload */
struct proto_err {
    proto_u8 code;          /* enum proto_err_code           */
} PROTO_PACKED;

/* CMD_DISTANCE payload : 라이다 수신 거리 */
struct proto_distance {
    proto_u16 distance_mm;  /* 라이다 측정 거리 (mm)          */
} PROTO_PACKED;

PROTO_PACKED_END


/* status flags 비트 정의 */
#define STF_HOMED    (1u << 0)
#define STF_ALIGNED  (1u << 1)
#define STF_MOVING   (1u << 2)

/* ---------------------------------------------------------------------------
 *  6. ioctl 인터페이스  (유저 데몬 <-> /dev/turret 커널 드라이버)
 *
 *   커널 헤더가 있을 때만 정의한다(유저/커널 공용).
 *   STM 펌웨어 빌드에는 포함되지 않는다.
 * ------------------------------------------------------------------------- */
#if defined(__KERNEL__) || defined(PROTO_WANT_IOCTL)
  #ifdef __KERNEL__
    #include <linux/ioctl.h>
  #else
    #include <sys/ioctl.h>
  #endif

  #define TURRET_IOC_MAGIC   'T'

  /* 목표각 설정: 유저가 proto_target 을 넘김 → 드라이버가 프레임화해 전송 */
  #define TURRET_SET_TARGET  _IOW(TURRET_IOC_MAGIC, 1, struct proto_target)
  /* 홈 시작 (양축) */
  #define TURRET_HOME        _IO (TURRET_IOC_MAGIC, 2)
  /* 모드 전환 */
  #define TURRET_SET_MODE    _IOW(TURRET_IOC_MAGIC, 3, struct proto_mode)
  /* 즉시 안전정지 */
  #define TURRET_DISARM      _IO (TURRET_IOC_MAGIC, 4)
  /* 링크/상태 조회: 아래 turret_link_state 를 반환 */
  #define TURRET_GET_STATE   _IOR(TURRET_IOC_MAGIC, 5, struct turret_link_state)
  /* 라이다 거리 조회: 드라이버가 STM32에 CMD_QUERY_DIST 송신 후 CMD_DISTANCE 대기 및 반환 */
  #define TURRET_GET_DISTANCE _IOR(TURRET_IOC_MAGIC, 6, struct proto_distance)

  /* 드라이버가 유저에게 노출하는 종합 상태
   * (link_alive 는 heartbeat 로 커널이 판단, 나머지는 STM STATUS 캐시) */
  struct turret_link_state {
      proto_u8  link_alive;      /* 1=heartbeat 정상, 0=link_dead     */
      proto_u8  flags;           /* STM proto_status.flags 최신값      */
      proto_s16 cur_theta_ddeg;  /* 최근 보고된 현재 방위각            */
      proto_s16 cur_phi_ddeg;    /* 최근 보고된 현재 고각              */
      proto_u8  last_err;        /* 최근 CMD_ERROR code               */
  };

  /* poll() 이벤트 의미(참고):
   *   POLLIN  : ALIGNED/HOMED/STATUS/DISTANCE 등 STM 통지 도착
   *   POLLERR : link_dead (heartbeat 끊김)  → 유저는 GET_STATE 로 확인
   */
#endif /* ioctl */

/* ---------------------------------------------------------------------------
 *  7. heartbeat / 타이밍 상수 (양측 공유)
 * ------------------------------------------------------------------------- */
#define HB_PING_PERIOD_MS   100u   /* RPi가 PING 보내는 주기            */
#define HB_TIMEOUT_MS       300u   /* 이 시간 무응답이면 link_dead      */
#define HB_MISS_LIMIT       3u     /* PONG 연속 누락 허용치             */

/* ---------------------------------------------------------------------------
 *  8. CRC-16/CCITT-FALSE  (poly=0x1021, init=0xFFFF)
 *
 *   양측이 "완전히 동일한" 구현을 써야 하므로 헤더에 inline 로 박아둔다.
 *   SOF~PAYLOAD 전체(헤더 3B + payload LEN B)에 대해 계산.
 *   ※ 이 CRC 는 turret 프로토콜 전용. 라이다 NLink 는 별도 합산 체크섬(§11-5).
 * ------------------------------------------------------------------------- */
static inline proto_u16 proto_crc16(const proto_u8 *data, proto_u16 len)
{
    proto_u16 crc = 0xFFFFu;
    proto_u16 i;
    proto_u8  b;
    for (i = 0u; i < len; i++) {
        crc ^= (proto_u16)((proto_u16)data[i] << 8);
        for (b = 0u; b < 8u; b++) {
            if ((crc & 0x8000u) != 0u) {
                crc = (proto_u16)((crc << 1) ^ 0x1021u);
            } else {
                crc = (proto_u16)(crc << 1);
            }
        }
    }
    return crc;
}

#endif /* PROTOCOL_H */
