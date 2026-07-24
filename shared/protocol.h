/* ============================================================================
 *  protocol.h  --  RPi <-> STM32 UART 통신 계약 (공용 헤더)  [v4 스캐너]
 * ----------------------------------------------------------------------------
 *  이 파일은 세 곳에서 "그대로 동일하게" 사용된다:
 *    1) RPi 커널 드라이버 (/dev/turret)   - 프레임 조립/파싱
 *    2) STM32 펌웨어                        - 프레임 파싱/조립
 *    3) RPi 유저 데몬                       - ioctl 호출 + read() 스캔 스트림
 *
 *  ★ 단일 진실 소스. 수정 시 3자 모두 재빌드하고 VERSION을 올린다.
 *  ★ v4: 안티드론 조준(SET_TARGET/ALIGNED/MODE/DISTANCE) 제거,
 *        스캔 스트림(SCAN_START/STOP/DATA/DONE) 추가. phi 부호각 확장.
 *
 *  담당: 이현우 (RPi↔STM32 프로토콜 관리)
 * ==========================================================================*/
#ifndef PROTOCOL_H
#define PROTOCOL_H

#ifdef __KERNEL__
  #include <linux/types.h>
  typedef __u8  proto_u8;
  typedef __u16 proto_u16;
  typedef __s16 proto_s16;
  typedef __u32 proto_u32;
#else
  #include <stdint.h>
  typedef uint8_t  proto_u8;
  typedef uint16_t proto_u16;
  typedef int16_t  proto_s16;
  typedef uint32_t proto_u32;
#endif

/* 0. 버전 */
#define PROTO_VERSION   4u

/* 1. 프레임 구조 상수 */
#define PROTO_SOF            0xAAu
#define PROTO_MAX_PAYLOAD    16u
#define PROTO_HEADER_LEN     3u
#define PROTO_CRC_LEN        2u
#define PROTO_MAX_FRAME      (PROTO_HEADER_LEN + PROTO_MAX_PAYLOAD + PROTO_CRC_LEN)

/* 2. 명령 코드 (R->S = RPi가 STM으로, S->R = STM이 RPi로) */
enum proto_cmd {
    /* 링크 감시 */
    CMD_PING        = 0x01,   /* R->S : 살아있나? (100ms)                 */
    CMD_PONG        = 0x02,   /* S->R : 살아있음                          */
    /* 제어 (R->S) */
    CMD_HOME        = 0x10,   /* R->S : 홈 캘리브 (팬=리밋 / 틸트=엔코더) */
    CMD_SCAN_START  = 0x11,   /* R->S : 스캔 시작 (payload: proto_scan_start) */
    CMD_SCAN_STOP   = 0x12,   /* R->S : 스캔 중단                         */
    CMD_DISARM      = 0x13,   /* R->S : 즉시 안전정지 (스텝 2축 disable)  */
    /* 통지 (S->R) */
    CMD_HOMED       = 0x20,   /* S->R : 양축 홈 완료                      */
    CMD_STATUS      = 0x21,   /* S->R : 현재 상태 (payload: proto_status) */
    CMD_SCAN_DATA   = 0x22,   /* S->R : 스캔 점 스트림 (proto_scan_point) */
    CMD_SCAN_DONE   = 0x23,   /* S->R : 스캔 완료 (proto_scan_done)       */
    CMD_ERROR       = 0x2F,   /* S->R : 오류 통지 (payload: proto_err)    */
};

/* 3. 에러 코드 */
enum proto_err_code {
    ERR_NONE         = 0,
    ERR_BAD_CRC      = 1,      /* 수신 프레임 CRC 불일치        */
    ERR_BAD_LEN      = 2,      /* LEN 초과 (CWE-120)            */
    ERR_NOT_HOMED    = 3,      /* 홈 전에 SCAN_START 수신       */
    ERR_OUT_OF_RANGE = 4,      /* 스캔 범위 밖                  */
    ERR_STALL        = 5,      /* 팬 스텝모터 탈조 감지         */
    ERR_LIDAR        = 6,      /* 라이다 무응답/무효            */
};

/* 4. 각도 규약
 *   - 단위: 0.1도 (deci-degree). 예) 1234 => 123.4도
 *   - theta (팬 방위): 절대각 0 ~ 3599. 홈=리밋스위치+스텝카운트.
 *   - phi   (틸트 고각): 절대각 -900 ~ +900 (부호). 홈/각도=MT6701 엔코더.
 *   - 홈(CMD_HOMED) 전 SCAN_START 은 STM이 무시하고 ERR_NOT_HOMED 응답.
 *   - (θ,φ,d) -> (x,y,z) 변환은 RPi 데몬. 원점=천장 팬/틸트 축 교점. */
#define ANGLE_SCALE       10
#define THETA_MIN         0
#define THETA_MAX         3599
#define PHI_MIN           (-900)   /* -90.0도 (v3의 0 에서 확장) */
#define PHI_MAX           900      /* +90.0도                    */

/* 5. PAYLOAD 구조체 (모두 __packed + 리틀엔디언)
 *  - struct padding 을 생성하지 않기 위해 사용
 *  -  packed 을 사용하지 않을시
 *      uint8_t  a;    // 1바이트
 *      uint16_t b;    // 2바이트
 *      2바이트 경계에 맞추려고 a 뒤에 1바이트를 넣음
 *      PROTO_PAKED_BEGIN , END 사이에 구조체 생성후 PROTO_PACKED 매크로 삽입
 */
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

/* CMD_SCAN_START payload : 스캔 범위·격자 (10B) */
struct proto_scan_start {
    proto_s16 theta_start_ddeg;  /* 팬 시작각 (0.1도)            */
    proto_s16 theta_end_ddeg;    /* 팬 끝각                      */
    proto_s16 phi_start_ddeg;    /* 틸트 시작각 (0.1도, 부호)    */
    proto_s16 phi_end_ddeg;      /* 틸트 끝각                    */
    proto_u16 step_ddeg;         /* 격자 간격 (0.1도, 10=1.0도)  */
} PROTO_PACKED;

/* CMD_SCAN_DATA payload : 스캔 점 하나 (6B) */
struct proto_scan_point {
    proto_s16 theta_ddeg;   /* 방위 (0.1도, 스텝카운트 래치)     */
    proto_s16 phi_ddeg;     /* 고각 (0.1도, 엔코더 래치, 부호)   */
    proto_u16 d_mm;         /* 거리 (mm)                         */
} PROTO_PACKED;

/* CMD_SCAN_DONE payload : 스캔 완료 요약 (4B) */
struct proto_scan_done {
    proto_u32 point_count;  /* 상행한 총 점 수 (데몬이 유실 검증)*/
} PROTO_PACKED;

/* CMD_STATUS payload : STM -> RPi 현재 상태 (5B) */
struct proto_status {
    proto_s16 cur_theta_ddeg;  /* 현재 방위 (스텝카운트)         */
    proto_s16 cur_phi_ddeg;    /* 현재 고각 (엔코더, 부호)       */
    proto_u8  flags;           /* bit0=homed, bit1=scanning      */
} PROTO_PACKED;

/* CMD_ERROR payload (1B) */
struct proto_err {
    proto_u8 code;          /* enum proto_err_code               */
} PROTO_PACKED;

PROTO_PACKED_END

/* status flags 비트 정의 */
#define STF_HOMED     (1u << 0)
#define STF_SCANNING  (1u << 1)

/* 6. ioctl + read() 인터페이스 (유저 데몬 <-> /dev/turret)
 *    커널/유저 공용. STM 펌웨어 빌드에는 미포함.
 *    스캔 점(CMD_SCAN_DATA)은 ioctl 이 아니라 read() 로 스트리밍한다:
 *      드라이버가 kfifo 에 누적 -> 유저가 read() 로 proto_scan_point 배치 수신,
 *      poll() POLLIN 으로 대기. */
#if defined(__KERNEL__) || defined(PROTO_WANT_IOCTL)
  #ifdef __KERNEL__
    #include <linux/ioctl.h>
  #else
    #include <sys/ioctl.h>
  #endif

  #define TURRET_IOC_MAGIC   'T'

  #define TURRET_HOME        _IO (TURRET_IOC_MAGIC, 1)
  #define TURRET_SCAN_START  _IOW(TURRET_IOC_MAGIC, 2, struct proto_scan_start)
  #define TURRET_SCAN_STOP   _IO (TURRET_IOC_MAGIC, 3)
  #define TURRET_DISARM      _IO (TURRET_IOC_MAGIC, 4)
  #define TURRET_GET_STATE   _IOR(TURRET_IOC_MAGIC, 5, struct turret_link_state)
  /* heartbeat PING 1회 송신(fire-and-forget). 주기·타임아웃 판정은 데몬 소유,
   * PONG 도착은 GET_STATE.pong_seq 증가로 감지. */
  #define TURRET_PING        _IO (TURRET_IOC_MAGIC, 6)

  struct turret_link_state {
      proto_u8  link_alive;      /* 1=heartbeat 정상, 0=link_dead     */
      proto_u8  flags;           /* STM proto_status.flags 최신값      */
      proto_s16 cur_theta_ddeg;  /* 최근 보고된 현재 방위각            */
      proto_s16 cur_phi_ddeg;    /* 최근 보고된 현재 고각 (부호)       */
      proto_u8  last_err;        /* 최근 CMD_ERROR code               */
      proto_u32 pong_seq;        /* PONG 누적 카운터 (heartbeat 감지) */
  };

  /* poll(): POLLIN = 스캔 점/통지 도착, POLLERR = link_dead */
#endif /* ioctl */

/* 7. heartbeat / 타이밍 상수 */
#define HB_PING_PERIOD_MS   100u
#define HB_TIMEOUT_MS       300u
#define HB_MISS_LIMIT       3u

/* 8. CRC-16/CCITT-FALSE (poly=0x1021, init=0xFFFF)
 *    SOF~PAYLOAD 전체에 대해 계산. 3자가 동일 함수 공유. */
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
