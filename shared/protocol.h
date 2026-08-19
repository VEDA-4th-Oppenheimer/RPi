/* ============================================================================
 *  protocol.h  --  RPi <-> STM32 UART 통신 계약 (공용 헤더)  [v6 스캐너]
 * ----------------------------------------------------------------------------
 *  이 파일은 세 곳에서 "그대로 동일하게" 사용된다:
 *    1) RPi 커널 드라이버 (/dev/turret)   - 프레임 조립/파싱
 *    2) STM32 펌웨어                        - 프레임 파싱/조립
 *    3) RPi 유저 데몬                       - ioctl 호출 + read() 스캔 스트림
 *
 *  핵심: 단일 진실 소스. 수정 시 3자 모두 재빌드하고 VERSION을 올린다.
 *  핵심: v4: 안티드론 조준(SET_TARGET/ALIGNED/MODE/DISTANCE) 제거,
 *        스캔 스트림(SCAN_START/STOP/DATA/DONE) 추가. tilt 부호각 확장.
 *  핵심: v5: ① 스캔 점에 라이다 원시 품질 필드 추가 (6B -> 18B)
 *        ② CMD_HOMED 에 엔코더 원본값 payload 추가 (2축 HOME 확립용)
 *  핵심: v6: ① 오류에 축(axis) 필드 — 어느 축이 문제인지 프로토콜이 나른다
 *        ② ERR_BUSY / ERR_ENCODER 신설
 *        ③ proto_status 확장 + STM 이 실제로 주기 송신 (진단 카운터)
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

/* 0. 버전
 *   v5 (2026-07-30)
 *     ① proto_scan_point 확장 (6B -> 18B) — 라이다 원시 품질 필드 보존.
 *        TOFSense-F2P 16바이트 프레임에 이미 실려 오지만 버리고 있던 값들
 *        (device_time / dis_status / signal_strength / range_precision)과
 *        STM32 래치 시각을 상행한다. JSON 인터페이스 계약의 필수·권고 필드를
 *        채우기 위함. 프레임 11B -> 23B (115200/100Hz 에서 UART 9.5% -> 20%).
 *     ② proto_homed 신설 — CMD_HOMED 가 엔코더 원본값을 함께 올린다.
 *        MT6701 은 절대 엔코더라 HOME 은 "읽기 1회"로 끝나며, 그 원본값이
 *        산출물 헤더의 provenance 가 된다. */
/* v6 (2026-08-19)
 *   왜 한 번에 묶었나 — 프로토콜 개정은 마스터 sha 변경 -> 4개 사본 동기 ->
 *   push 순서(rpi main 먼저 -> STM32) -> 드라이버·데몬·Qt 반영까지 비용이
 *   고정으로 든다. 나눠 하면 그 비용을 그만큼 반복해서 낸다.
 *
 *     ① proto_err 에 axis 추가 (1B -> 2B)
 *        축을 나르는 필드가 없어서, 홈 실패 시 어느 축인지 알려고 **다른
 *        오류코드를 빌려 쓰는** 브링업 장치(SCAN_HOME_AXIS_PROBE)를 두고
 *        있었다. 빌린 코드는 그 코드가 진짜로 났을 때 오독을 부른다.
 *        이제 코드는 "무엇이" 를, axis 는 "어디서" 를 말한다.
 *     ② ERR_BUSY / ERR_ENCODER 신설 — 아래 enum 주석 참조.
 *     ③ proto_status 5B -> 15B 로 확장하고 **STM 이 1초 주기로 실제 송신**한다.
 *        지금까지 이 구조체는 정의만 있고 한 번도 보내진 적이 없었다. 그래서
 *        드라이버의 STF_HOMED 는 한 번 서면 갱신될 일이 없었고, STM 을
 *        리셋/재플래시하면 STM 은 홈이 풀렸는데 드라이버 캐시만 참으로 남았다.
 *        진단 카운터(TX 실패/RX 오버플로/엔코더 재시도/라이다 drop/거절)도
 *        여기 실린다 — 지금까지 그 값들은 계측만 되고 읽을 방법이 없었다.
 *
 *   PROTO_VERSION 자체는 여전히 와이어로 보내지 않는다. 4개 사본은 CI 의
 *   drift-check 가 보장하므로 남는 위험은 "보드에 옛 펌웨어가 그대로" 인
 *   경우뿐인데, 그건 버전 바이트보다 **드라이버의 payload 길이 불일치 경고**가
 *   더 넓게(버전을 안 올리고 구조체만 바꾼 경우까지) 잡는다. */
#define PROTO_VERSION   6u

/* 1. 프레임 구조 상수 */
#define PROTO_SOF            0xAAu
/* v5 에서 proto_scan_point 가 18B 가 되어 16 -> 24 로 확장(여유 포함).
 * 주의: 이 값을 줄이면 스캔 점 프레임이 CWE-120 경계검사에서 조용히 버려진다. */
#define PROTO_MAX_PAYLOAD    24u
#define PROTO_HEADER_LEN     3u
#define PROTO_CRC_LEN        2u
#define PROTO_MAX_FRAME      (PROTO_HEADER_LEN + PROTO_MAX_PAYLOAD + PROTO_CRC_LEN)

/* 2. 명령 코드 (R->S = RPi가 STM으로, S->R = STM이 RPi로) */
enum proto_cmd {
    /* 링크 감시 */
    CMD_PING        = 0x01,   /* R->S : 살아있나? (100ms)                 */
    CMD_PONG        = 0x02,   /* S->R : 살아있음                          */
    /* 제어 (R->S) */
    CMD_HOME        = 0x10,   /* R->S : 홈 확립 (양축 MT6701 절대 엔코더) */
    CMD_SCAN_START  = 0x11,   /* R->S : 스캔 시작 (payload: proto_scan_start) */
    CMD_SCAN_STOP   = 0x12,   /* R->S : 스캔 중단                         */
    CMD_DISARM      = 0x13,   /* R->S : 즉시 안전정지 (스텝 2축 disable)  */
    /* 통지 (S->R) */
    CMD_HOMED       = 0x20,   /* S->R : 홈 완료 (payload: proto_homed)    */
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
    ERR_STALL        = 5,      /* 탈조 감지 (엔코더 대조 불일치)*/
    ERR_LIDAR        = 6,      /* 라이다 무응답/무효            */
    /* v6 신설 */
    ERR_BUSY         = 7,      /* 지금 상태에서 받을 수 없는 요청 */
    ERR_ENCODER      = 8,      /* 엔코더 판독 실패 (I2C)          */
};

/* 오류가 난 축. proto_err.axis 에 실린다.
 *
 * 핵심: 코드는 "무엇이", axis 는 "어디서" 를 말한다. 이 필드가 없어서 홈 실패
 *   시 어느 축인지 알려고 다른 오류코드를 빌려 쓰는 브링업 장치를 두고 있었다
 *   (SCAN_HOME_AXIS_PROBE: 4=팬 / 6=틸트). 빌린 코드는 그 코드가 진짜로 났을
 *   때 오독을 부르므로 v6 에서 정식 필드로 올린다. */
/* 핵심: 비트 플래그다 — PAN=bit0, TILT=bit1, BOTH=둘 다.
 *   축별 판정을 OR 로 합칠 수 있게 일부러 이렇게 잡았다:
 *       axis = (ok_pan ? 0 : ERR_AXIS_PAN) | (ok_tilt ? 0 : ERR_AXIS_TILT)
 *   값을 바꾸면 그 관용구가 조용히 깨진다. */
enum proto_err_axis {
    ERR_AXIS_NONE = 0,         /* 축과 무관한 오류 (CRC/LEN 등)  */
    ERR_AXIS_PAN  = 1u << 0,
    ERR_AXIS_TILT = 1u << 1,
    ERR_AXIS_BOTH = (1u << 0) | (1u << 1),
};

/* 4. 각도 규약
 *   - 단위: 0.1도 (deci-degree). 예) 1234 => 123.4도
 *   - pan  (팬 방위): 0 ~ 3599.   2축 스캔에서는 0~1800(180도)만 사용.
 *   - tilt (틸트 고각): -900 ~ +900 (부호).
 *   - 홈(CMD_HOMED) 전 SCAN_START 은 STM이 무시하고 ERR_NOT_HOMED 응답.
 *
 *   핵심: 이 프로토콜이 나르는 각도는 **기구(mechanism) 각도**다.
 *     천장 마운트 2축 구성에서 축 역할은 다음과 같다:
 *       - 틸트 = 빠른 축. 한 스윕이 -90 -> +90 (한쪽 벽 -> 바닥 -> 반대쪽 벽).
 *       - 팬   = 느린 축. 1도씩 180회. 스윕 동안 정지.
 *
 *      두 축 모두 각도원은 **스텝카운트**다 (라이다 ISR 에서 원자적 래치).
 *       엔코더(MT6701) 읽기를 샘플마다 끼우지 않는 이유: I2C 읽기가 끝난
 *       시각과 라이다 샘플 시각이 달라 오히려 동기 오차가 생긴다.
 *
 *      엔코더는 세 곳에서만 쓴다:
 *         ① 홈 확립 — 절대 엔코더라 구동 없이 읽기 1회로 끝난다.
 *                      리밋스위치는 쓰지 않는다(양축 모두 없음).
 *         ② 틸트 스윕 끝점(±90) 대조 — 방향 전환이 탈조가 나는 자리라
 *                      여기서 스텝카운터를 재영점해 누적을 끊는다.
 *         ③ 팬 줄 시작 시 대조 — 보정은 하지 않고 검증만. 팬은 리밋이
 *                      없어 한 번 미끄러지면 남은 스캔 내내 어긋나는데,
 *                      정지 중이라 읽기 비용이 사실상 0 이라서 감시만 둔다.
 *       ②③ 에서 오차가 크면 ERR_STALL 을 올린다.
 *
 *      영점 상수(PAN_ZERO_RAW / TILT_ZERO_RAW)는 조립 후 1회 실측해
 *       펌웨어에 넣는다. 상수가 틀려도 CMD_HOMED 가 엔코더 raw 를 함께
 *       올리므로 이미 찍은 스캔의 각도를 오프라인에서 재계산할 수 있다.
 *
 *   주의: 계약 좌표계(lidar_scan: +x right, +y down, +z forward)로의 변환은
 *     **RPi 데몬 담당**이다. 틸트 스윕이 nadir 를 지나므로 기구 각도와
 *     계약 각도가 1:1 이 아니다:
 *         스윕 전반부(벽->바닥)   -> 계약 pan = p,      tilt =   0 -> -90
 *         스윕 후반부(바닥->벽)   -> 계약 pan = p+1800, tilt = -90 ->   0
 *     즉 기구 팬은 180도만 돌지만 계약상 방위는 360도가 채워진다.
 *     펌웨어는 기구 각도만 신경 쓰고, 좌표계 해석은 데몬에 맡긴다. */
#define ANGLE_SCALE       10
#define PAN_MIN           0
#define PAN_MAX           3599
#define TILT_MIN          (-900)   /* -90.0도 (v3의 0 에서 확장) */
#define TILT_MAX          900      /* +90.0도                    */

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
    proto_s16 pan_start_ddeg;  /* 팬 시작각 (0.1도)            */
    proto_s16 pan_end_ddeg;    /* 팬 끝각                      */
    proto_s16 tilt_start_ddeg;    /* 틸트 시작각 (0.1도, 부호)    */
    proto_s16 tilt_end_ddeg;      /* 틸트 끝각                    */
    proto_u16 step_ddeg;         /* 격자 간격 (0.1도, 10=1.0도)  */
} PROTO_PACKED;

/* CMD_SCAN_DATA payload : 스캔 점 하나 (18B, v5)
 *
 *  주의: signal_strength / range_precision / dis_status 는 **원본 그대로** 전달한다.
 *    정규화·판정은 상위(데몬/캘리브)에서. 특히 signal_strength 는 calibrated
 *    reflectivity 가 아니므로 재질 판별에 단독 사용 금지.
 *
 *  주의: device_time_ms(라이다 시계) 와 stm_ts_ms(STM32 시계) 는 서로 다른
 *    clock domain 이다. 섞어 쓰지 말 것. 계약 JSON 의 timestamp_ns 는
 *    데몬이 자기 단조시계로 채운다(scan.started/ended 와 동일 시계).
 *
 *  주의: 엔코더 원본값은 여기 없다. 엔코더는 HOME(proto_homed)과 줄 경계
 *    탈조 검증에만 쓰고, 스캔 중에는 I2C 를 읽지 않는다(ISR 블로킹 금지). */
struct proto_scan_point {
    proto_s16 pan_ddeg;          /* 기구 방위 (0.1도)                        */
    proto_s16 tilt_ddeg;         /* 기구 고각 (0.1도, 부호)                  */
    proto_u16 d_mm;              /* 거리 (mm)                                */
    proto_u16 signal_strength;   /* F2P 원본 신호세기 (정규화 안 함)         */
    proto_u32 device_time_ms;    /* F2P system time 원본 (라이다 시계)       */
    proto_u32 stm_ts_ms;         /* 래치 시각 (STM32 HAL tick, ms)           */
    proto_u8  dis_status;        /* F2P 원본 거리상태 (1=valid, 0=invalid)   */
    proto_u8  range_precision;   /* F2P 원본 거리정밀도 (F2P 는 0xFF 미지원) */
} PROTO_PACKED;

/* CMD_HOMED payload : 홈 확립 결과 (8B, v5)
 *
 *  MT6701 은 **절대 엔코더**라 기계식 호밍 시퀀스가 필요 없다. 전원을 꺼도
 *  위치를 알므로 HOME 은 "엔코더 2개 읽기"로 끝난다(~0.3ms).
 *
 *  raw 값을 함께 올리는 이유: *_ddeg 는 조립 시 실측한 영점 상수를 적용한
 *  결과라, 나중에 영점이 틀렸다고 밝혀지면 raw 로부터 재계산할 수 있어야
 *  한다. 재스캔 없이 복구 가능한 유일한 경로다. 데몬은 이 값을 산출물
 *  헤더의 provenance 로 남긴다. */
struct proto_homed {
    proto_u16 pan_encoder_raw;   /* MT6701 14비트 원본 (0~16383)             */
    proto_u16 tilt_encoder_raw;
    proto_s16 pan_ddeg;          /* 영점 적용 후 절대각                      */
    proto_s16 tilt_ddeg;
} PROTO_PACKED;

/* CMD_SCAN_DONE payload : 스캔 완료 요약 (4B) */
struct proto_scan_done {
    proto_u32 point_count;  /* 상행한 총 점 수 (데몬이 유실 검증)*/
} PROTO_PACKED;

/* CMD_STATUS payload : STM -> RPi 현재 상태 + 진단 (15B, v6)
 *
 * 핵심: v6 부터 STM 이 **1초 주기로 실제 송신**한다. v5 까지는 이 구조체가
 *   정의만 있고 한 번도 보내진 적이 없었고, 그래서 두 가지가 망가져 있었다:
 *     ① 드라이버의 STF_HOMED 는 CMD_HOMED 때 서기만 하고 내려갈 일이 없었다.
 *        STM 을 리셋/재플래시하면 STM 은 홈이 풀렸는데 캐시만 참으로 남아,
 *        그 상태로 홈을 건너뛰면 SCAN_START 가 매번 거절됐다(실기 발생).
 *     ② 아래 카운터들이 펌웨어 안에서 증가만 하고 읽을 방법이 없었다.
 *
 * 카운터는 u16 이고 포화(65535)해도 무해하다 — 거기까지 갔으면 이미 큰
 * 문제이고, "값이 크다" 는 것만 알면 진단에 충분하다. u32 로 하면 25B 가 되어
 * PROTO_MAX_PAYLOAD(24) 를 넘는다. */
struct proto_status {
    proto_s16 cur_pan_ddeg;    /* 현재 방위 (스텝카운트)          */
    proto_s16 cur_tilt_ddeg;   /* 현재 고각 (엔코더, 부호)        */
    proto_u8  flags;           /* bit0=homed, bit1=scanning       */
    /* --- v6 진단 카운터 (누적, 부팅 이후) --- */
    proto_u16 tx_fail;         /* UART 송신 실패                  */
    proto_u16 rx_ovf;          /* 수신 링버퍼 오버플로.           */
                               /*   0 이 아니면 메인루프가 오래 막혔다는 뜻 */
    proto_u16 enc_retry;       /* 엔코더 판독 재시도              */
    proto_u16 lidar_drop;      /* 라이다 큐 넘침으로 버린 샘플    */
    proto_u16 reject_busy;     /* 진행 중이라 거절한 SCAN_START   */
} PROTO_PACKED;

/* CMD_ERROR payload (2B, v6) */
struct proto_err {
    proto_u8 code;          /* enum proto_err_code               */
    proto_u8 axis;          /* enum proto_err_axis (v6 신설)     */
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
  /* 주의: v5 와 v6 에서 turret_link_state 가 커져 _IOR 인코딩(크기 필드)이
   *    두 번 바뀌었다. 구버전 유저스페이스가 신버전 드라이버를 때리면
   *    -ENOTTY 로 즉시 실패한다(조용한 구조체 오해석보다 안전).
   *    **드라이버·데몬은 반드시 같이 재빌드할 것.** */
  #define TURRET_GET_STATE   _IOR(TURRET_IOC_MAGIC, 5, struct turret_link_state)
  /* heartbeat PING 1회 송신(fire-and-forget). 주기·타임아웃 판정은 데몬 소유,
   * PONG 도착은 GET_STATE.pong_seq 증가로 감지. */
  #define TURRET_PING        _IO (TURRET_IOC_MAGIC, 6)

  struct turret_link_state {
      proto_u8  link_alive;      /* 1=heartbeat 정상, 0=link_dead     */
      proto_u8  flags;           /* STM proto_status.flags 최신값      */
      proto_s16 cur_pan_ddeg;    /* 최근 보고된 현재 방위각            */
      proto_s16 cur_tilt_ddeg;   /* 최근 보고된 현재 고각 (부호)       */
      proto_u8  last_err;        /* 최근 CMD_ERROR code               */
      proto_u32 pong_seq;        /* PONG 누적 카운터 (heartbeat 감지) */

      /* v5: 최근 CMD_HOMED 결과 캐시. flags 에 STF_HOMED 가 서면 유효.
       * 데몬이 산출물 헤더에 provenance 로 기록한다 — 영점 상수가 나중에
       * 틀렸다고 밝혀져도 raw 카운트로부터 각도를 재계산할 수 있게. */
      proto_u16 home_pan_encoder_raw;   /* MT6701 14비트 (0~16383)      */
      proto_u16 home_tilt_encoder_raw;
      proto_s16 home_pan_ddeg;          /* 영점 적용 후 각도 (0.1도)    */
      proto_s16 home_tilt_ddeg;

      /* v6: 최근 CMD_ERROR 의 축. last_err 와 짝이다.
       * 이 필드가 생기기 전에는 어느 축인지 알 방법이 없어, 브링업 중에
       * 다른 오류코드를 빌려 축을 표시했다(SCAN_HOME_AXIS_PROBE). */
      proto_u8  last_err_axis;          /* enum proto_err_axis          */

      /* v6: 최근 CMD_STATUS 의 진단 카운터. STM 이 1초 주기로 보낸다.
       * 데몬이 MQTT state/daemon 의 diag 객체로 그대로 올린다. */
      proto_u16 tx_fail;
      proto_u16 rx_ovf;
      proto_u16 enc_retry;
      proto_u16 lidar_drop;
      proto_u16 reject_busy;

      /* v6: CMD_STATUS 를 한 번이라도 받았나. 0 이면 위 카운터는 의미 없다
       * (구버전 펌웨어이거나 아직 첫 주기가 안 왔다). */
      proto_u8  status_seen;
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

/* 9. 컴파일 타임 계약 검증
 *
 *  구조체 크기가 3자(STM32 펌웨어 / 커널 드라이버 / 데몬) 중 한 곳에서만
 *  달라지면 프레임이 조용히 어긋난다(패딩·타입 폭 차이로 실제 겪을 수 있음).
 *  여기서 빌드 자체를 깨뜨려 막는다 — "배열 크기가 음수" 에러가 난다.
 *  (매크로 대신 직접 typedef — MISRA 20.10 '##' 회피) */
typedef char proto_assert_scan_point_18B
    [(sizeof(struct proto_scan_point) == 18u) ? 1 : -1];
typedef char proto_assert_scan_start_10B
    [(sizeof(struct proto_scan_start) == 10u) ? 1 : -1];
typedef char proto_assert_scan_done_4B
    [(sizeof(struct proto_scan_done) == 4u) ? 1 : -1];
typedef char proto_assert_homed_8B
    [(sizeof(struct proto_homed) == 8u) ? 1 : -1];
typedef char proto_assert_point_fits_payload
    [(sizeof(struct proto_scan_point) <= PROTO_MAX_PAYLOAD) ? 1 : -1];
/* v6 */
typedef char proto_assert_err_2B
    [(sizeof(struct proto_err) == 2u) ? 1 : -1];
typedef char proto_assert_status_15B
    [(sizeof(struct proto_status) == 15u) ? 1 : -1];
typedef char proto_assert_status_fits_payload
    [(sizeof(struct proto_status) <= PROTO_MAX_PAYLOAD) ? 1 : -1];

#endif /* PROTOCOL_H */
