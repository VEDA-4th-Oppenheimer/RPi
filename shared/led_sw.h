/* ============================================================================
 *  led_sw.h  --  /dev/led_sw LED 및 스위치 캐릭터 드라이버 헤더
 * ----------------------------------------------------------------------------
 *  RPi GPIO 직결 LED(초록, 노랑, 빨강) 및 스위치(스캔시작, EMS) 제어 계약
 *  - 커널 모듈 (driver/led_sw_driver.c)
 *  - 유저 데몬   (daemon/modules/led/led_module.c)
 *  - 테스트 앱   (driver/led_sw_test.c)
 * ==========================================================================*/
#ifndef LED_SW_H
#define LED_SW_H

#ifdef __KERNEL__
  #include <linux/types.h>
  #include <linux/ioctl.h>
  typedef __u8  led_sw_u8;
  typedef __u16 led_sw_u16;
  typedef __u32 led_sw_u32;
#else
  #include <stdint.h>
  #include <sys/ioctl.h>
  typedef uint8_t  led_sw_u8;
  typedef uint16_t led_sw_u16;
  typedef uint32_t led_sw_u32;
#endif

#define LED_SW_DEV_NAME  "led_sw"
#define LED_SW_DEV_PATH  "/dev/led_sw"

/* LED 채널 ID */
enum led_channel {
    LED_GREEN  = 0,   /* 명령코드 중 제어코드 동작 중 */
    LED_YELLOW = 1,   /* 명령 대기 중                 */
    LED_RED    = 2,   /* 에러(코드) 발생              */
    LED_BUZZER = 3,   /* 부저 제어                    */
    LED_MAX    = 4
};

/* 스위치 ID */
enum switch_id {
    SW_SCAN_START = 1,  /* 스캔 시작 (CMD_SCAN_START) */
    SW_EMS        = 2,  /* 즉시 정지 (CMD_DISARM)     */
    SW_MAX        = 3
};

/* 스위치 이벤트 구조체 (read() / poll() 스트리밍용) */
struct led_sw_event {
    led_sw_u8  sw_id;         /* enum switch_id (SW_SCAN_START, SW_EMS) */
    led_sw_u8  state;         /* 1 = Pressed, 0 = Released               */
    led_sw_u32 timestamp_ms;  /* 커널 틱/시각 (ms)                       */
};

/* LED 제어 구조체 (ioctl LED_SW_SET_LEDS) */
struct led_sw_ctrl {
    led_sw_u8 green;   /* 1 = ON, 0 = OFF */
    led_sw_u8 yellow;  /* 1 = ON, 0 = OFF */
    led_sw_u8 red;     /* 1 = ON, 0 = OFF */
    led_sw_u8 buzzer;
};

/* 상태 조회 구조체 (ioctl LED_SW_GET_STATE) */
struct led_sw_state {
    led_sw_u8 leds[LED_MAX];    /* 각 LED ON/OFF 상태     */
    led_sw_u8 sw[SW_MAX];       /* 각 스위치 눌림 상태   */
};

/* ioctl 정의 */
#define LED_SW_IOC_MAGIC    'L'

#define LED_SW_SET_LEDS     _IOW(LED_SW_IOC_MAGIC, 1, struct led_sw_ctrl)
#define LED_SW_GET_STATE    _IOR(LED_SW_IOC_MAGIC, 2, struct led_sw_state)
#define LED_SW_SET_SINGLE   _IOW(LED_SW_IOC_MAGIC, 3, led_sw_u32) /* (ch << 16) | (on & 0xFFFF) */

#endif /* LED_SW_H */
