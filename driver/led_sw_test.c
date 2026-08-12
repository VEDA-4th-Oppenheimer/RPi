/* ============================================================================
 *  led_sw_test.c  --  /dev/led_sw 드라이버 테스트 프로그램
 * ==========================================================================*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <poll.h>
#include <sys/ioctl.h>

#include "../shared/led_sw.h"

static void print_usage(const char *prog)
{
    (void)fprintf(stderr,
        "Usage: %s [command]\n"
        "Commands:\n"
        "  led <green:0|1> <yellow:0|1> <red:0|1> <buzzer:0|1> Set all states\n"
        "  state                                    Get current LED and Switch states\n"
        "  monitor                                  Monitor switch press events in real-time\n",
        prog);
}

int main(int argc, char *argv[])
{
    int fd;
    int ret = 0;

    if (argc < 2) {
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }

    fd = open(LED_SW_DEV_PATH, O_RDWR | O_CLOEXEC);
    if (fd < 0) {
        (void)fprintf(stderr, "Failed to open %s: %s\n", LED_SW_DEV_PATH, strerror(errno));
        return EXIT_FAILURE;
    }

    if (strcmp(argv[1], "led") == 0) {
        struct led_sw_ctrl ctrl;
        if (argc < 6) {
            print_usage(argv[0]);
            (void)close(fd);
            return EXIT_FAILURE;
        }

        ctrl.green  = (uint8_t)atoi(argv[2]);
        ctrl.yellow = (uint8_t)atoi(argv[3]);
        ctrl.red    = (uint8_t)atoi(argv[4]);
        ctrl.buzzer = (uint8_t)atoi(argv[5]);

        if (ioctl(fd, LED_SW_SET_LEDS, &ctrl) < 0) {
            (void)fprintf(stderr, "ioctl(LED_SW_SET_LEDS) failed: %s\n", strerror(errno));
            ret = -1;
        } else {
            (void)printf("States updated: Green=%u, Yellow=%u, Red=%u, Buzzer=%u\n",
                         ctrl.green, ctrl.yellow, ctrl.red, ctrl.buzzer);
        }
    } else if (strcmp(argv[1], "state") == 0) {
        struct led_sw_state st;

        if (ioctl(fd, LED_SW_GET_STATE, &st) < 0) {
            (void)fprintf(stderr, "ioctl(LED_SW_GET_STATE) failed: %s\n", strerror(errno));
            ret = -1;
        } else {
            (void)printf("Current Device State:\n");
            (void)printf("  LED Green (BCM 27, Pin 11)  : %s\n", st.leds[LED_GREEN] ? "ON" : "OFF");
            (void)printf("  LED Yellow (BCM 17, Pin 13) : %s\n", st.leds[LED_YELLOW] ? "ON" : "OFF");
            (void)printf("  LED Red (BCM 22, Pin 15)    : %s\n", st.leds[LED_RED] ? "ON" : "OFF");
            (void)printf("  Buzzer (BCM 26, Pin 37)     : %s\n", st.leds[LED_BUZZER] ? "ON" : "OFF");
            (void)printf("  SW ScanStart (BCM 23, Pin 16): %s\n", st.sw[SW_SCAN_START] ? "PRESSED" : "RELEASED");
            (void)printf("  SW EMS (BCM 24, Pin 18)      : %s\n", st.sw[SW_EMS] ? "PRESSED" : "RELEASED");

        }
    } else if (strcmp(argv[1], "monitor") == 0) {
        struct pollfd pfd;

        (void)printf("Monitoring switch events on %s (Press Ctrl+C to exit)...\n", LED_SW_DEV_PATH);
        pfd.fd = fd;
        pfd.events = POLLIN;

        while (1) {
            int pnum = poll(&pfd, 1, 5000);
            if (pnum < 0) {
                if (errno == EINTR)
                    continue;
                (void)fprintf(stderr, "poll() failed: %s\n", strerror(errno));
                break;
            }
            if (pnum == 0) {
                (void)printf("Waiting for switch press...\n");
                continue;
            }

            if (pfd.revents & POLLIN) {
                struct led_sw_event evt;
                ssize_t n = read(fd, &evt, sizeof(evt));
                if (n == (ssize_t)sizeof(evt)) {
                    (void)printf("[EVENT] Switch ID: %u (%s), State: %s, Timestamp: %u ms\n",
                                 evt.sw_id,
                                 (evt.sw_id == SW_SCAN_START) ? "SCAN_START" :
                                 (evt.sw_id == SW_EMS)        ? "EMS" : "UNKNOWN",
                                 evt.state ? "PRESSED" : "RELEASED",
                                 evt.timestamp_ms);
                }
            }
        }
    } else {
        print_usage(argv[0]);
        ret = -1;
    }

    (void)close(fd);
    return (ret == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
