/*
 * turret_test.c — /dev/turret 유저 테스트 앱 (protocol.h v4 스캐너)
 *
 * STM32/라이다/모터 없이도 "protocol.h v4 프레임이 ioctl→드라이버→UART TX 로
 * 제대로 나가는지" + "STM 상행 스캔 스트림이 read()/poll() 로 올라오는지" 검증.
 *
 * 빌드:  make turret_test
 *        (또는 gcc -DPROTO_WANT_IOCTL -I../shared -o turret_test turret_test.c)
 *
 * 사용법:
 *   ./turret_test home
 *   ./turret_test scan [th0 th1 ph0 ph1 step]   # 기본=전체 방(0 3599 -900 900 10)
 *   ./turret_test stop
 *   ./turret_test disarm
 *   ./turret_test ping
 *   ./turret_test state                          # 드라이버 캐시(pong_seq 포함)
 *   ./turret_test stream [timeout_ms]            # poll+read 로 스캔 점 수신(기본 5000ms)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <poll.h>

#ifndef PROTO_WANT_IOCTL
#define PROTO_WANT_IOCTL          /* 유저공간에서 TURRET_* ioctl 매크로 활성화 */
#endif
#include "protocol.h"

#define DEV "/dev/turret"

static void usage(const char *p)
{
	fprintf(stderr,
	"Usage:\n"
	"  %s home\n"
	"  %s scan [th0 th1 ph0 ph1 step]   (기본 0 %d %d %d 10)\n"
	"  %s stop\n"
	"  %s disarm\n"
	"  %s ping\n"
	"  %s state\n"
	"  %s stream [timeout_ms]\n"
	"    theta 0..%d, phi %d..%d (0.1deg), step>0 (10=1.0deg)\n",
	p, p, THETA_MAX, PHI_MIN, PHI_MAX, p, p, p, p, p,
	THETA_MAX, PHI_MIN, PHI_MAX);
}

static void print_state(const struct turret_link_state *s)
{
	printf("link_alive=%u flags=0x%02X(homed=%d scanning=%d) "
	       "theta=%d phi=%d last_err=%u pong_seq=%u\n",
	       s->link_alive, s->flags,
	       (s->flags & STF_HOMED) ? 1 : 0,
	       (s->flags & STF_SCANNING) ? 1 : 0,
	       s->cur_theta_ddeg, s->cur_phi_ddeg, s->last_err, s->pong_seq);
}

/* poll()+read() 로 스캔 점 스트림 수신. SCAN_DONE(스캐닝 해제) or 타임아웃까지. */
static int do_stream(int fd, int timeout_ms)
{
	struct pollfd pfd = { .fd = fd, .events = POLLIN };
	unsigned long total = 0;
	int saw_scanning = 0;

	printf("→ stream 시작 (timeout=%dms, SCAN_DONE 까지 대기)\n", timeout_ms);

	for (;;) {
		int pr = poll(&pfd, 1, timeout_ms);

		if (pr < 0) {
			if (errno == EINTR)
				continue;
			perror("poll");
			return -1;
		}
		if (pr == 0) {
			printf("… %dms 무이벤트 → 종료 (총 %lu점)\n",
			       timeout_ms, total);
			return 0;
		}
		if (pfd.revents & POLLERR) {
			fprintf(stderr, "POLLERR: link_dead (총 %lu점)\n", total);
			return -1;
		}
		if (pfd.revents & POLLIN) {
			struct proto_scan_point pts[128];
			ssize_t n = read(fd, pts, sizeof(pts));

			if (n > 0) {
				int cnt = (int)(n / (ssize_t)sizeof(pts[0]));

				total += (unsigned long)cnt;
				/* 배치 끝 점만 샘플 출력(플러딩 방지) */
				printf("  +%d점 (누적 %lu)  마지막: θ=%d φ=%d d=%umm\n",
				       cnt, total,
				       pts[cnt - 1].theta_ddeg,
				       pts[cnt - 1].phi_ddeg,
				       pts[cnt - 1].d_mm);
			} else if (n < 0 && errno != EAGAIN) {
				perror("read");
				return -1;
			}

			/* 통지 확인: 스캔 완료/에러 감지 */
			struct turret_link_state s;

			if (ioctl(fd, TURRET_GET_STATE, &s) == 0) {
				if (s.flags & STF_SCANNING)
					saw_scanning = 1;
				if (s.last_err)
					printf("  [STM ERROR] code=%u\n", s.last_err);
				if (saw_scanning && !(s.flags & STF_SCANNING)) {
					printf("✓ SCAN_DONE 감지 (총 %lu점)\n",
					       total);
					return 0;
				}
			}
		}
	}
}

int main(int argc, char *argv[])
{
	int fd, rc = 0;

	if (argc < 2) { usage(argv[0]); return 1; }

	fd = open(DEV, O_RDWR);
	if (fd < 0) {
		perror("open " DEV);
		fprintf(stderr, "  → dtoverlay + insmod turret_driver.ko 확인\n");
		return 1;
	}

	if (!strcmp(argv[1], "home")) {
		if (ioctl(fd, TURRET_HOME) < 0) {
			fprintf(stderr, "HOME failed: %s\n", strerror(errno));
			rc = 1;
		} else {
			printf("✓ CMD_HOME 전송\n");
		}
	}
	else if (!strcmp(argv[1], "scan")) {
		struct proto_scan_start ss = {
			.theta_start_ddeg = 0,
			.theta_end_ddeg   = THETA_MAX,
			.phi_start_ddeg   = PHI_MIN,
			.phi_end_ddeg     = PHI_MAX,
			.step_ddeg        = 10,        /* 1.0도 격자 */
		};

		if (argc >= 7) {
			ss.theta_start_ddeg = (proto_s16)atoi(argv[2]);
			ss.theta_end_ddeg   = (proto_s16)atoi(argv[3]);
			ss.phi_start_ddeg   = (proto_s16)atoi(argv[4]);
			ss.phi_end_ddeg     = (proto_s16)atoi(argv[5]);
			ss.step_ddeg        = (proto_u16)atoi(argv[6]);
		}
		if (ioctl(fd, TURRET_SCAN_START, &ss) < 0) {
			fprintf(stderr, "SCAN_START failed: %s "
				"(홈 전이면 STM 이 ERR_NOT_HOMED 응답)\n",
				strerror(errno));
			rc = 1;
		} else {
			printf("✓ CMD_SCAN_START θ[%d..%d] φ[%d..%d] step=%u\n",
			       ss.theta_start_ddeg, ss.theta_end_ddeg,
			       ss.phi_start_ddeg, ss.phi_end_ddeg, ss.step_ddeg);
		}
	}
	else if (!strcmp(argv[1], "stop")) {
		if (ioctl(fd, TURRET_SCAN_STOP) < 0) {
			fprintf(stderr, "SCAN_STOP failed: %s\n", strerror(errno));
			rc = 1;
		} else {
			printf("✓ CMD_SCAN_STOP 전송\n");
		}
	}
	else if (!strcmp(argv[1], "disarm")) {
		if (ioctl(fd, TURRET_DISARM) < 0) {
			fprintf(stderr, "DISARM failed: %s\n", strerror(errno));
			rc = 1;
		} else {
			printf("✓ CMD_DISARM 전송\n");
		}
	}
	else if (!strcmp(argv[1], "ping")) {
		if (ioctl(fd, TURRET_PING) < 0) {
			fprintf(stderr, "PING failed: %s\n", strerror(errno));
			rc = 1;
		} else {
			printf("✓ CMD_PING 전송 (PONG 은 state 의 pong_seq 로 확인)\n");
		}
	}
	else if (!strcmp(argv[1], "state")) {
		struct turret_link_state s;

		if (ioctl(fd, TURRET_GET_STATE, &s) < 0) {
			fprintf(stderr, "GET_STATE failed: %s\n", strerror(errno));
			rc = 1;
		} else {
			print_state(&s);
		}
	}
	else if (!strcmp(argv[1], "stream")) {
		int timeout_ms = (argc >= 3) ? atoi(argv[2]) : 5000;

		if (timeout_ms < 1)
			timeout_ms = 5000;
		rc = do_stream(fd, timeout_ms) < 0 ? 1 : 0;
	}
	else {
		usage(argv[0]);
		rc = 1;
	}

	close(fd);
	return rc;
}
