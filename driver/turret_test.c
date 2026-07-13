/*
 * turret_test.c — /dev/turret ioctl 유저 테스트 앱
 *
 * STM32/라이다/모터 없이도 "protocol.h 프레임이 ioctl→드라이버→UART TX 로
 * 제대로 나가는지" 검증하기 위한 도구. STM32(USART1)가 프레임을 받아
 * USART2(ST-Link VCP) printf 로 [feed]/CRC OK 를 찍는 걸로 왕복 확인한다.
 *
 * 빌드:  make turret_test   (또는 gcc -DPROTO_WANT_IOCTL -I../ -o turret_test turret_test.c)
 *
 * 사용법:
 *   ./turret_test target <theta_ddeg> <phi_ddeg> [count] [interval_ms]
 *       예) ./turret_test target 900 450            # 1회 (90.0°, 45.0°)
 *           ./turret_test target 1200 300 50 100    # 50회, 100ms 간격 연속 송신
 *   ./turret_test home
 *   ./turret_test mode <manual|track>
 *   ./turret_test disarm
 *   ./turret_test dist                              # QUERY_DIST 후 거리 수신(상행선 필요)
 *   ./turret_test state                             # 드라이버 캐시 상태 조회
 *   ./turret_test seq [count]                       # 시나리오 연속 송신(스윕)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>

#ifndef PROTO_WANT_IOCTL
#define PROTO_WANT_IOCTL          /* 유저공간에서 TURRET_* ioctl 매크로 활성화 */
#endif
#include "protocol.h"

#define DEV "/dev/turret"

static void usage(const char *p)
{
	fprintf(stderr,
	"Usage:\n"
	"  %s target <theta_ddeg> <phi_ddeg> [count] [interval_ms]\n"
	"  %s home\n"
	"  %s mode <manual|track>\n"
	"  %s disarm\n"
	"  %s dist\n"
	"  %s state\n"
	"  %s seq [count]\n"
	"    theta 0..%d(0.1deg), phi %d..%d(0.1deg)\n",
	p, p, p, p, p, p, p, THETA_MAX, PHI_MIN, PHI_MAX);
}

static void msleep(long ms)
{
	struct timespec ts = { ms / 1000, (ms % 1000) * 1000000L };
	nanosleep(&ts, NULL);
}

static int do_target(int fd, int theta, int phi)
{
	struct proto_target t = { .theta_ddeg = (proto_s16)theta,
				  .phi_ddeg   = (proto_s16)phi };
	if (ioctl(fd, TURRET_SET_TARGET, &t) < 0) {
		fprintf(stderr, "SET_TARGET(%d,%d) failed: %s\n",
			theta, phi, strerror(errno));
		return -1;
	}
	return 0;
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

	if (!strcmp(argv[1], "target")) {
		if (argc < 4) { usage(argv[0]); rc = 1; goto out; }
		int theta = atoi(argv[2]);
		int phi   = atoi(argv[3]);
		int count = (argc >= 5) ? atoi(argv[4]) : 1;
		long iv   = (argc >= 6) ? atol(argv[5]) : 0;
		if (count < 1) count = 1;

		printf("→ SET_TARGET theta=%d phi=%d ×%d (interval=%ldms)\n",
		       theta, phi, count, iv);
		for (int i = 0; i < count; i++) {
			if (do_target(fd, theta, phi) < 0) { rc = 1; break; }
			if (iv) msleep(iv);
		}
		if (!rc) printf("✓ %d 프레임 전송 완료 (STM32 VCP 로그 확인)\n", count);
	}
	else if (!strcmp(argv[1], "home")) {
		if (ioctl(fd, TURRET_HOME) < 0) {
			fprintf(stderr, "HOME failed: %s\n", strerror(errno));
			rc = 1;
		} else printf("✓ CMD_HOME 전송\n");
	}
	else if (!strcmp(argv[1], "mode")) {
		if (argc < 3) { usage(argv[0]); rc = 1; goto out; }
		struct proto_mode m;
		if (!strcmp(argv[2], "manual"))      m.mode = MODE_MANUAL;
		else if (!strcmp(argv[2], "track"))  m.mode = MODE_TRACK;
		else { usage(argv[0]); rc = 1; goto out; }
		if (ioctl(fd, TURRET_SET_MODE, &m) < 0) {
			fprintf(stderr, "SET_MODE failed: %s\n", strerror(errno));
			rc = 1;
		} else printf("✓ CMD_SET_MODE(%s) 전송\n", argv[2]);
	}
	else if (!strcmp(argv[1], "disarm")) {
		if (ioctl(fd, TURRET_DISARM) < 0) {
			fprintf(stderr, "DISARM failed: %s\n", strerror(errno));
			rc = 1;
		} else printf("✓ CMD_DISARM 전송\n");
	}
	else if (!strcmp(argv[1], "dist")) {
		struct proto_distance d;
		if (ioctl(fd, TURRET_GET_DISTANCE, &d) < 0) {
			fprintf(stderr, "GET_DISTANCE failed: %s "
				"(상행선 PA9→GPIO15 + STM CMD_DISTANCE 구현 필요)\n",
				strerror(errno));
			rc = 1;
		} else printf("✓ distance = %u mm\n", d.distance_mm);
	}
	else if (!strcmp(argv[1], "state")) {
		struct turret_link_state s;
		if (ioctl(fd, TURRET_GET_STATE, &s) < 0) {
			fprintf(stderr, "GET_STATE failed: %s\n", strerror(errno));
			rc = 1;
		} else {
			printf("link_alive=%u flags=0x%02X theta=%d phi=%d last_err=%u\n",
			       s.link_alive, s.flags, s.cur_theta_ddeg,
			       s.cur_phi_ddeg, s.last_err);
		}
	}
	else if (!strcmp(argv[1], "seq")) {
		int count = (argc >= 3) ? atoi(argv[2]) : 1;
		if (count < 1) count = 1;
		printf("→ 시나리오 ×%d: HOME→TRACK→SET_TARGET 스윕→DISARM\n", count);
		for (int c = 0; c < count && !rc; c++) {
			struct proto_mode m = { .mode = MODE_TRACK };
			if (ioctl(fd, TURRET_HOME) < 0) { rc = 1; break; }
			msleep(20);
			if (ioctl(fd, TURRET_SET_MODE, &m) < 0) { rc = 1; break; }
			msleep(20);
			/* theta 0→3599 스윕(10도씩), phi 고정 450 */
			for (int th = 0; th <= THETA_MAX; th += 100) {
				if (do_target(fd, th, 450) < 0) { rc = 1; break; }
				msleep(20);
			}
			if (ioctl(fd, TURRET_DISARM) < 0) { rc = 1; break; }
			msleep(20);
		}
		if (!rc) printf("✓ 시나리오 %d회 완료 (STM32 VCP 로그 확인)\n", count);
	}
	else {
		usage(argv[0]); rc = 1;
	}

out:
	close(fd);
	return rc;
}
