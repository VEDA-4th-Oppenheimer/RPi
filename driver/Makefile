# driver/Makefile
#
# RPi에서 빌드:
#   make            커널 모듈(.ko) + 유저 테스트 앱
#   make dtbo       Device Tree 오버레이(turret-overlay.dtbo)
#   make clean      정리
#
# protocol.h 마스터는 rpi repo 의 ../shared/ 에 있다 (single source).

# ★ Makefile이 위치한 현재 디렉토리 경로를 절대경로로 고정 (Kbuild clean/all 오류 방지)
PWD := $(CURDIR)

KDIR ?= /lib/modules/$(shell uname -r)/build

# 라즈베리파이 크로스컴파일 기본값 ('make rpi' 에서 사용)
RPI_KDIR  ?= /usr/src/linux
RPI_ARCH  ?= arm64
RPI_CROSS ?= aarch64-linux-gnu-

# 유저 앱 컴파일러: CROSS_COMPILE 지정 시 그 크로스 gcc, 아니면 호스트 gcc
APP_CC := $(CROSS_COMPILE)gcc

# =====================================================================
# ★ 방산 표준 커널/유저 컴파일러 보안 및 경고 플래그 강제화
# =====================================================================
# [커널 공간] 경고 극대화 및 스택 강도 강화
EXTRA_CFLAGS += -Wall -Wextra -Werror \
                -fstack-protector-strong \
                -D_FORTIFY_SOURCE=2 \
                -Wformat -Wformat-security

# [유저 공간] 완전한 하드닝 적용 (ASLR, 메모리 보호, 포맷 스트링 방어)
APP_CFLAGS := -Wall -Wextra -Werror \
              -fstack-protector-all \
              -D_FORTIFY_SOURCE=2 \
              -Wformat -Wformat-security \
              -fPIE -pie \
              -z relro -z now

# ── 커널 모듈 (kbuild) ──
obj-m     += turret_driver.o imu_driver.o
ccflags-y += -I$(src) -I$(src)/.. -I$(src)/../shared $(EXTRA_CFLAGS)

# ── 유저 앱: protocol.h 위치 자동 탐색 (../shared 우선) ──
PROTO_H := $(firstword $(wildcard protocol.h) $(wildcard ../shared/protocol.h) $(wildcard ../protocol.h))

# ── 타겟 ──
.PHONY: all rpi modules clean dtbo

all: modules turret_test imu_test

# 라즈베리파이(arm64) 크로스 빌드 원샷: .ko + 테스트 앱 둘 다 arm64
rpi:
	$(MAKE) ARCH=$(RPI_ARCH) CROSS_COMPILE=$(RPI_CROSS) KDIR=$(RPI_KDIR) all

modules:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

turret_test: turret_test.c $(PROTO_H)
	$(APP_CC) $(APP_CFLAGS) -DPROTO_WANT_IOCTL -I. -I.. -I../shared -o $@ $<

imu_test: imu_test.c
	$(APP_CC) $(APP_CFLAGS) -I. -I.. -I../shared -o $@ $< -lm

dtbo: turret-overlay.dtbo

turret-overlay.dtbo: turret-overlay.dts
	dtc -@ -I dts -O dtb -o $@ $<

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
	rm -f turret_test imu_test turret-overlay.dtbo