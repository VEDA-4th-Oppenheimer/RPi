# Raspberry Pi (드라이버 · 통합 데몬)

1D LiDAR Pan-Tilt 스캐너 / 자동 캘리브레이션 킷의 라즈베리파이 측 코드.
커널 드라이버 3개로 하드웨어를 다루고, 데몬이 스캔을 제어해 `(pan, tilt, d)`
스트림을 포인트클라우드로 만든 뒤 raw data(.json)은 카메라에, 포인트 클라우드 파일(.pcd)는 QT(client)에 전달한다.

| | |
|---|---|
| 보드 / 커널 | Raspberry Pi 4 / Linux 6.12.y (LTS) 고정 |
| 드라이버 | `/dev/turret`(serdev, STM32) · `/dev/imu`(ICM-20948) · `/dev/led_sw`(LED×3 + 스위치×2 + 부저) |
| 상행 | MQTT-over-TLS 8883 (Qt) · mTLS TCP 2222 (카메라, 스캔 JSON) |


---

##  디렉토리 구조

```
.
├── shared/                  # 핵심: 통신 계약 (single source of truth)
│   ├── protocol.h           #   RPi↔STM32 UART 규약 — 이 파일이 마스터.
│   │                        #   STM32 repo 가 drift-check 로 이걸 대조함.
│   └── daemon_module.h      #   데몬 코어 ↔ 모듈 계약
│
├── driver/                  # 커널 드라이버 3종
│   ├── turret_driver.c      #   /dev/turret  serdev char driver (이현우)
│   ├── imu_driver.c         #   /dev/imu     ICM-20948 I2C (송영빈)
│   ├── led_sw_driver.c      #   /dev/led_sw  platform driver (강유근)
│   ├── *_test.c             #   유저 테스트 앱
│   ├── overlays/            #   Device Tree 오버레이 (*-overlay.dts)
│   ├── Makefile             #   kbuild + 크로스컴파일
│   └── KERNEL_BUILD.md      #   빌드/커널 소스 정렬 가이드
│
├── daemon/                  # 통합 데몬 (adts_daemon)
│   ├── core/                #   epoll 루프·FSM·좌표변환·산출물 (이현우)
│   ├── modules/
│   │   ├── mqtt/            #   브로커 연동 (이현우)
│   │   ├── imu/             #   /dev/imu 수평 게이트 (송영빈)
│   │   ├── led/             #   /dev/led_sw 표시·스위치 (강유근 + 이현우)
│   │   └── camera/          #   스캔 JSON mTLS 업로드 (이현우)
│   ├── adts-daemon.service  #   systemd 유닛
│   └── tools/               #   install-service.sh · scan_batch.sh · fake_camera.py
│
├── broker/                  # Mosquitto 설정·인증서 발급 (이현우 + 송영빈)
│   ├── README.md            #   구축·발급·운영 런북  <- 이쪽을 볼 것
│   ├── gen-certs.sh         #   CA/서버/클라이언트 인증서
│   ├── enroll_service.c     #   /enroll 발급 서비스 (C, OpenSSL + cJSON)
│   ├── CMakeLists.txt       #   adts_enroll 빌드
│   ├── adts-enroll.service  #   systemd 유닛
│   ├── enroll_tokens.example
│   ├── mosquitto.conf.example
│   └── mosquitto.acl.example
│
├── docker/                  # 컨테이너 빌드 (리눅스 외 OS 에서 리눅스 전용 API 빌드용)
│
├── tools/                   # 정적분석 설정
│   ├── cppcheck_suppressions.txt
│   └── run_static_analysis.sh
└── .github/workflows/       # CI (정적분석 게이트)
```

---

##  protocol.h

`shared/protocol.h` 가 RPi↔STM32 통신 Protocol. 현재 **v6**.

- 드라이버는 사본 없이 `../shared/protocol.h` 를 직접 include 한다.
  `driver/protocol.h` 같은 사본을 두면 `-I$(src)` 가 앞이라 마스터를 가린다.
- STM32 repo 는 사본을 두고 CI drift-check 로 대조한다. 불일치 시 PR 차단.
- 변경은 RPI 먼저 하고 `PROTO_VERSION` 을 올린다.
- **push 순서**: rpi `main` 먼저 → STM32. drift-check 가 rpi `main` 의 raw 를 본다.

---

##  빌드

### 커널 드라이버 (driver/)

```bash
cd driver
make            # RPi 에서 로컬 빌드
make rpi        # 크로스컴파일 (KERNEL_BUILD.md)
make dtbo       # 오버레이
sudo insmod turret_driver.ko
```

- `.ko` vermagic 이 실행 커널과 맞아야 한다 → 커널 소스를 `rpi-6.12.y` 로 정렬.
- **드라이버와 데몬은 같이 재빌드한다.** proto v5·v6 에서 `turret_link_state` 가
  커져 ioctl 매직이 바뀌었다. 한쪽만 갈면 `-ENOTTY` 로 실패한다.
- `led_sw` 는 DT 오버레이가 필요하다. 없으면 `-EPROBE_DEFER(-517)` 로 probe 실패 —
  이 커널은 gpiochip base 가 512 라 BCM 번호(17, 27 …)로는 불가.

### 통합 데몬 (daemon/)

```bash
sudo apt install -y cmake libmosquitto-dev libcjson-dev libssl-dev
cmake -S daemon -B daemon/build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build daemon/build
./daemon/build/adts_daemon                 # 인자 없이 = 상주, MQTT 트리거 대기
```

의존성이 빠지면 **에러 없이 그 기능만 꺼진 채** 빌드된다. configure 로그를 볼 것.

| 없으면 | 결과 |
|---|---|
| `libmosquitto-dev` / `libcjson-dev` | MQTT 비활성 — 브로커에 안 붙음 |
| `libssl-dev` | 카메라 업로드 비활성 (평문 폴백 없음) |

인증서 권한은 [`broker/README.md`](broker/README.md) 참조.

CLI 1회 스캔. 아래가 표준값이며 물리 버튼·웹·`scan_batch.sh` 도 같다
(`daemon_module.h` 의 `SCAN_DEF_*`):

```bash
./daemon/build/adts_daemon --scan 0 1791 -900 900 9 --height 1805 --once
```

---

##  MQTT 브로커 · 인증서

브로커(Mosquitto)는 RPi 에 상주하고 데몬·Qt·카메라가 모두 클라이언트다.
8883 + mTLS, 권한은 인증서 CN 으로 판정한다.

구축·발급·운영은 [`broker/README.md`](broker/README.md).

---

##  정적분석 (push 전 로컬 검사)

```bash
bash tools/run_static_analysis.sh          # 전체
bash tools/run_static_analysis.sh daemon   # 개별
```

`driver`(cppcheck) · `daemon`(-Werror 빌드 + cppcheck) · `broker`(같음) 세 트랙.

- CI 는 **`main` 브랜치에만** 걸린다. required check 는 세 잡 전부.
- `develop` 에 쌓이는 동안은 CI 가 안 보므로 push 전에 로컬에서 돌릴 것.
- 로컬 통과가 CI 통과를 보장하지 않는다. CI 는 cppcheck 2.13 이라 로컬(2.21)에
  없는 오탐이 나온다.
- 전제: `cppcheck` 설치.

---

## 주의

- `protocol.h` 변경은 여기(마스터) 먼저 → STM32 사본 동기화.
- 상주 서비스가 `/dev/turret` 을 점유한다. CLI 스캔 전에 `sudo systemctl stop adts-daemon`.
- `*.ko`·`build/`·`compile_commands.json` 커밋 금지.
- 커널은 6.12.y 고정 (재현성·vermagic).
