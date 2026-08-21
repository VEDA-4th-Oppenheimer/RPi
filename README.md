# ADTS — Raspberry Pi (드라이버 · 통합 데몬)

**1D LiDAR Pan-Tilt 스캐너 / 자동 캘리브레이션 킷**의 라즈베리파이(엣지 서버) 측 코드.
자작 커널 드라이버 3개로 하드웨어를 다루고, 통합 데몬이 스캔을 지휘해
`(pan, tilt, d)` 스트림을 포인트클라우드로 만든 뒤 카메라 단에 올린다.

> 프로젝트명 `A.D.T.S`(Anti-Drone…)는 2026-07-22 주제 전환 **이전**의 것이다.
> 코드·경로에 남아 있지만 지금 하는 일은 안티드론과 무관하다.

- **보드**: Raspberry Pi 4 / **커널**: Linux 6.12.y (LTS) 고정
- **자작 드라이버**: `/dev/turret`(serdev, STM32 링크) · `/dev/imu`(ICM-20948) · `/dev/led_sw`(LED×3 + 스위치×2 + 부저)
- **상행**: MQTT-over-TLS 8883 (Qt 관제) · mTLS TCP 2222 (카메라로 스캔 JSON)

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
│   │   ├── mqtt/            #   브로커 연동 (이현우 + 이광진)
│   │   ├── imu/             #   /dev/imu 수평 게이트 (송영빈)
│   │   ├── led/             #   /dev/led_sw 표시·스위치 (강유근 + 이현우)
│   │   └── camera/          #   스캔 JSON mTLS 업로드 (이현우)
│   ├── adts-daemon.service  #   systemd 유닛
│   └── tools/               #   install-service.sh · scan_batch.sh · fake_camera.py
│
├── broker/                  # Mosquitto 설정·인증서 발급 (이광진)
│   ├── README.md            #   구축·발급·운영 런북  <- 이쪽을 볼 것
│   ├── gen-certs.sh         #   CA/서버/클라이언트 인증서
│   ├── enroll_service.c     #   /enroll 발급 서비스 (C, OpenSSL + cJSON)
│   ├── CMakeLists.txt       #   adts_enroll 빌드
│   ├── adts-enroll.service  #   systemd 유닛
│   ├── enroll_tokens.example
│   ├── mosquitto.conf.example
│   └── mosquitto.acl.example
│
├── docker/                  # 컨테이너 빌드 (macOS 에서 리눅스 전용 API 빌드용)
│
├── tools/                   # 정적분석 설정
│   ├── cppcheck_suppressions.txt
│   └── run_static_analysis.sh
└── .github/workflows/       # CI (정적분석 게이트)
```

---

##  protocol.h — 이 repo 가 마스터

`shared/protocol.h` 는 **RPi↔STM32 통신 계약의 단일 원본**이다.
- **드라이버는 사본 없이 `../shared/protocol.h` 를 직접 include** (Makefile 경로 설정).
- **STM32 repo** 는 이 파일의 사본을 두고, CI **drift-check** 로 이 마스터와 대조 → 불일치 시 PR 차단.
- 프로토콜 변경은 **여기서 먼저** 하고 PROTO_VERSION 을 올린다. (현재 **v6**)

⚠️ **push 순서**: rpi `main` 을 먼저 반영한 뒤 STM32 를 push 한다. drift-check 가
rpi `main` 의 raw 를 보므로 역순이면 STM32 PR 이 막힌다.

⚠️ **복붙 사본 금지.** `driver/protocol.h` 같은 사본을 두면 Makefile 의 `-I$(src)` 가
`-I$(src)/../shared` 보다 앞이라 **마스터를 가려** 옛 헤더로 조용히 빌드된다.

---

##  빌드

### 커널 드라이버 (driver/)
```bash
cd driver
# RPi 에서 로컬 빌드
make
# 또는 크로스컴파일 (커널 소스는 6.12.y 로 정렬 — KERNEL_BUILD.md 참고)
make rpi

# 오버레이
make dtbo

# 적재 (3종 전부 만들어진다)
sudo insmod turret_driver.ko
```
주의: `.ko` vermagic 이 실행 커널과 맞아야 함 → 커널 소스를 `rpi-6.12.y` 로 정렬 (KERNEL_BUILD.md).

주의: **드라이버와 데몬은 반드시 같이 재빌드한다.** proto v5·v6 에서 `turret_link_state`
가 커져 `TURRET_GET_STATE` 의 ioctl 매직이 두 번 바뀌었다. 한쪽만 갈면 `-ENOTTY` 로
즉시 실패한다(조용한 구조체 오해석보다 안전하게 그렇게 만들었다).

주의: `led_sw` 는 DT 오버레이가 적용돼 있어야 한다. 없으면 `of_get_named_gpio` 가
`-EPROBE_DEFER(-517)` 를 내고 probe 가 실패한다 — 이 커널은 **gpiochip base 가 512**
라 모듈 파라미터의 BCM 번호(17, 27 …)로는 절대 성공할 수 없다.

### 통합 데몬 (daemon/)

```bash
sudo apt install -y cmake libmosquitto-dev libcjson-dev libssl-dev
cmake -S daemon -B daemon/build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build daemon/build
sudo ./daemon/build/adts_daemon          # 인자 없이 = 상주, MQTT 트리거 대기
```

주의: `libmosquitto-dev`/`libcjson-dev` 가 없으면 **에러 없이 MQTT 를 비활성으로**
빌드한다(경고만 뜨고 넘어감). 데몬은 뜨지만 브로커에 붙지 않는다. 빌드 로그에
`libmosquitto/libcjson 를 찾지 못해 MQTT 를 비활성으로 빌드합니다` 가 보이면
의존성부터 설치할 것.

주의: `libssl-dev` 가 없으면 **카메라 업로드가 비활성으로** 빌드된다. 평문으로
되돌리지 않고 아예 끄는 쪽을 택했다 — 보안 기능이 조용히 꺼진 채 도는 것이
실패보다 나쁘고, 스캔 파일은 로컬에 남으므로 잃는 것도 없다. configure 로그에
`카메라 업로드: mTLS 활성` 이 보여야 한다.

주의: 인증서 권한 때문에 일반 계정으로는 TLS 가 실패할 수 있다 —
[`broker/README.md`](broker/README.md) 의 "최초 구축" 절 참조. 권한을 맞춰두면
`sudo` 없이 돌아간다(systemd 유닛도 `User=pi` 다).

CLI 로 1회 스캔만 돌리려면 `--scan` 을 쓴다 (`--help` 참고). 아래가 **표준 스캔**이며
물리 버튼·웹·`scan_batch.sh` 도 같은 값을 쓴다(`daemon_module.h` 의 `SCAN_DEF_*`):

```bash
./daemon/build/adts_daemon --scan 0 1791 -900 900 9 --height 1805 --once
```

---

##  MQTT 브로커 · 인증서

브로커(Mosquitto)는 **RPi 에 상주**하고 데몬·Qt 관제·카메라가 모두 이 브로커의
클라이언트다. 포트 8883 + mTLS 이며, 권한은 **인증서 CN** 으로 판정한다.

**구축·발급·운영 절차는 [`broker/README.md`](broker/README.md) 에 있다.**
인증서 발급, ACL, `/enroll` 발급 서비스, 문제 해결이 전부 그쪽이다.

⚠️ 데몬을 처음 띄울 때 가장 자주 걸리는 것은 **`daemon.key` 권한**이다. `gen-certs.sh`
가 `600 root` 로 두는데 데몬은 `User=pi` 로 돌아서, 그대로면 TLS 가 `MOSQ_ERR_INVAL`
("Invalid function arguments")로 실패한다 — **권한 문제로 안 보인다.**
`broker/README.md` 의 "최초 구축" 절 참조.


##  정적분석 (push 전 로컬 검사)

```bash
bash tools/run_static_analysis.sh      # repo 루트에서
```
세 트랙을 돈다 — `driver`(cppcheck) · `daemon`(-Werror 빌드 + cppcheck) ·
`broker`(같음). 개별로 돌리려면 `bash tools/run_static_analysis.sh daemon`.

- CI(`.github/workflows/static_analysis.yml`)는 **`main` 브랜치에만** 걸린다(팀 결정).
  required check 는 `driver-analysis` + `daemon-analysis` + `broker-analysis` 셋.
- ⚠️ 그래서 `develop` 에 쌓이는 동안은 CI 가 안 봐준다. **push 전에 위 명령을 돌릴 것.**
- ⚠️ 로컬 통과가 CI 통과를 보장하지 않는다. CI 의 cppcheck 는 2.13 이라 로컬(2.21)에
  없는 오탐이 나온다(MISRA 11.8 등). 실제로 로컬 green 인데 CI 가 막은 적이 있다.
- 전제: `cppcheck` 설치 (`brew install cppcheck` / `apt install cppcheck`).

---

##  소유권 (CODEOWNERS)

| 경로 | 담당 |
|---|---|
| `shared/`, `driver/`, `daemon/core/` | 이현우 |
| `daemon/modules/mqtt/` | 이현우 + 이광진 |
| `daemon/modules/imu/` | 송영빈 |
| `daemon/modules/led/` | 강유근 (드라이버) + 이현우 (모듈) |
| `daemon/modules/camera/` | 이현우 |
| `broker/` | 이광진 |
| `tools/`, `.github/` | 강유근 (QA) |

GitHub 핸들은 `CODEOWNERS` 참조. 다섯 명 모두 저장소 협업자라 PR 리뷰어가
자동 지정된다. 폴더가 늘면 `CODEOWNERS` 와 이 표를 **같이** 고칠 것.

---

## 주의

- `protocol.h` 변경 = **여기(마스터) 먼저** → STM32 사본 동기화 (drift-check 가 강제).
- 상주 서비스가 `/dev/turret` 을 **점유**한다. CLI 로 스캔하려면 먼저
  `sudo systemctl stop adts-daemon`.
- `*.ko`·`build/`·`compile_commands.json` 커밋 금지 (`.gitignore` 처리됨).
- 커널 버전은 **6.12.y 고정** (재현성·vermagic).
