# A.D.T.S — Raspberry Pi (드라이버 · 통합 데몬)

**Anti-Drone Tracking & Targeting System** 의 라즈베리파이(엣지 서버) 측 코드.
`/dev/turret` 커널 드라이버로 STM32와 UART 통신하고, 통합 데몬이 카메라 메타데이터·라이다 거리를 융합해 3D 좌표를 산출한다.

- **보드**: Raspberry Pi 4 / **커널**: Linux 6.12.y (LTS) 고정
- **STM32 링크**: `/dev/turret` (serdev, USART1)

---

## 📂 디렉토리 구조

```
.
├── shared/                  # ★ 통신 계약 (single source of truth)
│   ├── protocol.h           #   RPi↔STM32 UART 규약 — 이 파일이 마스터.
│   │                        #   STM32 repo 가 drift-check 로 이걸 대조함.
│   └── daemon_module.h      #   데몬 코어 ↔ 모듈 계약
│
├── driver/                  # /dev/turret 커널 드라이버 (이현우)
│   ├── turret_driver.c      #   serdev char driver
│   ├── turret_test.c        #   유저 테스트 앱
│   ├── turret-overlay.dts   #   Device Tree 오버레이
│   ├── Makefile             #   kbuild + 크로스컴파일
│   └── KERNEL_BUILD.md      #   빌드/커널 소스 정렬 가이드
│
├── daemon/                  # 통합 데몬 (adts_daemon)
│   ├── core/                #   epoll 루프·FSM·좌표변환·pcd 내보내기 (이현우)
│   └── modules/
│       ├── mqtt/            #   브로커 연동 (이현우 + 이광진)
│       ├── imu/             #   /dev/imu (MPU-6050) 수평 기준 (송영빈)
│       └── led/             #   ⏳ STUB — /dev/led 미구현
│
├── broker/                  # Mosquitto 설정·인증서 발급 (이광진)
│   ├── gen-certs.sh         #   CA/서버/클라이언트 인증서
│   ├── mosquitto.conf.example
│   └── mosquitto.acl.example
│
├── docker/                  # 컨테이너 빌드 (macOS 에서 리눅스 전용 API 빌드용)
│
├── vision/                  # CLAHE/샤프닝 이미지 보정 (⏳ 이영민)
│
├── tools/                   # 정적분석 설정
│   ├── cppcheck_suppressions.txt
│   └── run_static_analysis.sh
└── .github/workflows/       # CI (정적분석 게이트)
```

---

## 🔗 protocol.h — 이 repo 가 마스터

`shared/protocol.h` 는 **RPi↔STM32 통신 계약의 단일 원본**이다.
- **드라이버는 사본 없이 `../shared/protocol.h` 를 직접 include** (Makefile 경로 설정).
- **STM32 repo** 는 이 파일의 사본을 두고, CI **drift-check** 로 이 마스터와 대조 → 불일치 시 PR 차단.
- 프로토콜 변경은 **여기서 먼저** 하고 PROTO_VERSION 을 올린다. (현재 v5)

---

## 🔨 빌드

### 커널 드라이버 (driver/)
```bash
cd driver
# RPi 에서 로컬 빌드
make
# 또는 크로스컴파일 (커널 소스는 6.12.y 로 정렬 — KERNEL_BUILD.md 참고)
make rpi

# 오버레이
make dtbo

# 적재
sudo insmod turret_driver.ko
```
⚠️ `.ko` vermagic 이 실행 커널과 맞아야 함 → 커널 소스를 `rpi-6.12.y` 로 정렬 (KERNEL_BUILD.md).

### 통합 데몬 (daemon/)

```bash
sudo apt install -y cmake libmosquitto-dev libcjson-dev
cmake -S daemon -B daemon/build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build daemon/build
sudo ./daemon/build/adts_daemon          # 인자 없이 = 상주, MQTT 트리거 대기
```

⚠️ `libmosquitto-dev`/`libcjson-dev` 가 없으면 **에러 없이 MQTT 를 비활성으로**
빌드한다(경고만 뜨고 넘어감). 데몬은 뜨지만 브로커에 붙지 않는다. 빌드 로그에
`libmosquitto/libcjson 를 찾지 못해 MQTT 를 비활성으로 빌드합니다` 가 보이면
의존성부터 설치할 것.

⚠️ `sudo` 가 필요하다. 데몬이 읽는 `/etc/adts/certs/daemon.key` 가 `0600 root:root`
라서 일반 계정으로 실행하면 TLS 접속이 실패한다.

CLI 로 1회 스캔만 돌리려면 `--scan` 을 쓴다 (`--help` 참고):

```bash
sudo ./daemon/build/adts_daemon --scan 0 1790 -900 900 10 --height 2400 --once
```

---

## 🔐 MQTT 브로커 · 인증서

브로커(Mosquitto)는 **RPi 에 상주**하고 데몬·Qt 관제·카메라가 모두 이 브로커의
클라이언트다. 포트 8883 + mTLS 이며, 권한은 **인증서 CN** 으로 판정한다
(`use_identity_as_username true`).

### 최초 구축 (1회)

```bash
sudo bash broker/gen-certs.sh <RPi_IP> /etc/adts/certs
sudo cp broker/mosquitto.conf.example /etc/mosquitto/conf.d/adts.conf
sudo cp broker/mosquitto.acl.example  /etc/mosquitto/conf.d/adts.acl
sudo systemctl restart mosquitto
```

`ca.key` 는 **이 장비 밖으로 내보내지 않는다.** 클라이언트에게는 인증서와 그
클라이언트의 키만 전달한다.

### 클라이언트 1개 추가 발급

Qt 관제 콘솔을 쓰는 사람이 늘면 사람마다 CN 을 따로 발급한다.

```bash
sudo bash broker/gen-certs.sh --client qt-console-youngbin /etc/adts/certs
```

`<CN>.crt` 와 `<CN>-trad.key` 가 만들어진다. **`-trad.key` 가 Qt 에 줄 키다** —
전통 RSA 포맷이어야 하고, PKCS#8 이면 `QSslKey` 가 null 을 반환하며 조용히 실패한다.

발급 후 **반드시** ACL 에 그 CN 블록을 추가하고 reload 한다:

```bash
sudo tee -a /etc/mosquitto/conf.d/adts.acl <<'EOF'

user qt-console-youngbin
topic write adts/cmd/#
topic read  adts/state/#
topic read  adts/event/#
EOF
sudo systemctl reload mosquitto
```

mosquitto ACL 의 `user` 는 **정확 매칭**이라 와일드카드가 없다. 이 단계를 빠뜨리면
TLS 핸드셰이크는 성공하는데 구독·발행만 막혀서 원인을 찾기 어렵다.

### 발급 서비스 `/enroll` (⏳ 구현 예정 — 송영빈)

Qt 배포본에는 인증서도 카메라 설정도 담지 않는다. 인증서에는 `adts/cmd/#` 쓰기
권한이 있어 장비를 움직일 수 있고, 카메라 설정에는 admin 비밀번호가 RTSP URL 에
박혀 있어서, 배포물에 넣으면 받은 사람 전원이 그 권한을 갖기 때문이다.

대신 사용자가 1회용 토큰을 입력하면 이 서비스가 인증서와 설정을 한 번에 내려준다.
Qt 클라이언트는 이 계약대로 이미 구현돼 있다(`src/EnrollDialog`).

```
POST https://<RPi>:8443/enroll
    {"token": "...", "device_name": "..."}

200 {"cn":"qt-console-<사용자>",
     "ca_crt":"...", "client_crt":"...", "client_key":"...",
     "mqtt":{"host":"...","port":8883},
     "cameras":{"channels":{"1":"rtsp://...", ...}}}

401/409 {"error":"사유"}
```

구현 시 지켜야 할 것:

| 항목 | 내용 |
|---|---|
| 인증서 발급 | `gen-certs.sh --client <CN>` 재사용 — 서명 로직을 새로 짜지 말 것 |
| ACL | 발급할 때마다 CN 블록 append + `systemctl reload mosquitto`. **가장 놓치기 쉽다** |
| 키 포맷 | `<CN>-trad.key`(전통 RSA)를 `client_key` 로 내려줄 것 |
| 서버 인증서 | 기존 `server.crt` 재사용 가능 — SAN 에 IP 가 들어 있다 |
| 토큰 | 1회용. 사용 후 소멸시킬 것 |
| 권한 | CA 키(`/etc/adts/certs/ca.key`)를 읽어야 하므로 실행 계정 설계에 주의 |

Qt 는 실행파일에 박아둔 `ca.crt` 로 이 서버의 신원을 검증한다(시스템 CA 는 쓰지
않는다). CA 를 재발급하면 Qt 저장소의 `resources/ca.crt` 도 함께 갱신해야 한다.

### 로그아웃 · 접근 차단

Qt 의 로그아웃은 **기기에 저장된 인증서·설정을 지울 뿐**이다. 발급된 인증서 자체는
여전히 유효해서, 파일을 따로 보관해 뒀다면 다시 붙을 수 있다.

- **권한만 끊기**: ACL 에서 해당 CN 블록 삭제 → reload. 연결은 되지만 아무것도 못 한다.
- **연결까지 끊기**: `mosquitto.conf` 에 `crlfile` 을 걸고 인증서를 폐기 목록에 올린다.
  기기 분실 대응이 필요해지면 도입한다(현재 미적용).

### 문제 해결

브로커에서 직접 토픽을 보면 어느 구간이 끊겼는지 빨리 갈린다.

```bash
mosquitto_sub -h <RPi_IP> -p 8883 \
  --cafile ca.crt --cert qt-console.crt --key qt-console-trad.key \
  -t 'adts/#' -v -i debug-$$        # -i: 다른 클라이언트와 Client ID 가 겹치지 않게
```

`adts/state/daemon` 이 `"online":false` 면 브로커는 살아 있고 **데몬이 죽은** 것이다.

---

## 🛡️ 정적분석 (push 전 로컬 검사)

```bash
bash tools/run_static_analysis.sh      # repo 루트에서
```
- 현재: 드라이버(cppcheck). 데몬 코드 추가되면 daemon 분석 잡 추가 예정.
- CI(`.github/workflows/static_analysis.yml`)가 push/PR 시 자동 실행 → **지적 시 머지 차단**.
- 전제: `cppcheck` 설치 (`brew install cppcheck` / `apt install cppcheck`).

---

## 👥 소유권 (CODEOWNERS)

| 경로 | 담당 |
|---|---|
| `shared/`, `driver/`, `daemon/core/` | 이현우 |
| `daemon/modules/mqtt/` | 이현우 + 이광진 |
| `daemon/modules/imu/` | 송영빈 |
| `daemon/modules/led/` | 이현우 (⏳ STUB) |
| `broker/` | 이광진 |
| `vision/` | 이영민 |
| `tools/`, `.github/` | 강유근 (QA) |

GitHub 핸들은 `CODEOWNERS` 참조. 다섯 명 모두 저장소 협업자라 PR 리뷰어가
자동 지정된다. 폴더가 늘면 `CODEOWNERS` 와 이 표를 **같이** 고칠 것.

---

## ⚠️ 주의
- `protocol.h` 변경 = **여기(마스터) 먼저** → STM32 사본 동기화 (drift-check 가 강제).
- `*.ko`·`build/`·`compile_commands.json` 커밋 금지 (`.gitignore` 처리됨).
- 커널 버전은 **6.12.y 고정** (재현성·vermagic).
