# A.D.T.S — Raspberry Pi (드라이버 · 통합 데몬)

**Anti-Drone Tracking & Targeting System** 의 라즈베리파이(엣지 서버) 측 코드.
`/dev/turret` 커널 드라이버로 STM32와 UART 통신하고, 통합 데몬이 카메라 메타데이터·라이다 거리를 융합해 3D 좌표를 산출한다.

- **보드**: Raspberry Pi 4 / **커널**: Linux 6.12.y (LTS) 고정
- **STM32 링크**: `/dev/turret` (serdev, USART1)

---

##  디렉토리 구조

```
.
├── shared/                  # 핵심: 통신 계약 (single source of truth)
│   ├── protocol.h           #   RPi↔STM32 UART 규약 — 이 파일이 마스터.
│   │                        #   STM32 repo 가 drift-check 로 이걸 대조함.
│   └── daemon_module.h      #   데몬 코어 ↔ 모듈 계약
│
├── driver/                  # /dev/turret 커널 드라이버 (이현우)
│   ├── turret_driver.c      #   serdev char driver
│   ├── turret_test.c        #   유저 테스트 앱
│   ├── overlays/            #   Device Tree 오버레이 (*-overlay.dts)
│   ├── Makefile             #   kbuild + 크로스컴파일
│   └── KERNEL_BUILD.md      #   빌드/커널 소스 정렬 가이드
│
├── daemon/                  # 통합 데몬 (adts_daemon)
│   ├── core/                #   epoll 루프·FSM·좌표변환·pcd 내보내기 (이현우)
│   └── modules/
│       ├── mqtt/            #   브로커 연동 (이현우 + 이광진)
│       ├── imu/             #   /dev/imu (ICM-20948) 수평 기준 (송영빈)
│       └── led/             #    STUB — /dev/led 미구현
│
├── broker/                  # Mosquitto 설정·인증서 발급 (이광진)
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
- 프로토콜 변경은 **여기서 먼저** 하고 PROTO_VERSION 을 올린다. (현재 v5)

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

# 적재
sudo insmod turret_driver.ko
```
주의: `.ko` vermagic 이 실행 커널과 맞아야 함 → 커널 소스를 `rpi-6.12.y` 로 정렬 (KERNEL_BUILD.md).

### 통합 데몬 (daemon/)

```bash
sudo apt install -y cmake libmosquitto-dev libcjson-dev
cmake -S daemon -B daemon/build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build daemon/build
sudo ./daemon/build/adts_daemon          # 인자 없이 = 상주, MQTT 트리거 대기
```

주의: `libmosquitto-dev`/`libcjson-dev` 가 없으면 **에러 없이 MQTT 를 비활성으로**
빌드한다(경고만 뜨고 넘어감). 데몬은 뜨지만 브로커에 붙지 않는다. 빌드 로그에
`libmosquitto/libcjson 를 찾지 못해 MQTT 를 비활성으로 빌드합니다` 가 보이면
의존성부터 설치할 것.

주의: `sudo` 가 필요하다. 데몬이 읽는 `/etc/adts/certs/daemon.key` 가 `0600 root:root`
라서 일반 계정으로 실행하면 TLS 접속이 실패한다.

CLI 로 1회 스캔만 돌리려면 `--scan` 을 쓴다 (`--help` 참고):

```bash
sudo ./daemon/build/adts_daemon --scan 0 1790 -900 900 10 --height 2400 --once
```

---

##  MQTT 브로커 · 인증서

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

### 발급 서비스 `/enroll` (`broker/enroll_service.c`)

Qt 배포본에는 인증서도 카메라 설정도 담지 않는다. 인증서에는 `adts/cmd/#` 쓰기
권한이 있어 장비를 움직일 수 있고, 카메라 설정에는 admin 비밀번호가 RTSP URL 에
박혀 있어서, 배포물에 넣으면 받은 사람 전원이 그 권한을 갖기 때문이다.

대신 사용자가 1회용 토큰을 입력하면 이 서비스가 인증서와 설정을 한 번에 내려준다.
Qt 클라이언트(`src/EnrollDialog`)가 이 계약대로 구현돼 있다.

```
POST https://<RPi>:8443/enroll
    {"token": "...", "device_name": "..."}

200 {"cn":"qt-console-<라벨>",
     "ca_crt":"...", "client_crt":"...", "client_key":"...",
     "mqtt":{"host":"...","port":8883},
     "cameras":{"channels":{"1":"rtsp://...", ...}}}

401 {"error":"토큰이 유효하지 않거나 이미 사용되었습니다"}
```

`GET /healthz` 로 살아있는지 확인할 수 있다(인증서·설정은 노출하지 않는다).

#### 스캔 파일 조회 `/scans` · `/scan/<파일명>`

`adts/state/scan` 은 `.pcd` **경로**만 준다(계약 §9). 그 경로는 RPi 기준이라 Qt 가
도는 PC 에는 없으므로, 발급 서비스에 읽기 전용 경로를 얹어 기존 mTLS 신뢰를 재사용한다.

```
GET /scans              → {"scans":[{"name":"...","size":123,"mtime":1786085386}, ...]}
GET /scan/<파일명>.pcd  → 파일 본문
```

인증서를 발급하는 서비스에 파일 경로를 여는 것이라 세 겹으로 막았다. ①검증된
클라이언트 인증서 필수(`/enroll` 은 인증서를 받기 전에 부르는 곳이라
`FAIL_IF_NO_PEER_CERT` 를 못 켠다 — 핸들러에서 `SSL_get_verify_result` 를 직접
본다) ②파일명만 받는다(`/` 나 `%` 가 있으면 400, 디렉터리는 `ADTS_SCAN_DIR` 고정)
③`.pcd` 확장자만.

**디렉터리가 함정이다.** `ADTS_SCAN_DIR` 은 데몬의 `SCAN_OUT_DIR`
(`daemon/core/main.c`)과 같은 `/var/lib/adts/scans` 여야 한다. 두 가지가 겹쳐 조용히
어긋난다.

- 유닛에 `ProtectHome=true` 가 걸려 있어 **`/home` 아래는 경로가 맞아도 못 읽는다.**
- 데몬은 `/var/lib/adts/scans` 를 못 만들면 작업 디렉터리의 `./scans` 로 폴백한다.
  데몬을 `pi` 홈에서 띄우면 파일은 홈 아래에 쌓이고, 서비스는 하나도 못 보게 된다.

이 상태의 증상은 `GET /scans` → `404 {"error":"스캔 디렉터리가 없습니다"}` 이고,
Qt 관제에는 "로컬 N건 · 서버 목록 실패(HTTP 404)" 로 나타난다. 위 설치 절차대로
`/var/lib/adts/scans` 를 미리 만들어 두면 양쪽이 같은 곳을 본다.

```bash
curl -s --cacert /etc/adts/certs/ca.crt \
     --cert /etc/adts/certs/qt-console.crt \
     --key  /etc/adts/certs/qt-console-trad.key \
     https://localhost:8443/scans
```

#### 빌드 · 설치

```bash
sudo apt install -y libssl-dev libcjson-dev
cmake -S broker -B broker/build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build broker/build

sudo mkdir -p /opt/adts
sudo cp broker/build/adts_enroll broker/gen-certs.sh /opt/adts/
sudo cp broker/enroll_tokens.example /etc/adts/enroll_tokens
sudo chmod 600 /etc/adts/enroll_tokens

# 스캔 디렉터리 — 데몬이 쓰고 발급 서비스가 읽는 곳. 없으면 데몬이 작업
# 디렉터리의 ./scans 로 폴백해 버려서 서비스가 파일을 하나도 못 본다(아래).
sudo mkdir -p /var/lib/adts/scans && sudo chown pi:pi /var/lib/adts/scans

sudo cp broker/adts-enroll.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now adts-enroll
```

이미 돌고 있는 서비스를 갱신할 때는 실행 중인 바이너리를 덮어쓸 수 없다
(`Text file busy`). 멈추고 복사한 뒤 다시 띄운다.

```bash
sudo systemctl stop adts-enroll
sudo cp broker/build/adts_enroll /opt/adts/
sudo systemctl start adts-enroll
```

TLS 는 브로커와 같은 `server.crt`/`server.key` 를 쓴다 — SAN 에 IP 가 들어 있어
IP 로 접속해도 검증된다. **클라이언트 인증서는 요구하지 않는다**. 아직 인증서가
없는 사람이 받으러 오는 곳이라, 신원 확인은 1회용 토큰이 한다. 반대 방향으로,
Qt 는 실행파일에 박아둔 `ca.crt` 로 이 서버를 검증한다(시스템 CA 는 쓰지 않는다).
**CA 를 재발급하면 Qt 저장소의 `resources/ca.crt` 도 함께 갱신해야 한다.**

#### 토큰 발급 · 회수 (관리자)

토큰은 관리자가 만들어 팀원에게 **전달**한다(채팅·구두 등). 라벨이 CN 접미사가
되어 `qt-console-<라벨>` 로 발급되고, **사용되는 즉시 파일에서 지워진다**(재사용 401).

```bash
sudo bash broker/gen-certs.sh --new-token youngbin   # 생성 + 등록 + 전달문 출력
sudo bash broker/gen-certs.sh --list-tokens          # 미사용 토큰 목록
sudo bash broker/gen-certs.sh --revoke youngbin      # 미사용 토큰 회수
```

`--new-token` 은 그대로 복사해 전달할 수 있는 안내문을 출력한다 — 서버 주소(이
장비의 IPv4 를 추정), 포트, 토큰, 그리고 앱에서 무엇을 하면 되는지까지.

라벨 문자 검증은 **토큰을 만들 때** 한다. 발급 시점에 걸러면 "토큰은 받았는데
등록에서 500" 이 되기 때문이다.

`--list-tokens` 는 토큰 앞뒤 몇 글자만 보여준다. 전문을 찍으면 화면·터미널 로그에
남는 것 자체가 접근 권한이 된다.

주의: `--revoke` 는 **아직 안 쓴 토큰만** 회수한다. 이미 발급받아 간 인증서는 그대로
유효하므로, 그 사람의 접근을 끊으려면 ACL 에서 `user qt-console-<라벨>` 블록을
지우고 reload 해야 한다(아래 "로그아웃 · 접근 차단" 참고).

#### 서비스가 하는 일

1. 토큰 검증 — 상수 시간 비교(`CRYPTO_memcmp`), 성공 시 그 줄을 지우고 파일을
   원자적으로 교체한다. 토큰이 틀렸는지 이미 썼는지는 **구분해서 알려주지 않는다**.
2. `gen-certs.sh --client <CN>` 호출 — 서명 로직을 다시 구현하지 않는다.
   확장(`v3_client`)이나 키 포맷(전통 RSA) 같은 세부가 갈라지지 않도록.
3. ACL 에 CN 블록 추가 후 `systemctl reload mosquitto`. 이미 있으면 건너뛴다.
4. 인증서 3종 + `mqtt` + `cameras` 를 JSON 으로 응답.

#### 알아둘 것

- **재발급(같은 라벨)** 은 기존 파일을 지우고 새로 만든다. 그런데 **이전 인증서는
  파일을 지운다고 폐기되지 않는다** — 암호학적으로 여전히 유효하다. 실제로
  무효화하려면 CRL 이 필요하다(아래 참고). 로그에 WARN 으로 남는다.
- ACL 갱신에 실패하면 **인증서는 발급됐는데 권한이 없는 상태**가 된다. 이때는
  500 을 반환하고 로그에 남으므로, `/etc/mosquitto/conf.d/adts.acl` 을 확인할 것.
- 단일 스레드로 한 연결씩 처리한다. 발급은 사람이 가끔 하는 작업이라 성능이
  문제되지 않고, 토큰·ACL 파일을 동시에 건드릴 일이 없어 잠금이 필요 없다.

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

##  정적분석 (push 전 로컬 검사)

```bash
bash tools/run_static_analysis.sh      # repo 루트에서
```
- 현재: 드라이버(cppcheck). 데몬 코드 추가되면 daemon 분석 잡 추가 예정.
- CI(`.github/workflows/static_analysis.yml`)가 push/PR 시 자동 실행 → **지적 시 머지 차단**.
- 전제: `cppcheck` 설치 (`brew install cppcheck` / `apt install cppcheck`).

---

##  소유권 (CODEOWNERS)

| 경로 | 담당 |
|---|---|
| `shared/`, `driver/`, `daemon/core/` | 이현우 |
| `daemon/modules/mqtt/` | 이현우 + 이광진 |
| `daemon/modules/imu/` | 송영빈 |
| `daemon/modules/led/` | 이현우 ( STUB) |
| `broker/` | 이광진 |
| `tools/`, `.github/` | 강유근 (QA) |

GitHub 핸들은 `CODEOWNERS` 참조. 다섯 명 모두 저장소 협업자라 PR 리뷰어가
자동 지정된다. 폴더가 늘면 `CODEOWNERS` 와 이 표를 **같이** 고칠 것.

---

## 주의: 주의
- `protocol.h` 변경 = **여기(마스터) 먼저** → STM32 사본 동기화 (drift-check 가 강제).
- `*.ko`·`build/`·`compile_commands.json` 커밋 금지 (`.gitignore` 처리됨).
- 커널 버전은 **6.12.y 고정** (재현성·vermagic).
