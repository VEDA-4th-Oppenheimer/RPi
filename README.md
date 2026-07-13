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
├── daemon/                  # 통합 데몬 (⏳ 스캐폴딩, 구현 예정)
│   ├── core/                #   epoll 루프·상태머신·shared_ctx (이현우)
│   └── modules/
│       ├── tracking/        #   칼만·기하변환(이현우) + 비주얼서보잉(송영빈)
│       ├── meta/            #   MQTT bbox 수신 (이영민)
│       └── tls/             #   Qt push TLS (이광진)
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
- 프로토콜 변경은 **여기서 먼저** 하고 PROTO_VERSION 을 올린다. (현재 v3)

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
⏳ 구현 예정 (CMake 빌드).

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
| `daemon/modules/tracking/` | 이현우 + 송영빈 |
| `daemon/modules/meta/`, `vision/` | 이영민 |
| `daemon/modules/tls/` | 이광진 |
| `tools/`, `.github/` | 강유근 (QA) |

---

## ⚠️ 주의
- `protocol.h` 변경 = **여기(마스터) 먼저** → STM32 사본 동기화 (drift-check 가 강제).
- `*.ko`·`build/`·`compile_commands.json` 커밋 금지 (`.gitignore` 처리됨).
- 커널 버전은 **6.12.y 고정** (재현성·vermagic).
