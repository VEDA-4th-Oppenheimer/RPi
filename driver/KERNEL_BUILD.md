# 커널 버전 고정 & 빌드 환경 가이드 (/dev/turret)

> `/dev/turret` 커널 드라이버(`turret_driver.ko`)를 안정적으로 빌드·로드하기 위한 커널 버전 고정 및 빌드 방법 정리.
> 대상: RPi 4 / 현재 RPi OS → 최종 Yocto 커스텀 이미지(경량화) 이관 예정.

---

## 0. 왜 이게 중요한가

커널 모듈(`.ko`)은 **실행 중인 커널과 vermagic(버전 문자열) + 심볼 CRC가 정확히 일치**해야 `insmod` 된다.
불일치 시 `insmod: ERROR: could not insert module ... Invalid module format` 또는
`disagrees about version of symbol` 에러가 난다.

→ 커널이 apt 업데이트로 바뀌면 어제 되던 드라이버가 오늘 안 뜬다. 데모 사고 방지를 위해 **커널 버전 고정**이 필요하다.

---

## 1. 권장 커널: Linux 6.12 LTS (`rpi-6.12.y`)

| 이유 | 설명 |
| --- | --- |
| **LTS** | 장기 유지보수 커널 → 데모·최종까지 안정 |
| **드라이버 소스와 일치** | 커널 **6.10부터 serdev `receive_buf` 반환형이 `int → size_t`로 변경**됨. 현재 `turret_driver.c`는 `size_t` 기준 → 6.12에서 **소스 수정 0** |
| **RPi OS ↔ Yocto 정렬** | 최신 RPi OS(Bookworm) 및 최신 Yocto(Styhead/Walnascar)의 `linux-raspberrypi`가 6.12 → 지금 검증한 게 Yocto 이관 후에도 유효 |

### ⚠️ 6.6 LTS 주의
6.6도 LTS지만 `receive_buf`가 아직 **`int` 시그니처**라, 현재 드라이버를 그대로 쓰면 **컴파일 에러**가 난다.
6.6으로 가려면 드라이버를 `int` 반환형으로 되돌리거나, 아래 §4 버전 가드를 반드시 적용해야 한다.
(참고: Yocto 4년 LTS인 Scarthgap이 6.6을 사용)

---

## 2. 현재 커널 확인 & 고정

```bash
uname -r        # 현재 실행 커널 확인
```

- **`6.12.x` 로 나오면** → 그대로 고정 (최선)

```bash
sudo apt update
sudo apt install raspberrypi-kernel-headers    # 네이티브 빌드용 헤더
sudo apt-mark hold raspberrypi-kernel raspberrypi-kernel-headers  # apt upgrade 시 커널 고정
```

- **`6.6.x` 로 나오면** → 6.12 이미지로 업그레이드 권장, 또는 6.6 고정 + §4 버전 가드 적용

> `apt-mark hold`는 **개발 중 커널이 안 바뀌게 막는 최소 고정**이다.
> 최종 재현성은 Yocto가 담당하므로(§5), 지금 무거운 이미지 스냅샷까지는 불필요하다.

---

## 3. 빌드 방법 — 네이티브 vs 크로스컴파일

### 핵심: out-of-tree 모듈은 **전체 커널 빌드가 필요 없다**

`turret_driver.ko`는 커널 내장 모듈이 아니라 별도 소스다. kbuild가 필요로 하는 건 4가지뿐:

1. 커널 **소스 트리**(또는 headers)
2. **`.config`** (타깃 커널 설정)
3. **`Module.symvers`** (심볼 CRC — 로드 시 매칭용)
4. **준비된 scripts/생성 헤더** (`make modules_prepare`로 생성)

`vmlinux`(커널 본체)나 수백 개 다른 모듈은 **안 만들어도 된다**.

### ✅ 방법 A — Pi 네이티브 빌드 (권장, 커널 빌드 0)

```bash
sudo apt install raspberrypi-kernel-headers    # 이미 준비된 트리 제공
cd turret_driver
make                                           # /lib/modules/$(uname -r)/build 사용
```

- headers 패키지가 `.config` + `Module.symvers` + prepared scripts를 전부 포함한
  준비된 트리를 `/lib/modules/$(uname -r)/build`에 설치한다.
- 커널을 **한 번도 빌드하지 않고** `.ko`가 나오며, 심볼 CRC가 실행 커널과 자동 일치 → `insmod` 바로 됨.

### ⚙️ 방법 B — 호스트 크로스컴파일 (속도용, 준비 필요)

크로스는 headers의 scripts가 arm64 바이너리라 x86 호스트에서 못 돈다. 따라서 **소스가 필요**하다.

```bash
# 1) uname -r 과 정확히 맞는 rpi 커널 소스 확보 (rpi-source 또는 git tag 체크아웃)
# 2) Pi의 .config 복사 (/proc/config.gz 또는 /boot/config-*)
# 3) 준비만 수행 (전체 커널 빌드 아님)
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- modules_prepare
# 4) 우리 모듈만 빌드
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- -C <kernel_src> M=$PWD modules
```

> **함정**: `modules_prepare`만으론 정확한 `Module.symvers`가 안 생긴다.
> insmod가 되려면 타깃의 `Module.symvers`(headers 패키지 안에 있음)를 소스 트리에 갖다놔야 한다.
> 이 매칭이 번거로워서 사람들이 "그냥 커널을 통째로 빌드"하게 되는 것.
> → **속도 이득 대비 번거로움이 커서 현 단계엔 비추. 방법 A 권장.**

### 빌드 후 아키텍처 확인 (필수)

```bash
file turret_driver.ko                       # ARM aarch64 여야 함
modinfo turret_driver.ko | grep vermagic    # 실행 커널과 일치해야 insmod 가능
```

---

## 4. (권장) 드라이버 버전 가드 — 커널 버전 무관 빌드

커널 버전에 상관없이 빌드되게 `turret_driver.c`의 `receive_buf` 콜백을 아래처럼 감싸면,
6.6이든 6.12든 소스 수정 없이 컴파일된다. → 커널 고정이 "소스 호환성"이 아닌 "안정성" 문제로 내려가고,
Yocto가 어떤 6.x를 쓰든 그대로 컴파일된다.

```c
#include <linux/version.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,10,0)
static size_t turret_rx_callback(struct serdev_device *serdev,
                                 const unsigned char *buf, size_t count)
#else
static int turret_rx_callback(struct serdev_device *serdev,
                              const unsigned char *buf, size_t count)
#endif
{
    /* ... 본문 동일 ... */
    return count;   /* size_t/int 양쪽 모두 count 반환으로 호환 */
}
```

---

## 5. 최종 단계 — Yocto 이관 시 (경량화)

Yocto(meta-raspberrypi)는 `linux-raspberrypi` 레시피로 **커널을 소스에서 빌드**하고,
우리 모듈도 레시피(`inherit module`)가 그 커널에 맞춰 **자동 재컴파일**한다.
→ 지금의 vermagic/Module.symvers 매칭 문제가 **원천적으로 사라진다.**

### 지금 해둘 것 (Yocto 이관 대비)
- 커널 모듈 Makefile을 `KDIR ?=` + `obj-m` + `$(MAKE) -C $(KDIR) M=$(PWD)` 형태로 유지 → `module.bbclass`가 그대로 호출 (✅ 현재 충족)
- `.dts` 오버레이 소스 유지 (✅ `turret-overlay.dts`)
- `MODULE_LICENSE("GPL")` 명시 (✅)
- **검증한 커널 버전 기록** → Yocto `PREFERRED_VERSION_linux-raspberrypi`를 그에 맞춰 재현
- 런타임/빌드 의존성 목록화 → Yocto `DEPENDS`/`IMAGE_INSTALL`로 직행

### 예정 레이어 구조 (미리보기)
```
meta-adts/
├── recipes-kernel/turret-driver/turret-driver_1.0.bb   # inherit module
├── recipes-adts/turret-daemon/turret-daemon_1.0.bb
├── recipes-bsp/device-tree/adts-overlay.bb             # dtbo + config.txt
└── recipes-core/images/adts-image.bb                   # 경량 이미지 (no desktop)
```
경량화 목표는 `adts-image.bb`의 `IMAGE_INSTALL`에 최소 패키지만 담아 달성
(RPi OS 수백 MB → 필요한 것만 담은 수십 MB 이미지).

---

## 요약

| 단계 | 커널 | 빌드 방법 | 비고 |
| --- | --- | --- | --- |
| **현재 (RPi OS)** | 6.12 LTS 고정 (`apt-mark hold`) | **Pi 네이티브** (`make`) | headers만 설치, 커널 빌드 0 |
| **최종 (Yocto)** | `linux-raspberrypi` 6.12 | bitbake 레시피 자동 | 매칭 문제 소멸, 경량화 |

- 크로스컴파일은 "전체 커널 빌드"가 아니라 "**타깃과 매칭되는 준비된 트리 확보**"가 필수 → 번거로워서 현 단계엔 비추.
- **버전 가드(§4)** 를 적용하면 커널 버전 선택이 자유로워진다.
