# RPi 빌드환경 (Docker) — 맥 M4 + CLion

VM/Gateway 없이 맥(M4)에서 RPi **데몬(epoll)** + **커널 드라이버(.ko)** 를 빌드.
소스는 레포를 런타임 마운트(`-v`)하므로 이 폴더는 레포 밖에 있어도 됨.
원리: [CrossCompile 문서](https://lkj000619.atlassian.net/wiki/spaces/VPT/pages/14450710/CrossCompile) / [Notion Docker Build](https://app.notion.com/p/3a7fdb06b70f8031b0f4e9c8ca6a6e54)

> 주의: 커널 값 `SUBLEVEL=75 / +rpt-rpi / -v8` 은 RPi `6.12.75+rpt-rpi-v8` 기준.
> Pi 커널 바뀌면 `uname -r` 보고 Dockerfile ARG + 아래 KREL 갱신.

## 1. 이미지 빌드 (1회)
```bash
docker build -t adts-build <repo>/docker
# Docker Desktop 메모리 8GB+ 권장 (커널 prepare OOM 방지)
```

## 2. Pi 고유 파일 2개 받기 (1회, 커널 바뀔 때만)
```bash
KREL=6.12.75+rpt-rpi-v8
PI=pi@10.144.31.125
mkdir -p ~/pi-kernel
scp $PI:/usr/src/linux-headers-$KREL/.config        ~/pi-kernel/.config
scp $PI:/usr/src/linux-headers-$KREL/Module.symvers ~/pi-kernel/Module.symvers
ssh $PI 'sudo apt-mark hold linux-image-rpi-v8 linux-headers-rpi-v8'   # 자동 업그레이드 차단
```
 `linux-headers` 디렉토리 통째/`linux-kbuild` 복사 금지 (aarch64 바이너리). 이 2개만.

## 3. 컨테이너 실행 + 커널 prepare (--rm 없이 유지)
```bash
docker run -it --name adts \
  -v <repo>:/work \
  -v ~/pi-kernel:/pi-kernel \
  adts-build bash
```
컨테이너 안:
```bash
cd /usr/src/linux
cp /pi-kernel/.config .config
export ARCH=arm64                 # M4 네이티브 → CROSS_COMPILE 생략
make olddefconfig
make LOCALVERSION= modules_prepare
cp /pi-kernel/Module.symvers .
cat include/config/kernel.release  # 핵심: 6.12.75+rpt-rpi-v8 (Pi uname -r) 와 100% 일치 확인
```
다음 세션엔 `docker start -ai adts` 로 재진입 (prepare 유지됨).

## 4. 빌드
```bash
# 드라이버
cd /work/driver && make clean && make rpi RPI_KDIR=/usr/src/linux LOCALVERSION=
modinfo -F vermagic turret_driver.ko   # 6.12.75+rpt-rpi-v8 SMP preempt mod_unload modversions aarch64

# 데몬
cd /work/daemon && cmake -S . -B build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && cmake --build build
bash /work/tools/run_static_analysis.sh daemon
```

## 5. CLion 연결
1. Settings → Build, Execution, Deployment → **Docker** → + → Docker for Mac (연결 확인)
2. Settings → **Toolchains** → + → Docker → Image: `adts-build`
3. **데몬**: `RPi/daemon` 을 CMake 프로젝트로 열고 CMake Profile의 Toolchain을 `Docker(adts-build)` 로 지정 → 컨테이너에서 빌드/디버그/cppcheck (편집·인덱싱은 맥).
4. **드라이버**: CMake 불가(Kbuild). 빌드는 `docker exec -it adts bash` 터미널에서 `make rpi`. 인덱싱은 컨테이너에서 `bear -- make rpi RPI_KDIR=/usr/src/linux LOCALVERSION=` → 생긴 `driver/compile_commands.json` 을 CLion Compilation Database로 열기.

## 6. 배포 & 테스트 (RPi에서만 — 컨테이너는 커널 없어 insmod 불가)
```bash
scp <repo>/driver/turret_driver.ko $PI:~/
ssh -t $PI 'sudo insmod ~/turret_driver.ko && dmesg | tail -15 && lsmod | grep turret'
```
