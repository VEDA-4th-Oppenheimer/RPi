# Dockerfile — RPi 데몬(userspace, epoll) + 커널 드라이버(turret_driver.ko) 통합 빌드 환경
# 위치: <repo>/docker/ (소스는 COPY 안 하고 docker run -v 로 마운트한다)
# 원리: CrossCompile 문서(Confluence 14450710)의 vermagic 절차를 컨테이너화.
# M4(arm64) 네이티브 → RPi(arm64)와 같은 아키텍처라 크로스툴체인 불필요, 그냥 gcc.
#
# 빌드:  docker build -t adts-build <repo>/docker
# 사용:  docker run -it --name adts \
#          -v <repo>:/work \
#          -v ~/pi-kernel:/pi-kernel \
#          adts-build bash
FROM ubuntu:24.04

# 현재 RPi 커널 6.12.75+rpt-rpi-v8 기준. Pi 커널 바뀌면 uname -r 보고 갱신.
ARG RPI_BRANCH=rpi-6.12.y
ARG SUBLEVEL=75

# --- 데몬 툴 + 커널 빌드 의존성 (bear = CLion 드라이버 인덱싱용 compile_commands) ---
#   ninja-build       : CLion Docker 툴체인의 기본 제너레이터
#   libmosquitto-dev  : mqtt_module (MQTT 클라이언트)
#   libcjson-dev      : MQTT 페이로드 파싱·생성
#   mosquitto(-clients): 브로커·pub/sub — mTLS end-to-end 테스트용
RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates build-essential cmake ninja-build cppcheck gdb git bear \
        vim less file \
        bc bison flex libssl-dev libelf-dev kmod \
        device-tree-compiler \
        libmosquitto-dev libcjson-dev mosquitto mosquitto-clients \
    && rm -rf /var/lib/apt/lists/*

# --- RPi 커널 소스 clone + SUBLEVEL 일치 커밋 pin (브랜치 tip 쓰면 SUBLEVEL 이동해 불일치) ---
#   ⚠️ git log -S(pickaxe) + --reverse 는 전체 히스토리를 훑어 blob:none에선 커밋마다
#      원격 blob 페치 → 사실상 무한정 느림. tip 에서 Makefile 건드린 커밋만 거슬러 올라가며
#      SUBLEVEL 일치 첫 커밋에서 멈춤(=blob 수십 개만 페치, 빠름).
RUN git clone --branch ${RPI_BRANCH} --filter=blob:none \
        https://github.com/raspberrypi/linux.git /usr/src/linux
WORKDIR /usr/src/linux
RUN PIN=$(git log --first-parent --format=%H -- Makefile | while read c; do \
              if git show "$c:Makefile" | grep -q "^SUBLEVEL = ${SUBLEVEL}$"; then \
                  echo "$c"; break; \
              fi; \
          done) \
    && test -n "$PIN" \
    && echo "pinned commit = $PIN (SUBLEVEL=${SUBLEVEL})" \
    && git checkout -b build-pin "$PIN" \
    && git show HEAD:Makefile | grep -E '^(VERSION|PATCHLEVEL|SUBLEVEL) '

# --- 릴리스 문자열 중간 조각 (KERNELVERSION + 이것 + CONFIG_LOCALVERSION = 6.12.75+rpt-rpi-v8) ---
RUN printf '+rpt-rpi' > /usr/src/linux/localversion-rpt

WORKDIR /work
CMD ["/bin/bash"]
