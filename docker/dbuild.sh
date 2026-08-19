#!/usr/bin/env bash
# 컨테이너(adts)에서 드라이버 빌드/클린. CLion Custom Build Target 에서 호출.
#   빌드:  dbuild.sh
#   클린:  dbuild.sh clean
# CLion 은 PATH 가 축소돼 있어 docker 절대경로를 쓴다.
set -uo pipefail

# 컴파일러가 뱉는 경로는 컨테이너 기준(/work/driver)이라 CLion 이 눌러도 파일을
# 못 연다. 맥 경로로 바꿔야 에러 줄 클릭이 먹는다. (스크립트 위치에서 역산)
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
fixpath() { sed "s|/work/driver|$REPO/driver|g"; }
DOCKER=/usr/local/bin/docker
[ -x "$DOCKER" ] || DOCKER=$(command -v docker) || { echo "docker 없음"; exit 1; }
CONTAINER=adts

# 컨테이너가 멈춰 있으면 자동 기동 (맥/Docker 재시작 후 대비)
if ! "$DOCKER" ps --filter "name=^${CONTAINER}$" --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
    echo ">> 컨테이너 ${CONTAINER} 기동 중..."
    "$DOCKER" start "$CONTAINER" >/dev/null || { echo "!! ${CONTAINER} 기동 실패 (docker run 으로 새로 만들어야 할 수 있음)"; exit 1; }
fi

if [ "${1:-}" = "clean" ]; then
    # 주의: compile_commands.json 은 지우지 않는다 (CLion 인덱싱이 그 파일에 물려 있어서
    #    지우면 "compile_commands.json 을 찾을 수 없습니다" 로 프로젝트가 깨진다).
    echo ">> clean (compile_commands.json 은 보존)"
    exec "$DOCKER" exec "$CONTAINER" bash -c \
        'cd /work/driver && rm -f *.o *.ko *.mod *.mod.c *.mod.o .*.cmd modules.order Module.symvers turret_test && echo "cleaned"'
fi

echo ">> build (vermagic 검증 포함)"
"$DOCKER" exec "$CONTAINER" bash -c \
    'cd /work/driver && make rpi RPI_KDIR=/usr/src/linux LOCALVERSION=' 2>&1 | fixpath

# 주의: 파이프를 거쳤으므로 $? 는 sed 것이다. make 의 실제 결과를 봐야
#   CLion 이 빌드 실패를 빨간불로 표시한다.
STATUS="${PIPESTATUS[0]}"
[ "$STATUS" -ne 0 ] && exit "$STATUS"

# vermagic 이 RPi uname -r 과 다르면 insmod 가 거부된다. 두 모듈 다 확인.
for ko in turret_driver imu_driver; do
    "$DOCKER" exec "$CONTAINER" bash -c \
        "modinfo -F vermagic /work/driver/$ko.ko 2>/dev/null | sed 's|^|vermagic $ko: |'"
done
