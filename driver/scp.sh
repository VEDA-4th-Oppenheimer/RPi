#!/usr/bin/env bash
# ============================================================================
#  deploy_to_pi.sh — 크로스빌드 산출물(.ko/.dtbo/test)을 RPi로 전송
#  실행 위치: VM 의 RPi/driver/ (make rpi && make dtbo 이후)
# ============================================================================
set -euo pipefail

PI="pi@10.144.31.125"          # 대상 라즈베리파이
DEST="final_project/driver"           # Pi 홈(~) 아래 배포 폴더
FILES=(turret_driver.ko turret-overlay.dtbo turret_test)

# 스크립트가 있는 driver/ 로 이동 (어디서 실행해도 동작)
cd "$(dirname "$0")"

# 산출물 존재 확인
for f in "${FILES[@]}"; do
    [[ -f "$f" ]] || { echo " 없음: $f  (먼저 make rpi / make dtbo)"; exit 1; }
done

# Pi 에 폴더 생성 후 전송
ssh "$PI" "mkdir -p ~/$DEST"
scp "${FILES[@]}" "$PI:~/$DEST/"

echo " 전송 완료 → $PI:~/$DEST/"
printf '   %s\n' "${FILES[@]}"
cat <<EOF

── Pi 에서 적용 (참고) ──
  ssh $PI
  cd ~/$DEST
  sudo dtoverlay turret-overlay.dtbo     # DT 오버레이 적용(런타임)
  sudo insmod turret_driver.ko           # 드라이버 로드
  ls -l /dev/turret && dmesg | tail -20  # 노드 생성/바인딩 확인
  ./turret_test                          # 유저 테스트앱
EOF

