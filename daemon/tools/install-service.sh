#!/usr/bin/env bash
# ============================================================================
#  install-service.sh — 데몬을 systemd 서비스로 설치한다 (RPi 에서 실행)
# ----------------------------------------------------------------------------
#  하는 일
#    1. adts_daemon 바이너리를 /opt/adts/ 로
#    2. 커널 모듈을 /lib/modules/<커널>/extra/ 로 + 부팅 시 자동 적재 등록
#    3. dtbo 를 /boot/firmware/overlays/ 로 + config.txt 에 dtoverlay= 등록
#    4. adts-daemon.service 설치·활성화
#
#  ⚠️ 2·3 이 없으면 부팅 시 /dev/turret 이 없어서 서비스가 실패한다.
#     데몬만 다시 올리고 싶으면  --daemon-only
#
#  사용:  sudo bash daemon/tools/install-service.sh [--daemon-only] [--no-start]
# ==========================================================================*/
set -euo pipefail

DAEMON_ONLY=0
NO_START=0
for a in "$@"; do
    case "$a" in
        --daemon-only) DAEMON_ONLY=1 ;;
        --no-start)    NO_START=1 ;;
        -h|--help) sed -n '2,17p' "$0"; exit 0 ;;
        *) echo "알 수 없는 인자: $a" >&2; exit 2 ;;
    esac
done

[ "$(id -u)" -eq 0 ] || { echo "!! root 로 실행할 것 (sudo)" >&2; exit 1; }

# 레포 루트 = 이 스크립트의 두 단계 위 (daemon/tools/ → RPi/)
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
KREL="$(uname -r)"

# config.txt 위치는 OS 버전에 따라 다르다.
if   [ -f /boot/firmware/config.txt ]; then BOOTCFG=/boot/firmware/config.txt; BOOTOVL=/boot/firmware/overlays
elif [ -f /boot/config.txt ];          then BOOTCFG=/boot/config.txt;          BOOTOVL=/boot/overlays
else BOOTCFG=""; fi

say() { echo "── $*"; }

# ── 1. 데몬 바이너리 ────────────────────────────────────────────────────────
BIN=""
for c in "$REPO/daemon/build/adts_daemon" "$REPO/daemon/adts_daemon" "$REPO/adts_daemon"; do
    [ -x "$c" ] && { BIN="$c"; break; }
done
[ -n "$BIN" ] || { echo "!! adts_daemon 을 못 찾았다. 먼저 빌드할 것:
   cd $REPO/daemon && cmake -S . -B build && cmake --build build" >&2; exit 1; }

say "데몬 설치: $BIN → /opt/adts/adts_daemon"
install -Dm755 "$BIN" /opt/adts/adts_daemon

# ── 2·3. 드라이버 + 오버레이 ────────────────────────────────────────────────
if [ "$DAEMON_ONLY" -eq 0 ]; then
    MODS=""
    for m in turret_driver imu_driver led_sw_driver; do
        [ -f "$REPO/driver/$m.ko" ] && MODS="$MODS $m"
    done

    if [ -n "$MODS" ]; then
        say "커널 모듈:$MODS → /lib/modules/$KREL/extra/"
        for m in $MODS; do
            install -Dm644 "$REPO/driver/$m.ko" "/lib/modules/$KREL/extra/$m.ko"
        done
        depmod -a "$KREL"

        # 부팅 시 자동 적재. 이게 없으면 매번 손으로 insmod 해야 하고,
        # 서비스는 /dev/turret 을 못 찾아 실패한다.
        say "자동 적재 등록: /etc/modules-load.d/adts.conf"
        : > /etc/modules-load.d/adts.conf
        for m in $MODS; do echo "$m" >> /etc/modules-load.d/adts.conf; done
    else
        echo "⚠ driver/*.ko 가 없다 — 모듈 설치를 건너뛴다 (make rpi 로 빌드)"
    fi

    # 오버레이. turret 은 serdev 바인딩이라 **필수**, imu/led_sw 는 해당 하드웨어가
    # 붙어 있을 때만 의미가 있다.
    if [ -n "$BOOTCFG" ]; then
        for d in "$REPO"/driver/overlays/*.dtbo; do
            [ -e "$d" ] || continue
            n="$(basename "$d" .dtbo)"
            say "오버레이: $n → $BOOTOVL/"
            install -Dm644 "$d" "$BOOTOVL/$n.dtbo"
            if grep -q "^dtoverlay=$n\b" "$BOOTCFG"; then
                echo "   config.txt 에 이미 등록됨"
            else
                echo "dtoverlay=$n" >> "$BOOTCFG"
                echo "   config.txt 에 추가 — ⚠️ 적용하려면 재부팅 필요"
            fi
        done
    else
        echo "⚠ config.txt 를 못 찾았다 — 오버레이 등록을 건너뛴다"
    fi
fi

# ── 4. 서비스 ───────────────────────────────────────────────────────────────
say "유닛 설치: /etc/systemd/system/adts-daemon.service"
install -Dm644 "$REPO/daemon/adts-daemon.service" /etc/systemd/system/adts-daemon.service
systemctl daemon-reload

# 인증서를 서비스 사용자가 읽을 수 있는지 미리 본다. 여기서 안 잡으면 데몬이
# 뜬 뒤 mosquitto_tls_set 이 MOSQ_ERR_INVAL 로 실패하는데, 그 에러 메시지가
# "Invalid function arguments" 라 권한 문제로 안 보인다 — 실제로 반나절 걸렸다.
SVC_USER=$(awk -F= '/^User=/{print $2}' /etc/systemd/system/adts-daemon.service)
for f in /etc/adts/certs/ca.crt /etc/adts/certs/daemon.crt /etc/adts/certs/daemon.key; do
    if [ ! -e "$f" ]; then
        echo "⚠ 인증서 없음: $f  (broker/gen-certs.sh 로 발급)"
    elif ! sudo -u "$SVC_USER" test -r "$f"; then
        echo "⚠ $SVC_USER 가 못 읽음: $f"
        echo "   고치기: sudo chgrp $SVC_USER $f && sudo chmod 640 $f"
        echo "   ⚠️ daemon.key 는 절대 644 로 두지 말 것 (개인키)"
    fi
done

if [ "$NO_START" -eq 1 ]; then
    say "설치만 완료 (--no-start). 시작: sudo systemctl enable --now adts-daemon"
else
    say "서비스 활성화·시작"
    systemctl enable --now adts-daemon
    sleep 2
    systemctl --no-pager --lines=20 status adts-daemon || true
fi

cat <<EOF

── 확인 ──────────────────────────────────────────────────────────────────────
  systemctl status adts-daemon
  journalctl -u adts-daemon -f          # 실시간 로그
  systemctl restart adts-daemon

── 주의 ──────────────────────────────────────────────────────────────────────
  · 서비스가 상주하면 /dev/turret 을 **점유**한다. 손으로 스캔을 돌리려면
    먼저  sudo systemctl stop adts-daemon  할 것.
    (scan_batch.sh 도 마찬가지)
  · config.txt 에 dtoverlay 를 새로 넣었으면 재부팅해야 적용된다.
EOF
