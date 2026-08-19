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
#  주의: 2·3 이 없으면 부팅 시 /dev/turret 이 없어서 서비스가 실패한다.
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

# 주의: 상주 서비스가 /dev/turret 을 쥐고 있으면 모듈을 못 내린다. 먼저 멈춘다.
#   (마지막에 다시 켠다 — --no-start 면 안 켠다)
if systemctl is-active --quiet adts-daemon 2>/dev/null; then
    say "실행 중인 adts-daemon 정지 (모듈 교체를 위해)"
    systemctl stop adts-daemon
fi

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

        # 핵심: 지금 커널에도 반영한다.
        #
        #   modules-load.d 는 **부팅 때만** 읽힌다. 복사만 하고 끝내면
        #   지금 돌고 있는 것은 여전히 옛 모듈이라, /dev/turret 의 권한이나
        #   동작이 새 .ko 와 다른 채로 남는다. 실제로 이것 때문에 데몬이
        #   "open /dev/turret 실패 (Permission denied)" 로 degraded 로 떴다.
        #
        #   내리고 다시 올린다. 사용 중이라 못 내리면 재부팅을 안내한다 —
        #   조용히 넘어가면 "설치했는데 왜 그대로지" 가 된다.
        say "모듈 적재 갱신"
        NEED_REBOOT=0
        for m in $MODS; do
            if lsmod | awk '{print $1}' | grep -qx "$m"; then
                if rmmod "$m" 2>/dev/null; then
                    echo "   $m: 옛 모듈 내림"
                else
                    echo "   주의: $m: 사용 중이라 못 내림 — 새 .ko 는 재부팅 후 적용"
                    NEED_REBOOT=1
                    continue
                fi
            fi
            if modprobe "$m" 2>/dev/null; then
                echo "   $m: 적재됨"
            else
                echo "   주의: $m: 적재 실패 — dmesg 확인"
                NEED_REBOOT=1
            fi
        done
    else
        echo "주의: driver/*.ko 가 없다 — 모듈 설치를 건너뛴다 (make rpi 로 빌드)"
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
                echo "   config.txt 에 추가 — 주의: 적용하려면 재부팅 필요"
            fi
        done
    else
        echo "주의: config.txt 를 못 찾았다 — 오버레이 등록을 건너뛴다"
    fi
fi

# ── 4. 서비스 ───────────────────────────────────────────────────────────────
say "유닛 설치: /etc/systemd/system/adts-daemon.service"
install -Dm644 "$REPO/daemon/adts-daemon.service" /etc/systemd/system/adts-daemon.service
systemctl daemon-reload

# 카메라 접속 설정. 주의: **덮어쓰지 않는다** — 현장에서 사람이 고쳐 넣은 IP 가
# 들어있는 파일이라, 배포할 때마다 예제값으로 되돌리면 그날 스캔이 다 실패한다.
if [ ! -e /etc/adts/camera.conf ]; then
    install -Dm644 "$REPO/daemon/camera.conf.example" /etc/adts/camera.conf
    echo "   /etc/adts/camera.conf 생성 — 주의: host 를 실제 카메라 IP 로 고칠 것"
else
    echo "   /etc/adts/camera.conf 유지 (host = $(awk -F= '/^[[:space:]]*host/{gsub(/ /,"",$2); print $2}' /etc/adts/camera.conf))"
fi

# 인증서를 서비스 사용자가 읽을 수 있는지 미리 본다. 여기서 안 잡으면 데몬이
# 뜬 뒤 mosquitto_tls_set 이 MOSQ_ERR_INVAL 로 실패하는데, 그 에러 메시지가
# "Invalid function arguments" 라 권한 문제로 안 보인다 — 실제로 반나절 걸렸다.
SVC_USER=$(awk -F= '/^User=/{print $2}' /etc/systemd/system/adts-daemon.service)
for f in /etc/adts/certs/ca.crt /etc/adts/certs/daemon.crt /etc/adts/certs/daemon.key; do
    if [ ! -e "$f" ]; then
        echo "주의: 인증서 없음: $f  (broker/gen-certs.sh 로 발급)"
    elif ! sudo -u "$SVC_USER" test -r "$f"; then
        echo "주의: $SVC_USER 가 못 읽음: $f"
        echo "   고치기: sudo chgrp $SVC_USER $f && sudo chmod 640 $f"
        echo "   주의: daemon.key 는 절대 644 로 두지 말 것 (개인키)"
    fi
done

# 유닛의 ExecStartPre 가 /dev/turret 을 기다리다 실패하는 일이 잦아, 켜기 전에
# 미리 보여준다. 없으면 오버레이(dtoverlay=turret-overlay) 미적용이 1순위다.
for d in /dev/turret /dev/imu /dev/led_sw; do
    if [ -e "$d" ]; then
        echo "   $(ls -l "$d")"
    else
        echo "   주의: $d 없음 — 오버레이 적용/모듈 probe 확인 (dmesg)"
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

if [ "${NEED_REBOOT:-0}" -eq 1 ]; then
    echo
    echo "주의: 일부 모듈을 교체하지 못했다. 재부팅해야 새 .ko 가 적용된다: sudo reboot"
fi
