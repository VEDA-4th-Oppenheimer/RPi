#!/usr/bin/env bash
# ============================================================================
#  publish-config.sh — 카메라 설정을 브로커에 retained 로 발행 (RPi 에서 실행)
# ----------------------------------------------------------------------------
#  카메라는 사용자별 자산이 아니라 킷의 일부다. 관리자가 여기서 한 번 바꾸면
#  접속 중인 Qt 콘솔 전부에 즉시 반영된다.
#
#  사용:
#      sudo bash publish-config.sh [카메라설정파일]
#      기본값 /etc/adts/cameras.json
#
#  왜 retained 인가:
#    나중에 켜는 콘솔도 접속하자마자 현재 값을 받아야 한다. retained 가 아니면
#    발행 시점에 붙어 있던 콘솔만 받고, 늦게 켠 사람은 영원히 못 받는다.
#
#  발급(/enroll) 응답에도 cameras 가 들어가지만 그건 **브로커 연결 전 초기값**
#  이다. 등록 시점에 한 번 박히고 끝이라 나중에 카메라가 바뀌면 전달할 방법이
#  없다. 그래서 실제 기준은 이 토픽이다.
#
#  ⚠️ 이 스크립트를 돌린 뒤 파일을 또 고쳤다면 다시 돌려야 한다. 자동 감시는
#     하지 않는다 — 설정 변경은 사람이 하는 드문 작업이라 명시적인 편이 낫다.
# ============================================================================
set -euo pipefail

SRC="${1:-/etc/adts/cameras.json}"
CERT_DIR="${ADTS_CERT_DIR:-/etc/adts/certs}"
HOST="${ADTS_MQTT_HOST:-127.0.0.1}"
PORT="${ADTS_MQTT_PORT:-8883}"
TOPIC="adts/config/cameras"

[ -f "$SRC" ] || { echo "카메라 설정 파일이 없습니다: $SRC" >&2; exit 1; }

# 형식이 깨진 채로 발행하면 콘솔들이 조용히 무시한다. 여기서 먼저 막는다.
if command -v python3 >/dev/null 2>&1; then
    python3 - "$SRC" <<'PY' || exit 1
import json, sys
try:
    d = json.load(open(sys.argv[1]))
except Exception as e:
    print(f"JSON 을 해석하지 못했습니다: {e}", file=sys.stderr); sys.exit(1)
ch = d.get("channels")
if not isinstance(ch, dict) or not ch:
    print("channels 객체가 없거나 비어 있습니다.", file=sys.stderr); sys.exit(1)
bad = [k for k in ch if not k.isdigit() or not (1 <= int(k) <= 4)]
if bad:
    print(f"채널 번호는 1~4 여야 합니다: {bad}", file=sys.stderr); sys.exit(1)
print(f"검증 통과 — 채널 {len(ch)}개")
PY
else
    echo "⚠️ python3 이 없어 JSON 검증을 건너뜁니다."
fi

# 발행 주체는 데몬 인증서(CN=adts-daemon)를 쓴다. ACL 에서 adts-daemon 에게
# adts/config/# 쓰기를 허용해 두었다(mosquitto.acl.example 참고).
mosquitto_pub -h "$HOST" -p "$PORT" \
    --cafile "$CERT_DIR/ca.crt" \
    --cert   "$CERT_DIR/daemon.crt" \
    --key    "$CERT_DIR/daemon.key" \
    -t "$TOPIC" -r -f "$SRC" -q 1 -i adts-config-pub

echo "발행 완료: $TOPIC (retained) ← $SRC"
echo "확인:"
echo "  mosquitto_sub -h $HOST -p $PORT --cafile $CERT_DIR/ca.crt \\"
echo "    --cert $CERT_DIR/daemon.crt --key $CERT_DIR/daemon.key \\"
echo "    -t $TOPIC -v -W 3 -i config-check"
