#!/usr/bin/env bash
# ============================================================================
#  gen-certs.sh — MQTT mTLS 인증서 발급 (브로커 호스트 = RPi 에서 실행)
# ----------------------------------------------------------------------------
#  ★ 반드시 RPi 에서 실행한다. CA 개인키(ca.key)는 이 장비 밖으로 나가지 않는다.
#    클라이언트에게는 인증서와 그 클라이언트의 키만 내보낸다.
#
#  사용:
#      bash gen-certs.sh <RPi_IP> [출력디렉터리]
#      예)  sudo bash gen-certs.sh 10.144.31.125 /etc/adts/certs
#
#  만들어지는 것:
#      ca.crt / ca.key            CA (key 는 RPi 에만)
#      server.crt / server.key    브로커용. SAN 에 IP·호스트명 포함
#      daemon.crt / daemon.key    데몬 클라이언트 (CN=adts-daemon)
#      qt-console.crt / .key      Qt 관제 클라이언트 (CN=qt-console)
#      qt-console-trad.key        위 키의 전통 RSA 포맷 ← Qt 는 이걸 써야 한다
#
#  ⚠️ Qt(QSslKey)는 OpenSSL 3.x 기본인 PKCS#8 키를 QSsl::Rsa 로 읽으면
#    null 을 반환하고 **조용히** 실패한다. 그래서 변환본을 같이 만든다.
# ============================================================================
set -euo pipefail

HOST_IP="${1:-}"
OUT="${2:-./certs}"
DAYS=3650

if [ -z "$HOST_IP" ]; then
    echo "사용: bash gen-certs.sh <RPi_IP> [출력디렉터리]" >&2
    echo "예)  sudo bash gen-certs.sh 10.144.31.125 /etc/adts/certs" >&2
    exit 1
fi

HOSTNAME_S="$(hostname)"
mkdir -p "$OUT" && cd "$OUT"

# SAN — IP 로 접속하면 인증서에 IP SAN 이 없으면 검증이 깨진다.
# 호스트명·localhost 도 넣어 데몬(로컬)과 Qt(원격) 양쪽을 커버한다.
cat > san.cnf <<EOF
[req]
distinguished_name = dn
[dn]
[v3_ca]
basicConstraints = critical,CA:TRUE
keyUsage         = critical,keyCertSign,cRLSign
[v3_server]
basicConstraints = CA:FALSE
keyUsage         = digitalSignature,keyEncipherment
extendedKeyUsage = serverAuth
subjectAltName   = @alt
[v3_client]
basicConstraints = CA:FALSE
keyUsage         = digitalSignature,keyEncipherment
extendedKeyUsage = clientAuth
[alt]
IP.1  = ${HOST_IP}
IP.2  = 127.0.0.1
DNS.1 = ${HOSTNAME_S}
DNS.2 = ${HOSTNAME_S}.local
DNS.3 = localhost
EOF

echo "== CA =="
openssl req -x509 -newkey rsa:2048 -nodes -days "$DAYS" \
    -keyout ca.key -out ca.crt -subj "/CN=ADTS-CA" \
    -extensions v3_ca -config san.cnf 2>/dev/null

issue() {   # issue <이름> <CN> <확장>
    openssl req -newkey rsa:2048 -nodes -keyout "$1.key" -out "$1.csr" \
        -subj "/CN=$2" -config san.cnf 2>/dev/null
    openssl x509 -req -in "$1.csr" -CA ca.crt -CAkey ca.key -CAcreateserial \
        -days "$DAYS" -out "$1.crt" -extfile san.cnf -extensions "$3" 2>/dev/null
    rm -f "$1.csr"
    echo "   $1.crt  (CN=$2)"
}

echo "== 브로커 =="   ; issue server     "$HOSTNAME_S" v3_server
echo "== 데몬 =="     ; issue daemon     adts-daemon  v3_client
echo "== Qt 관제 ==" ; issue qt-console qt-console   v3_client

# Qt 용 전통 RSA 포맷 (PKCS#8 이면 QSslKey 가 조용히 실패)
# OpenSSL 3.x 는 `openssl rsa` 도 기본 출력이 PKCS#8 이라 -traditional 이 필요하다.
openssl rsa -traditional -in qt-console.key -out qt-console-trad.key 2>/dev/null
echo "   qt-console-trad.key  (Qt 용 전통 RSA 포맷)"

chmod 600 ./*.key
chmod 644 ./*.crt

# 브로커는 root 로 떠도 곧바로 `mosquitto` 사용자로 권한을 낮춘다. 그래서
# server.key 가 600/root 이면 "Unable to load server key file — Permission denied"
# 로 TLS 리스너가 아예 안 뜬다(실제로 겪음). 그 계정이 있으면 읽게 해준다.
if id mosquitto >/dev/null 2>&1; then
    chown mosquitto:mosquitto ca.crt server.crt server.key 2>/dev/null || true
    chmod 640 server.key
    echo "   (server.key 를 mosquitto 사용자가 읽도록 조정)"
fi
rm -f ca.srl san.cnf

cat <<EOF

──────────────────────────────────────────────────────────────
발급 완료: $(pwd)

[RPi 에 남길 것]
  ca.crt ca.key server.crt server.key daemon.crt daemon.key

[Mac 으로 가져갈 것 — 3개만]
  ca.crt  qt-console.crt  qt-console-trad.key

  Mac 에서:
    mkdir -p ~/adts-certs
    scp pi@${HOST_IP}:$(pwd)/{ca.crt,qt-console.crt,qt-console-trad.key} ~/adts-certs/

⚠️ ca.key 는 절대 내보내지 말 것. 이 장비에만 있어야 한다.
⚠️ 유효기간 ${DAYS}일. RPi 는 RTC 가 없어 인터넷 없이 부팅하면 시계가
   틀어져 "not yet valid" 로 거부될 수 있다 (fake-hwclock 확인).
──────────────────────────────────────────────────────────────
EOF
