#!/usr/bin/env bash
# ============================================================================
#  gen-certs.sh — MQTT mTLS 인증서 발급 (브로커 호스트 = RPi 에서 실행)
# ----------------------------------------------------------------------------
#  ★ 반드시 RPi 에서 실행한다. CA 개인키(ca.key)는 이 장비 밖으로 나가지 않는다.
#    클라이언트에게는 인증서와 그 클라이언트의 키만 내보낸다.
#
#  사용 ①  전체 발급 (최초 1회 — CA·브로커·데몬·기본 Qt 인증서를 한꺼번에)
#      bash gen-certs.sh <RPi_IP> [출력디렉터리]
#      예)  sudo bash gen-certs.sh 10.144.31.125 /etc/adts/certs
#
#  사용 ②  클라이언트 1개만 추가 발급 (기존 CA 재사용)
#      bash gen-certs.sh --client <CN> [출력디렉터리]
#      예)  sudo bash gen-certs.sh --client qt-console-youngbin /etc/adts/certs
#
#      발급 서비스(/enroll)가 사람마다 다른 CN 으로 인증서를 내줄 때 쓴다.
#      ★ 발급 후 반드시 /etc/mosquitto/conf.d/adts.acl 에 그 CN 블록을 추가하고
#        브로커를 reload 해야 한다. mosquitto ACL 은 `user <CN>` **정확 매칭**이라
#        와일드카드가 없어서, 빠뜨리면 인증서는 정상인데 구독·발행이 조용히 막힌다.
#        (broker/mosquitto.acl.example 하단 참고)
#
#  만들어지는 것 (①):
#      ca.crt / ca.key            CA (key 는 RPi 에만)
#      server.crt / server.key    브로커용. SAN 에 IP·호스트명 포함
#      daemon.crt / daemon.key    데몬 클라이언트 (CN=adts-daemon)
#      qt-console.crt / .key      Qt 관제 클라이언트 (CN=qt-console)
#      qt-console-trad.key        위 키의 전통 RSA 포맷 ← Qt 는 이걸 써야 한다
#
#  사용 ③  발급 토큰 관리 (관리자용 — 팀원에게 나눠줄 1회용 토큰)
#      bash gen-certs.sh --new-token <라벨>     # 토큰 생성 + 등록 + 전달문 출력
#      bash gen-certs.sh --list-tokens          # 아직 안 쓴 토큰 목록
#      bash gen-certs.sh --revoke <라벨>        # 해당 라벨의 미사용 토큰 회수
#
#      토큰은 발급 서비스(/enroll)가 1회용으로 검증하고, 쓰이는 즉시 파일에서
#      사라진다. 관리자는 만들어서 팀원에게 전달만 하면 된다.
#
#  만들어지는 것 (②):
#      <CN>.crt / <CN>.key / <CN>-trad.key
#
#  ⚠️ Qt(QSslKey)는 OpenSSL 3.x 기본인 PKCS#8 키를 QSsl::Rsa 로 읽으면
#    null 을 반환하고 **조용히** 실패한다. 그래서 변환본을 같이 만든다.
# ============================================================================
set -euo pipefail

DAYS=3650
TOKEN_FILE="${ADTS_TOKEN_FILE:-/etc/adts/enroll_tokens}"

usage() {
    cat >&2 <<'USAGE'
사용:
  bash gen-certs.sh <RPi_IP> [출력디렉터리]        # 전체 발급 (최초 1회)
  bash gen-certs.sh --client <CN> [출력디렉터리]   # 클라이언트 인증서 1개 추가

  bash gen-certs.sh --new-token <라벨>             # 1회용 발급 토큰 생성
  bash gen-certs.sh --list-tokens                  # 미사용 토큰 목록
  bash gen-certs.sh --revoke <라벨>                # 미사용 토큰 회수

  토큰 파일 위치는 ADTS_TOKEN_FILE 로 바꾼다 (기본 /etc/adts/enroll_tokens).
USAGE
    exit 1
}

# 라벨은 CN 접미사가 되고, CN 은 파일명과 ACL 한 줄에 그대로 들어간다.
# 발급 시점이 아니라 **토큰을 만들 때** 걸러야 "토큰은 받았는데 발급이 500" 을 막는다.
check_label() {
    case "${1:-}" in
        *[!A-Za-z0-9._-]* | "" )
            echo "라벨에는 영숫자와 . _ - 만 쓸 수 있습니다: ${1:-<빈값>}" >&2
            exit 1 ;;
    esac
}

# ── 모드 ③-1 : 토큰 생성 ─────────────────────────────────────────────────────
if [ "${1:-}" = "--new-token" ]; then
    LABEL="${2:-}"
    check_label "$LABEL"

    mkdir -p "$(dirname "$TOKEN_FILE")"
    touch "$TOKEN_FILE"
    chmod 600 "$TOKEN_FILE"

    # 같은 라벨의 미사용 토큰이 이미 있으면 알려준다. 여러 개 두는 것 자체는
    # 문제가 없지만(먼저 쓴 것만 유효) 관리자가 헷갈리기 쉽다.
    if grep -qE "^[^#[:space:]]+[[:space:]]+${LABEL}$" "$TOKEN_FILE" 2>/dev/null; then
        echo "⚠️  '${LABEL}' 앞으로 아직 쓰지 않은 토큰이 이미 있습니다." >&2
        echo "    회수하려면: bash gen-certs.sh --revoke ${LABEL}" >&2
    fi

    TOKEN="$(openssl rand -hex 24)"
    printf '%s %s\n' "$TOKEN" "$LABEL" >> "$TOKEN_FILE"

    # 전달문에 넣을 서버 주소 — 이 장비의 첫 번째 IPv4 를 추정한다(틀리면 직접 고쳐 전달).
    # `hostname -I` 는 리눅스 전용이라 없는 환경도 있다. pipefail 이 켜져 있어서
    # 실패를 삼키지 않으면 **토큰만 쓰고 안내문 없이 죽는다**(실제로 겪음).
    IP="$( { hostname -I 2>/dev/null || true; } | awk '{print $1}' )"
    [ -n "$IP" ] || IP="<RPi_IP>"

    cat <<EOF

발급 대상 CN : qt-console-${LABEL}
토큰 파일    : ${TOKEN_FILE}

── 아래를 그대로 팀원에게 전달하십시오 ──────────────────────────
  SPATIAL-VMS 최초 설정에 입력할 값입니다.

    발급 서버 주소 : ${IP}
    포트           : 8443
    토큰           : ${TOKEN}

  앱을 처음 실행하면 등록 창이 뜹니다. 위 값을 넣고 '발급받기'를 누르면
  인증서와 카메라 설정이 자동으로 들어갑니다. 한 번만 하면 됩니다.
──────────────────────────────────────────────────────────────

※ 1회용입니다. 사용되면 자동으로 소멸합니다.
※ 회수: bash gen-certs.sh --revoke ${LABEL}
EOF
    exit 0
fi

# ── 모드 ③-2 : 미사용 토큰 목록 ──────────────────────────────────────────────
if [ "${1:-}" = "--list-tokens" ]; then
    if [ ! -f "$TOKEN_FILE" ]; then
        echo "토큰 파일이 없습니다: $TOKEN_FILE"
        exit 0
    fi
    echo "미사용 토큰 (${TOKEN_FILE})"
    echo "─────────────────────────────────────────────"
    # 토큰 전문은 찍지 않는다 — 화면·로그에 남으면 그 자체가 접근 권한이다.
    n=0
    while read -r tok label; do
        case "$tok" in ''|\#*) continue ;; esac
        [ -n "${label:-}" ] || continue
        printf '  %-20s %s…%s\n' "$label" "${tok:0:6}" "${tok: -4}"
        n=$((n + 1))
    done < "$TOKEN_FILE"
    [ "$n" -eq 0 ] && echo "  (없음)"
    exit 0
fi

# ── 모드 ③-3 : 토큰 회수 ─────────────────────────────────────────────────────
if [ "${1:-}" = "--revoke" ]; then
    LABEL="${2:-}"
    check_label "$LABEL"
    [ -f "$TOKEN_FILE" ] || { echo "토큰 파일이 없습니다: $TOKEN_FILE" >&2; exit 1; }

    BEFORE="$(grep -cE "^[^#[:space:]]+[[:space:]]+${LABEL}$" "$TOKEN_FILE" || true)"
    if [ "$BEFORE" -eq 0 ]; then
        echo "'${LABEL}' 앞으로 미사용 토큰이 없습니다. (이미 사용됐거나 발급된 적 없음)"
        exit 0
    fi

    TMP="$(mktemp)"
    # 남는 줄이 하나도 없으면 grep 이 1 을 반환한다. set -e 아래에서 그대로 두면
    # 파일을 교체하기 전에 죽어 **회수가 안 된 채 조용히 끝난다**(실제로 겪음).
    grep -vE "^[^#[:space:]]+[[:space:]]+${LABEL}$" "$TOKEN_FILE" > "$TMP" || true
    chmod 600 "$TMP"
    mv "$TMP" "$TOKEN_FILE"
    echo "'${LABEL}' 토큰 ${BEFORE}개를 회수했습니다."
    echo "※ 이미 발급받아 간 인증서는 그대로 유효합니다 — 그건 ACL 에서 빼야 합니다:"
    echo "   /etc/mosquitto/conf.d/adts.acl 의 'user qt-console-${LABEL}' 블록 삭제 후"
    echo "   sudo systemctl reload mosquitto"
    exit 0
fi

# ── 모드 ② : 기존 CA 로 클라이언트 인증서 하나만 발급 ────────────────────────
if [ "${1:-}" = "--client" ]; then
    CN="${2:-}"
    OUT="${3:-/etc/adts/certs}"
    [ -n "$CN" ] || usage

    # CN 은 파일명과 ACL 한 줄로 그대로 들어간다. 공백이나 개행이 섞이면 ACL 이
    # 깨지거나 엉뚱한 파일을 덮어쓸 수 있으므로 문자 집합을 좁힌다.
    case "$CN" in
        *[!A-Za-z0-9._-]* | "" ) echo "CN 에는 영숫자와 . _ - 만 쓸 수 있습니다: $CN" >&2; exit 1 ;;
    esac

    cd "$OUT" 2>/dev/null || { echo "출력 디렉터리가 없습니다: $OUT" >&2; exit 1; }
    [ -f ca.crt ] && [ -f ca.key ] || {
        echo "$OUT 에 ca.crt/ca.key 가 없습니다. 먼저 전체 발급을 실행하십시오." >&2; exit 1; }
    [ -e "$CN.crt" ] && { echo "이미 존재합니다: $OUT/$CN.crt (지우고 다시 실행)" >&2; exit 1; }

    # 클라이언트 인증서는 SAN 이 필요 없다 — v3_client 만 있으면 된다.
    TMP_CNF="$(mktemp)"
    trap 'rm -f "$TMP_CNF"' EXIT
    cat > "$TMP_CNF" <<'EOF'
[req]
distinguished_name = dn
[dn]
[v3_client]
basicConstraints = CA:FALSE
keyUsage         = digitalSignature,keyEncipherment
extendedKeyUsage = clientAuth
EOF

    openssl req -newkey rsa:2048 -nodes -keyout "$CN.key" -out "$CN.csr" \
        -subj "/CN=$CN" -config "$TMP_CNF" 2>/dev/null
    openssl x509 -req -in "$CN.csr" -CA ca.crt -CAkey ca.key -CAcreateserial \
        -days "$DAYS" -out "$CN.crt" -extfile "$TMP_CNF" -extensions v3_client 2>/dev/null
    rm -f "$CN.csr" ca.srl

    # Qt 용 전통 RSA 포맷 — 발급 서비스는 이 파일 내용을 client_key 로 내려준다.
    openssl rsa -traditional -in "$CN.key" -out "$CN-trad.key" 2>/dev/null

    chmod 600 "$CN.key" "$CN-trad.key"
    chmod 644 "$CN.crt"

    cat <<EOF
발급 완료 (CN=$CN)
  $OUT/$CN.crt
  $OUT/$CN-trad.key    ← Qt 에 내려줄 키 (전통 RSA 포맷)

다음을 반드시 수행할 것:
  1) /etc/mosquitto/conf.d/adts.acl 에 아래 블록 추가

       user $CN
       topic write adts/cmd/#
       topic read  adts/state/#
       topic read  adts/event/#

  2) sudo systemctl reload mosquitto
EOF
    exit 0
fi

# ── 모드 ① : 전체 발급 ───────────────────────────────────────────────────────
HOST_IP="${1:-}"
OUT="${2:-./certs}"

[ -n "$HOST_IP" ] || usage

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
