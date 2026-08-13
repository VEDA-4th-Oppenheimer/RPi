#!/usr/bin/env bash
# ============================================================================
#  scan_batch.sh — 같은 조건으로 스캔을 여러 번 반복해서 돌린다
# ----------------------------------------------------------------------------
#  용도: 반복성 확인(같은 장면을 N 번 찍어 편차 보기), 파라미터 비교,
#        장시간 안정성 시험.
#
#  왜 데몬을 매번 새로 띄우나:
#    --once 데몬은 스캔 1회를 마치면 종료한다. 상주시켜 MQTT 로 연달아
#    트리거할 수도 있지만, 그러면 브로커·인증서가 모두 살아 있어야 하고
#    한 번 어긋났을 때 어디서 멈췄는지 알기 어렵다. 매 회를 독립 프로세스로
#    돌리면 회차마다 로그가 따로 남고, 한 번 실패해도 다음 회차는 깨끗한
#    상태에서 시작한다.
#
#  성공/실패 판정은 **종료코드**로 한다(로그 문구 grep 금지 — 문구가 바뀌면
#  스크립트가 조용히 깨진다). 데몬은 --once 에서
#     0 = 산출까지 완료 / 1 = 스캔이 취소됨(홈 실패·수평 NG 등)
#  을 돌려준다.
# ==========================================================================*/
set -uo pipefail

# ── 기본값 ──────────────────────────────────────────────────────────────────
COUNT=5                 # 반복 횟수
INTERVAL=15             # 회차 사이 대기(초)
TIMEOUT=2400            # 회차당 제한시간(초). 0.9도 전체 스캔이 약 14분
# 데몬 실행파일. 스크립트 위치(<repo>/daemon/tools/)에서 역산해 찾는다 —
# 예전에는 ./adts_daemon 고정이라 레포 안에서 돌리면 못 찾았고, 실기에서는
# 절대경로를 손으로 박아 쓰다가 pull 마다 충돌이 났다. -d 로 덮어쓸 수 있다.
_SB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DAEMON=""
for _c in "$_SB_DIR/../build/adts_daemon" "$_SB_DIR/../adts_daemon" \
          "./adts_daemon" "/opt/adts/adts_daemon"; do
    [ -x "$_c" ] && { DAEMON="$_c"; break; }
done
[ -n "$DAEMON" ] || DAEMON=./adts_daemon      # 못 찾으면 아래에서 에러로 알린다
OUTDIR=""
KEEPGOING=0             # 1 = 실패해도 계속

# 데몬에 넘길 스캔 인자. -- 뒤에 주면 통째로 교체된다.
SCAN_ARGS=(--scan 0 1791 -900 900 9)

usage() {
    cat <<EOF
사용법: $0 [옵션] [-- <데몬 스캔 인자...>]

옵션
  -n <횟수>     반복 횟수            (기본 $COUNT)
  -i <초>       회차 사이 대기        (기본 $INTERVAL)
  -t <초>       회차당 제한시간       (기본 $TIMEOUT)
  -d <경로>     데몬 실행파일         (기본 $DAEMON)
  -o <디렉터리> 로그 저장 위치        (기본 ./batch-<날짜시각>)
  -k            실패해도 계속 진행    (기본: 첫 실패에서 중단)
  -h            이 도움말

-- 뒤의 인자는 데몬에 그대로 전달된다(--once 는 자동으로 붙는다).
기본 스캔 인자: ${SCAN_ARGS[*]}

중간에 멈추는 방법 (두 가지)
  Ctrl-C          지금 당장. 진행 중인 스캔도 끊긴다(그 회차 산출물은 없다).
                  두 번 누르면 데몬이 강제 종료된다.
  touch <로그디렉터리>/STOP
                  **진행 중인 스캔은 끝까지 마치고** 그다음 회차부터 멈춘다.
                  14분짜리 스캔을 버리지 않고 세울 때 쓴다.

예)
  $0 -n 10                              기본 조건으로 10회
  $0 -n 3 -i 60                         3회, 1분 간격
  $0 -n 5 -- --scan 0 1790 -900 900 10 --height 1805
                                        1.0도 격자로 5회
EOF
}

while getopts ":n:i:t:d:o:kh" opt; do
    case "$opt" in
        n) COUNT=$OPTARG ;;
        i) INTERVAL=$OPTARG ;;
        t) TIMEOUT=$OPTARG ;;
        d) DAEMON=$OPTARG ;;
        o) OUTDIR=$OPTARG ;;
        k) KEEPGOING=1 ;;
        h) usage; exit 0 ;;
        \?) echo "알 수 없는 옵션: -$OPTARG" >&2; usage; exit 2 ;;
        :)  echo "-$OPTARG 는 값이 필요하다" >&2; exit 2 ;;
    esac
done
shift $((OPTIND - 1))
[ "${1:-}" = "--" ] && shift
[ $# -gt 0 ] && SCAN_ARGS=("$@")

# --once 가 없으면 붙인다. 없으면 데몬이 안 끝나서 회차가 진행되지 않는다.
case " ${SCAN_ARGS[*]} " in
    *" --once "*) ;;
    *) SCAN_ARGS+=(--once) ;;
esac

[ -x "$DAEMON" ] || { echo "!! 데몬을 실행할 수 없다: $DAEMON" >&2; exit 2; }

# 제한시간 도구. 리눅스(=RPi)는 timeout, macOS 는 coreutils 의 gtimeout 이다.
# 둘 다 없으면 제한시간 없이 돈다 — 스캔이 멈추면 사람이 Ctrl-C 해야 한다.
if command -v timeout  >/dev/null; then TO=(timeout  --signal=INT --kill-after=20 "$TIMEOUT")
elif command -v gtimeout >/dev/null; then TO=(gtimeout --signal=INT --kill-after=20 "$TIMEOUT")
else
    TO=()
    echo "⚠ timeout 명령이 없다 — 회차 제한시간 없이 진행한다" >&2
fi

STAMP=$(date +%Y%m%d-%H%M%S)
[ -n "$OUTDIR" ] || OUTDIR="./batch-$STAMP"
mkdir -p "$OUTDIR" || exit 2

# ── Ctrl-C 처리 ─────────────────────────────────────────────────────────────
#  timeout 이 자식이라 신호는 그쪽으로도 간다. 여기서는 루프만 끊고
#  마지막에 요약이 찍히도록 플래그만 세운다.
ABORT=0
trap 'ABORT=1; echo; echo ">> 중단 요청 — 현재 회차 종료 후 멈춘다"' INT TERM

echo "=============================================================="
echo " 스캔 배치  $COUNT 회"
echo "   데몬 인자 : ${SCAN_ARGS[*]}"
echo "   간격      : ${INTERVAL}s   제한시간: ${TIMEOUT}s/회"
echo "   로그      : $OUTDIR"
echo "   중단      : Ctrl-C (즉시) / touch $OUTDIR/STOP (이번 회차까지)"
echo "=============================================================="

OK=0; NG=0
declare -a RESULTS=()

for ((i = 1; i <= COUNT; i++)); do
    [ "$ABORT" -eq 1 ] && break
    if [ -e "$OUTDIR/STOP" ]; then
        echo ">> STOP 파일 발견 — 여기서 멈춘다"
        rm -f "$OUTDIR/STOP"
        break
    fi

    LOG="$OUTDIR/run-$(printf '%03d' "$i").log"
    echo
    echo "── [$i/$COUNT] 시작  $(date '+%H:%M:%S')  → $LOG"

    T0=$(date +%s)
    # ⚠️ ${TO[@]+...} 형태를 쓴다. set -u 아래에서 **빈 배열을 "${TO[@]}" 로
    #   펴면 bash 3.2(macOS 기본)가 unbound variable 로 죽는다.** 4.4+ 는
    #   괜찮지만 여기서 갈리면 원인을 찾기 어렵다.
    ${TO[@]+"${TO[@]}"} "$DAEMON" "${SCAN_ARGS[@]}" 2>&1 | tee "$LOG"
    RC=${PIPESTATUS[0]}
    T1=$(date +%s)
    ELAPSED=$((T1 - T0))

    # 산출 파일 경로. 로그에서 뽑되 **판정에는 쓰지 않는다**(판정은 RC).
    PCD=$(grep -o '[^ ]*\.pcd' "$LOG" | tail -1)
    CELLS=$(grep -o '유효 [0-9]*셀' "$LOG" | tail -1 | tr -dc '0-9')

    # ⚠️ ABORT 를 먼저 본다. 데몬은 SIGINT 를 받아도 **정상 종료(rc=0)** 하므로,
    #   Ctrl-C 로 끊긴 회차가 rc 만 보면 "완료" 로 집계된다(유효셀 0 짜리 성공).
    if [ "$ABORT" -eq 1 ]; then
        NG=$((NG + 1))
        echo "── [$i/$COUNT] ⛔ 중단됨 (${ELAPSED}s) — 산출물 없음"
        RESULTS+=("$i aborted ${ELAPSED} 0 -")
        break
    fi

    case "$RC" in
        0)  OK=$((OK + 1))
            echo "── [$i/$COUNT] ✅ 완료 (${ELAPSED}s)  유효 ${CELLS:-?}셀  ${PCD:-}"
            RESULTS+=("$i ok ${ELAPSED} ${CELLS:-0} ${PCD:-}") ;;
        124|137)
            NG=$((NG + 1))
            echo "── [$i/$COUNT] ⏱  제한시간 ${TIMEOUT}s 초과 — 강제 종료"
            RESULTS+=("$i timeout ${ELAPSED} 0 -") ;;
        *)  NG=$((NG + 1))
            echo "── [$i/$COUNT] ❌ 실패 (rc=$RC, ${ELAPSED}s) — $LOG 확인"
            RESULTS+=("$i fail(rc=$RC) ${ELAPSED} 0 -") ;;
    esac

    if [ "$RC" -ne 0 ] && [ "$KEEPGOING" -eq 0 ]; then
        echo ">> 실패로 중단한다 (계속하려면 -k)"
        break
    fi

    # 마지막 회차 뒤에는 기다리지 않는다.
    if [ "$i" -lt "$COUNT" ] && [ "$ABORT" -eq 0 ]; then
        # ⚠️ 데몬은 SCAN_DONE 을 받은 뒤 곧바로 끝나지만 STM32 는 그때부터
        #   양축을 홈 자세로 되돌린다. 그게 끝나기 전에 다음 회차가 시작되면
        #   새 스캔이 이동 중인 축을 덮어써서 첫 줄이 어긋난다.
        echo "   ${INTERVAL}s 대기 (STM32 파킹 완료 여유)"
        # 1초씩 쪼개 잔다 — 통째로 sleep 하면 그동안 STOP 파일을 못 보고,
        # Ctrl-C 반응도 sleep 이 끝날 때까지 밀린다.
        for ((w = 0; w < INTERVAL; w++)); do
            [ "$ABORT" -eq 1 ] && break
            [ -e "$OUTDIR/STOP" ] && break
            sleep 1
        done
    fi
done

# ── 요약 ────────────────────────────────────────────────────────────────────
SUMMARY="$OUTDIR/summary.txt"
{
    echo "배치 $STAMP   인자: ${SCAN_ARGS[*]}"
    printf "%-4s %-12s %8s %9s  %s\n" "회차" "결과" "소요(s)" "유효셀" "파일"
    for r in ${RESULTS[@]+"${RESULTS[@]}"}; do
        # shellcheck disable=SC2086
        set -- $r
        printf "%-4s %-12s %8s %9s  %s\n" "$1" "$2" "$3" "$4" "${5:-}"
    done
    echo "성공 $OK / 실패 $NG"
} | tee "$SUMMARY"

echo
echo "요약: $SUMMARY"
[ "$NG" -eq 0 ] && exit 0 || exit 1
