#!/usr/bin/env bash
# =====================================================================
# rpi 정적분석 (cppcheck 단일 도구)
#   - 커널 드라이버(driver/) : cppcheck (커널 관용구는 억제목록으로 제외)
#   - 데몬(daemon/)          : cppcheck (compile_commands.json 기반)
#                              ※ C 데몬. 실제 컴파일 -Werror 게이트는 CMake 빌드가 담당.
#   - MISRA/clang-tidy 미사용:
#       · 드라이버=커널코드라 MISRA 부적합(checkpatch/sparse 영역) → 제거(§14)
#       · 데몬=전면 C 전환 → clang-tidy(C++ Core Guidelines) 대신 cppcheck 통일
#   - 에러가 나도 중단하지 않고 전체 검사를 완수한 뒤 최종 실패 여부를 반환.
# 위치: tools/ (스크립트는 repo 루트에서 동작)
# 사용법:
#   - 전체:     bash tools/run_static_analysis.sh
#   - 드라이버: bash tools/run_static_analysis.sh driver
#   - 데몬:     bash tools/run_static_analysis.sh daemon
#   - 발급서비스: bash tools/run_static_analysis.sh broker
# =====================================================================
set -uo pipefail
cd "$(dirname "$0")/.."          # tools/ -> repo 루트

SUPPRESS="tools/cppcheck_suppressions.txt"
TARGET="${1:-all}"               # 기본값은 전체(all)
EXIT_CODE=0

CPPCHECK_COMMON=(
  --enable=warning,style,performance,portability
  --inline-suppr
  --suppressions-list="${SUPPRESS}"
  --error-exitcode=1
  --template="{severity}|CWE-{cwe}|{file}:{line}| {message} ({id})"
)

# ── [Track 1] 커널 드라이버 분석 (cppcheck) ──
if [ "${TARGET}" = "all" ] || [ "${TARGET}" = "driver" ]; then
  echo "==> [driver] cppcheck 구동 중..."
  if command -v cppcheck &> /dev/null; then
    if ! cppcheck "${CPPCHECK_COMMON[@]}" -I shared -I driver driver/; then
      echo "❌ [driver] cppcheck 위반 발견"
      EXIT_CODE=1
    else
      echo "✅ [driver] cppcheck 통과!"
    fi
  else
    echo "⚠️ cppcheck 미설치 — 드라이버 분석 건너뜀"
    if [ "${TARGET}" = "driver" ]; then EXIT_CODE=1; fi
  fi
fi

echo "--------------------------------------------------"

# ── [Track 2] C 데몬 분석 (cppcheck, compile_commands.json 기반) ──
if [ "${TARGET}" = "all" ] || [ "${TARGET}" = "daemon" ]; then
  echo "==> [daemon] cppcheck 분석 준비 중..."

  # compile_commands.json 이 없으면 CMake 구성으로 자동 생성
  if [ ! -f "daemon/build/compile_commands.json" ]; then
    echo "ℹ️ compile_commands.json 없음 — CMake 구성 자동 실행..."
    if ! cmake -S daemon -B daemon/build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON &> /dev/null; then
      echo "❌ [daemon] CMake 구성(Generate) 실패"
      EXIT_CODE=1
    fi
  fi

  if [ -f "daemon/build/compile_commands.json" ]; then
    if command -v cppcheck &> /dev/null; then
      echo "==> [daemon] cppcheck 분석 구동 중..."
      if ! cppcheck "${CPPCHECK_COMMON[@]}" \
        --project=daemon/build/compile_commands.json; then
        echo "❌ [daemon] cppcheck 위반 발견"
        EXIT_CODE=1
      else
        echo "✅ [daemon] cppcheck 통과!"
      fi
    else
      echo "⚠️ cppcheck 미설치 — 데몬 분석 건너뜀"
      if [ "${TARGET}" = "daemon" ]; then EXIT_CODE=1; fi
    fi
  else
    echo "❌ [daemon] 컴파일 DB(compile_commands.json) 유실 — 분석 불가"
    EXIT_CODE=1
  fi
fi

echo "--------------------------------------------------"

# ── [Track 3] 발급 서비스 분석 (cppcheck, compile_commands.json 기반) ──
#   데몬과 같은 방식이지만 별도 CMake 프로젝트다(수명주기가 달라 타깃을 나눴다).
if [ "${TARGET}" = "all" ] || [ "${TARGET}" = "broker" ]; then
  echo "==> [broker] cppcheck 분석 준비 중..."

  if [ ! -f "broker/build/compile_commands.json" ]; then
    echo "ℹ️ compile_commands.json 없음 — CMake 구성 자동 실행..."
    if ! cmake -S broker -B broker/build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON &> /dev/null; then
      echo "❌ [broker] CMake 구성(Generate) 실패 — libssl-dev/libcjson-dev 설치 확인"
      EXIT_CODE=1
    fi
  fi

  if [ -f "broker/build/compile_commands.json" ]; then
    if command -v cppcheck &> /dev/null; then
      echo "==> [broker] cppcheck 분석 구동 중..."
      if ! cppcheck "${CPPCHECK_COMMON[@]}" \
        --project=broker/build/compile_commands.json; then
        echo "❌ [broker] cppcheck 위반 발견"
        EXIT_CODE=1
      else
        echo "✅ [broker] cppcheck 통과!"
      fi
    else
      echo "⚠️ cppcheck 미설치 — 발급 서비스 분석 건너뜀"
      if [ "${TARGET}" = "broker" ]; then EXIT_CODE=1; fi
    fi
  else
    echo "❌ [broker] 컴파일 DB(compile_commands.json) 유실 — 분석 불가"
    EXIT_CODE=1
  fi
fi

# ── [최종 보고] ──
echo "=================================================="
if [ ${EXIT_CODE} -eq 0 ]; then
  echo "==> 🎉 모든 레이어 정적분석 통과!"
  exit 0
else
  echo "==> ❌ 일부 정적분석 단계에서 결함 검출. 위 로그 확인 후 조치하십시오."
  exit 1
fi
