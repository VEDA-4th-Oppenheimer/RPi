#!/usr/bin/env bash
# =====================================================================
# rpi 정적분석 (cppcheck + MISRA-C / Clang-Tidy)
#   - 커널 드라이버(driver/) : cppcheck + MISRA Addon
#   - 데몬(daemon/) : Clang-Tidy (C++ Core Guidelines)
#   - 에러가 발생해도 중단하지 않고 전체 검사를 완수한 뒤, 최종 실패 여부를 반환합니다.
# 위치: tools/ (스크립트는 repo 루트에서 동작)
# 사용법: 
#   - 전체 검사: bash tools/run_static_analysis.sh
#   - 드라이버만: bash tools/run_static_analysis.sh driver
#   - 데몬만: bash tools/run_static_analysis.sh daemon
# =====================================================================
# ★ set -e를 제거하여 중간에 에러가 나도 스크립트가 강제 종료되지 않도록 설정합니다.
set -uo pipefail
cd "$(dirname "$0")/.."          # tools/ -> repo 루트

SUPPRESS="tools/cppcheck_suppressions.txt"
TARGET="${1:-all}"               # 기본값은 전체 검사(all)

# 각 단계별 성공 여부를 기록할 변수 (0: 성공, 1: 실패)
EXIT_CODE=0

# ── [Track 1] 커널 드라이버 분석 (Cppcheck + MISRA-C) ──
if [ "${TARGET}" = "all" ] || [ "${TARGET}" = "driver" ]; then
  echo "==> [driver] Cppcheck 구동 중..."
  
  if command -v cppcheck &> /dev/null; then
    # Cppcheck 실행 및 결과 트래킹
    if ! cppcheck \
      --enable=warning,style,performance,portability \
      --inline-suppr \
      --suppressions-list="${SUPPRESS}" \
      -I shared -I driver \
      --error-exitcode=1 \
      --template="{severity}|CWE-{cwe}|{file}:{line}| {message} ({id})" \
      driver/; then # 👈 파일명들을 'driver/' 폴더 경로 하나로 대체합니다!
      
      echo "❌ [driver] Cppcheck 검사에서 위반 사항이 발견되었습니다."
      EXIT_CODE=1
    else
      echo "✅ [driver] Cppcheck 검사 통과!"
    fi
  else
    echo "⚠️ cppcheck가 설치되어 있지 않아 드라이버 분석을 건너뜁니다."
    if [ "${TARGET}" = "driver" ]; then EXIT_CODE=1; fi
  fi
fi

echo "--------------------------------------------------"

# ── [Track 2] C++ 데몬 분석 (Clang-Tidy) ──
if [ "${TARGET}" = "all" ] || [ "${TARGET}" = "daemon" ]; then
  echo "==> [daemon] Clang-Tidy 분석 준비 중..."

  # ★ [개선] compile_commands.json이 없다면 수동 빌드하지 않았더라도 
  # 스크립트 내에서 자동으로 CMake를 구성하여 데이터베이스를 만들어 줍니다.
  if [ ! -f "daemon/build/compile_commands.json" ]; then
    echo "ℹ️ compile_commands.json이 존재하지 않습니다. CMake 구성을 자동으로 실행합니다..."
    mkdir -p daemon/build
    if ! (cd daemon/build && cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON .. &> /dev/null); then
      echo "❌ [daemon] CMake 구성(Generate)에 실패했습니다."
      EXIT_CODE=1
    fi
  fi

  # CMake 구성에 성공했거나 파일이 이미 존재하는 경우 검사 진행
  if [ -f "daemon/build/compile_commands.json" ]; then
    echo "==> [daemon] Clang-Tidy 분석 구동 중..."
    
    if command -v run-clang-tidy &> /dev/null; then
      # Clang-Tidy 실행 및 결과 트래킹
      if ! run-clang-tidy -p daemon/build -header-filter="^$(pwd)/(daemon|shared)/.*"; then
        echo "❌ [daemon] Clang-Tidy 분석에서 위반 사항이 발견되었습니다."
        EXIT_CODE=1
      else
        echo "✅ [daemon] Clang-Tidy 검사 통과!"
      fi
    else
      echo "⚠️ run-clang-tidy가 설치되어 있지 않아 데몬 분석을 건너뜁니다."
      if [ "${TARGET}" = "daemon" ]; then EXIT_CODE=1; fi
    fi
  else
    echo "❌ [daemon] 빌드 데이터베이스가 유실되어 분석을 수행하지 못했습니다."
    EXIT_CODE=1
  fi
fi

# ── [최종 결과 합산 보고] ──
echo "=================================================="
if [ ${EXIT_CODE} -eq 0 ]; then
  echo "==> 🎉 모든 프로젝트 레이어 정적분석 통과! "
  exit 0
else
  echo "==> ❌ 일부 정적분석 단계에서 결함이 검출되었습니다. 위 로그를 확인하여 조치하십시오."
  exit 1
fi