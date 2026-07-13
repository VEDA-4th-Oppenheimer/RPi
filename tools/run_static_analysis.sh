#!/usr/bin/env bash
# =====================================================================
# rpi 정적분석 (cppcheck)
#   - 커널 드라이버(driver/) + shared/ 대상
#   - 데몬(daemon/) 코드가 생기면 아래에 추가
#   - 지적사항 있으면 exit 1 (CI 게이트)
# 위치: tools/ (스크립트는 repo 루트에서 동작)
# 사용법:  bash tools/run_static_analysis.sh
# =====================================================================
set -euo pipefail
cd "$(dirname "$0")/.."          # tools/ -> repo 루트

SUPPRESS="tools/cppcheck_suppressions.txt"

echo "==> [driver] cppcheck (turret_driver.c, turret_test.c)"
cppcheck \
  --enable=warning,style,performance,portability \
  --inline-suppr \
  --suppressions-list="${SUPPRESS}" \
  -I shared -I driver \
  --error-exitcode=1 \
  --template="{severity}|{file}:{line}| {message} ({id})" \
  driver/turret_driver.c driver/turret_test.c

# TODO(daemon): daemon/ 코드 추가되면 cmake compile_commands 기반 cppcheck + clang-tidy 추가

echo "==> 정적분석 통과 ✅"
