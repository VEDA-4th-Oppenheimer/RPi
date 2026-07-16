# =====================================================================
#  A.D.T.S 통합 데몬 빌드
#   - 네이티브(x86-64, VM 정적분석/스모크):
#       cmake -B build && cmake --build build
#   - RPi 4 (aarch64) 배포용 크로스컴파일:
#       cmake -B build-arm64 -DCMAKE_TOOLCHAIN_FILE=cmake/aarch64-linux-gnu.cmake
#       cmake --build build-arm64
# =====================================================================
cmake_minimum_required(VERSION 3.16)
project(VEDA_ADTS_Daemon LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── 컴파일 전용 플래그 세팅 ──
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror -O2")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fstack-protector-all -D_FORTIFY_SOURCE=2")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIE")

# ── 링커 전용 플래그 세팅 (Clang-Tidy 경고 회피용 분리) ──
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pie -z relro -z now")

# Clang-Tidy 분석을 위한 compile_commands.json 생성 활성화 (핵심)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 공용 헤더 및 데몬 헤더 디렉터리 경로 추가
include_directories(
    ${CMAKE_SOURCE_DIR}/../shared
    ${CMAKE_SOURCE_DIR}/core
    ${CMAKE_SOURCE_DIR}/modules/meta
    ${CMAKE_SOURCE_DIR}/modules/tls
    ${CMAKE_SOURCE_DIR}/modules/tracking
)

# 분석 대상 실행 파일 및 소스 지정
#   core/     = 이현우 (epoll 루프 + 상태머신 + shared_ctx + core_* API)
#   modules/  = 각 담당 (meta:이영민 / tracking:이현우+송영빈 / tls:팀장) — 현재 STUB
add_executable(adts_daemon
    core/main.cpp
    modules/meta/meta_module.cpp
    modules/tracking/tracking_module.cpp
    modules/tls/tls_module.cpp
)
