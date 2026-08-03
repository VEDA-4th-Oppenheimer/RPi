# =====================================================================
#  ADTS 통합 데몬 빌드 (C11) — 1D LiDAR 스캐너 / 자동 캘리브레이션 킷
#   - 언어: C  (epoll/kernel 경계는 C. 커널드라이버(C)·STM32(C)와 한 멘탈모델)
#   - 정적분석: cppcheck (compile_commands.json 기반) + 컴파일러 -Werror 게이트
#
#   빌드:
#       cmake -S . -B build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
#       cmake --build build
#   ※ epoll/timerfd/signalfd 는 리눅스 전용 → macOS 네이티브 빌드 불가.
#     Docker 컨테이너(adts) 안에서 빌드한다. M4 는 arm64 라 RPi 와 동일 아키텍처
#     (크로스컴파일 불필요, 네이티브 빌드).
# =====================================================================
cmake_minimum_required(VERSION 3.16)
project(VEDA_ADTS_Daemon LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# ── 컴파일 게이트 (방산급: 경고=에러) ──
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror -O2")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fstack-protector-all -D_FORTIFY_SOURCE=2")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIE")

# ── 링커 강화 (RELRO/NOW/PIE) ──
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pie -z relro -z now")

# cppcheck 분석을 위한 compile_commands.json 생성 (핵심)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 공용 계약 헤더 + 데몬 헤더 경로
include_directories(
    ${CMAKE_SOURCE_DIR}/../shared
    ${CMAKE_SOURCE_DIR}/core
    ${CMAKE_SOURCE_DIR}/modules/mqtt
    ${CMAKE_SOURCE_DIR}/modules/imu
    ${CMAKE_SOURCE_DIR}/modules/led
)

# 실행 파일 및 소스
#   core/     = 이현우 (epoll 루프 + FSM + shared_ctx
#                       + (pan,tilt,d)->(x,y,z) 변환 + 포인트클라우드 .pcd)
#   modules/  = mqtt:이현우+이광진 / imu:송영빈 / led:이현우  — 현재 STUB
add_executable(adts_daemon
    core/main.c
    modules/mqtt/mqtt_module.c
    modules/imu/imu_module.c
    modules/led/led_module.c
)

# 좌표 변환에 libm (cos/sin) 필요
target_link_libraries(adts_daemon m)
