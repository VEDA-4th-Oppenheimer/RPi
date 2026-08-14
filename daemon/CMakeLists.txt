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
    ${CMAKE_SOURCE_DIR}/modules/camera
)

# 실행 파일 및 소스
#   core/     = 이현우 (epoll 루프 + FSM + shared_ctx
#                       + (pan,tilt,d)->(x,y,z) 변환 + 포인트클라우드 .pcd)
#   modules/  = mqtt:이현우+이광진 / imu:송영빈 / led:이현우
#               camera:이현우(송신) + 이영민(수신 CV5 앱)
add_executable(adts_daemon
    core/main.c
    core/scan_output.c
    modules/mqtt/mqtt_module.c
    modules/imu/imu_module.c
    modules/led/led_module.c
    modules/camera/camera_module.c
)

# 좌표 변환에 libm (cos/sin) 필요
# ── 외부 라이브러리 ────────────────────────────────────────────────────────
#  libmosquitto : MQTT 클라이언트 (mqtt_module, epoll 통합)
#  libcjson     : 명령/상태 페이로드 파싱·생성
#  둘 다 Debian/Ubuntu 패키지: apt install libmosquitto-dev libcjson-dev
#
#  ⚠️ 두 라이브러리는 **선택**이다. 없으면 MQTT 를 컴파일아웃하고 계속 빌드한다.
#    맥 크로스 툴체인 sysroot 에는 aarch64 용 패키지가 없어서, REQUIRED 로 두면
#    CLion 이 configure 단계에서 실패해 인덱싱조차 안 된다. 실제 배포 바이너리는
#    Docker(대상과 같은 glibc) 에서 굽고, 크로스 경로는 편집·인덱싱용이다.
find_library(MOSQUITTO_LIB mosquitto)
find_library(CJSON_LIB     cjson)

if(MOSQUITTO_LIB AND CJSON_LIB)
    target_link_libraries(adts_daemon m ${MOSQUITTO_LIB} ${CJSON_LIB})
    message(STATUS "MQTT: 활성 (${MOSQUITTO_LIB} / ${CJSON_LIB})")
else()
    target_link_libraries(adts_daemon m)
    target_compile_definitions(adts_daemon PRIVATE ADTS_NO_MQTT=1)
    message(WARNING
        "libmosquitto/libcjson 를 찾지 못해 MQTT 를 비활성으로 빌드합니다.\n"
        "  이 바이너리로는 Qt 관제 연동이 되지 않습니다(CLI --scan 은 정상).\n"
        "  배포용은 Docker 에서 빌드하십시오:  apt install libmosquitto-dev libcjson-dev")
endif()
