# =====================================================================
#  발급 서비스 (adts_enroll) 빌드
#   - 언어: C11. 프로젝트 전체가 C/C++ 로 통일돼 있다.
#   - 의존: OpenSSL(TLS) + libcjson(JSON) — 둘 다 데몬이 이미 쓰는 것과 동일.
#       apt install libssl-dev libcjson-dev
#
#   빌드:
#       cmake -S broker -B broker/build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
#       cmake --build broker/build
#
#   ※ 데몬(daemon/)과 별도 타깃이다. 발급 서비스는 브로커 운영 도구라
#     데몬 프로세스와 수명주기가 다르다(발급은 사람이 가끔, 데몬은 상주).
# =====================================================================
cmake_minimum_required(VERSION 3.16)
project(VEDA_ADTS_Enroll LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# ── 컴파일 게이트 (데몬과 동일: 경고=에러) ──
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror -O2")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fstack-protector-all -D_FORTIFY_SOURCE=2")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIE")

# ── 링커 강화 (RELRO/NOW/PIE) ──
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pie -z relro -z now")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(adts_enroll enroll_service.c)

find_package(OpenSSL REQUIRED)
find_library(CJSON_LIB cjson)

if(NOT CJSON_LIB)
    message(FATAL_ERROR
        "libcjson 을 찾지 못했습니다.  apt install libcjson-dev\n"
        "(데몬도 같은 라이브러리를 쓴다)")
endif()

target_link_libraries(adts_enroll OpenSSL::SSL OpenSSL::Crypto ${CJSON_LIB})
