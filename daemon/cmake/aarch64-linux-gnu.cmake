# =====================================================================
# CMake 툴체인 파일 — RPi 4 (aarch64) 크로스컴파일
#
# 용법 (daemon/ 에서):
#   cmake -B build-arm64 -DCMAKE_TOOLCHAIN_FILE=cmake/aarch64-linux-gnu.cmake
#   cmake --build build-arm64
#
# 산출물 build-arm64/adts_daemon 은 ELF aarch64 → RPi 4 에 scp 배포.
# 네이티브(x86-64) 빌드는 이 파일 없이 그냥 `cmake -B build ..` (VM 정적분석/스모크용).
#
# ⚠️ 지금은 외부 라이브러리 링크가 없어 sysroot 불필요.
#    tls_module 에 OpenSSL 이 들어가면 aarch64용 libssl-dev 가 있는
#    sysroot 를 CMAKE_SYSROOT 로 지정해야 한다(아래 주석 해제).
# =====================================================================
set(CMAKE_SYSTEM_NAME      Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(CMAKE_C_COMPILER   aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

# OpenSSL 등 타겟 라이브러리 링크 시 sysroot 지정(예):
# set(CMAKE_SYSROOT /opt/rpi-sysroot)

# 프로그램은 호스트에서, 라이브러리/헤더/패키지는 타겟 sysroot 에서만 탐색
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
