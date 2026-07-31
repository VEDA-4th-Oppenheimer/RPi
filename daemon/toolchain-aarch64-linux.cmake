# =====================================================================
#  CMake 툴체인 — macOS 호스트에서 RPi(aarch64 Linux) 용 데몬 크로스컴파일
#
#  Docker 없이 CLion(또는 CLI)에서 데몬만 빌드할 때 사용한다.
#
#  [CLion]
#    Settings ▸ Build, Execution, Deployment ▸ CMake ▸ (프로필 선택)
#      CMake options 에 아래 한 줄 추가:
#        -DCMAKE_TOOLCHAIN_FILE=${CMAKE_SOURCE_DIR}/toolchain-aarch64-linux.cmake
#      Toolchain 은 기본(Default/시스템) 그대로 둔다 — 여기서 컴파일러를 바꾸므로.
#
#  [CLI]
#      cmake -S . -B build-cross \
#            -DCMAKE_TOOLCHAIN_FILE=toolchain-aarch64-linux.cmake \
#            -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
#      cmake --build build-cross
#
#  왜 크로스가 되나:
#    데몬은 epoll/timerfd/signalfd 같은 리눅스 전용 API 를 쓰는데, 이 툴체인은
#    sysroot 에 glibc + 리눅스 헤더를 함께 제공하므로 맥에서도 전부 해석된다.
#    (macOS SDK 만으로는 불가 → 네이티브 빌드는 여전히 안 됨)
#
#  ⚠️ 빌드만 되고 **실행·디버그는 맥에서 불가** (aarch64 리눅스 ELF).
#    실행은 RPi 로 scp 후. CLion 에서도 Run 버튼은 쓰지 말 것.
#
#  ⚠️ glibc 호환 — 툴체인 glibc 가 RPi(Bookworm=2.36)보다 새로우면 실행 시
#    "GLIBC_2.xx not found". 배포 전 확인:
#        aarch64-linux-gnu-readelf -V <바이너리> | grep -oE 'GLIBC_[0-9.]+' | sort -u
#    최대값이 2.36 이하여야 안전.
#
#  Docker(adts) 경로와의 관계:
#    - Docker  = arm64 컨테이너 안에서 **네이티브** 빌드. cppcheck·커널 드라이버
#                까지 한 환경에서 처리.
#    - 이 파일 = 맥에서 **크로스** 빌드. 컨테이너 기동 불필요, 데몬 전용.
#    둘 다 동일한 aarch64 ELF 를 만든다.
# =====================================================================

set(CMAKE_SYSTEM_NAME      Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# 툴체인 접두사 자동 탐색.
#   Homebrew(messense/macos-cross-toolchains) 는 aarch64-unknown-linux-gnu- 로
#   설치되며 aarch64-linux-gnu- 심볼릭 링크도 함께 만든다. 배포판에 따라 후자만
#   있는 경우도 있어 둘 다 시도한다. -DCROSS_PREFIX=... 로 강제 지정 가능.
if(NOT DEFINED CROSS_PREFIX)
    foreach(_p "aarch64-unknown-linux-gnu-" "aarch64-linux-gnu-")
        find_program(_cc "${_p}gcc")
        if(_cc)
            set(CROSS_PREFIX "${_p}")
            break()
        endif()
        unset(_cc CACHE)
    endforeach()
endif()

find_program(CROSS_CC "${CROSS_PREFIX}gcc")
if(NOT CROSS_CC)
    message(FATAL_ERROR
        "aarch64 리눅스 크로스 컴파일러를 찾을 수 없습니다.\n"
        "시도한 접두사: aarch64-unknown-linux-gnu- / aarch64-linux-gnu-\n"
        "설치돼 있는데 못 찾으면 -DCROSS_PREFIX=<접두사> 로 지정하세요.")
endif()

set(CMAKE_C_COMPILER "${CROSS_CC}")

# sysroot 는 컴파일러가 알려주는 값을 그대로 쓴다 (Cellar 경로 하드코딩 회피 —
# Homebrew 업그레이드로 버전 디렉터리가 바뀌어도 깨지지 않는다).
execute_process(
    COMMAND "${CROSS_CC}" -print-sysroot
    OUTPUT_VARIABLE CROSS_SYSROOT
    OUTPUT_STRIP_TRAILING_WHITESPACE)
if(CROSS_SYSROOT AND EXISTS "${CROSS_SYSROOT}")
    set(CMAKE_SYSROOT "${CROSS_SYSROOT}")
endif()

# 호스트(macOS) 실행파일은 그대로 쓰되, 헤더·라이브러리는 sysroot 안에서만 찾는다.
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM BEFORE)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# ---------------------------------------------------------------------------
#  libssp 동적 의존 제거
#
#  증상: RPi 에서 실행하면
#      error while loading shared libraries: libssp.so.0: cannot open ...
#
#  원인: 우리는 -fstack-protector-all 로 빌드하는데, 이 크로스 툴체인의 GCC
#    spec 이 링크 시 `-lssp` 를 뒤에 붙인다. 그래서 __stack_chk_fail 이
#    libssp 에서 해결되고 DT_NEEDED 에 libssp.so.0 이 박힌다.
#    그런데 대상(Debian Bookworm, glibc 2.36)에는 libssp 가 없다 — glibc 가
#    __stack_chk_fail@@GLIBC_2.17 을 자체 제공하기 때문에 애초에 불필요하다.
#
#  해결: SSP 가 쓰는 심볼 두 개를 각각 원래 자리에서 해결시키고,
#    --as-needed 로 쓰이지 않게 된 libssp 를 DT_NEEDED 에서 뺀다.
#      __stack_chk_fail  -> libc.so.6                (-lc 를 앞쪽에 한 번)
#      __stack_chk_guard -> ld-linux-aarch64.so.1    (aarch64 는 동적 링커가 정의)
#    이 툴체인은 -mstack-protector-guard=global 이 기본이라 guard 를 심볼로
#    참조하는데, glibc 는 그걸 내보내지 않고 rtld 가 내보낸다. ld-linux 는
#    어차피 항상 적재되므로 명시적으로 걸어도 부담이 없다.
#    **스택 보호는 그대로 유지된다** — 구현 제공자만 바뀔 뿐이다.
#
#  ※ Docker(adts, arm64 네이티브) 빌드는 이 문제가 없다 — 대상과 같은 glibc
#    툴체인이라 처음부터 libssp 대신 libc + ld-linux 로 해결한다.
#    **배포용 바이너리는 Docker 빌드를 쓰는 것을 권장**하고, 이 크로스 경로는
#    맥에서 빠르게 빌드·인덱싱하는 용도로 둔다.
# ---------------------------------------------------------------------------
#  ⚠️ 배치가 중요하다. CMAKE_EXE_LINKER_FLAGS 는 링크 라인에서 **오브젝트
#    앞**에 놓인다. 거기에 -lc 를 걸면 그 시점엔 아직 미해결 심볼이 없어
#    --as-needed 가 통째로 버리고, 결국 뒤에 오는 spec 의 -lssp 가 심볼을
#    가져간다. 그래서 링크 라인 **맨 뒤**에 붙는 CMAKE_C_STANDARD_LIBRARIES
#    를 쓴다.
set(CMAKE_EXE_LINKER_FLAGS_INIT "-Wl,--as-needed")

if(CROSS_SYSROOT AND EXISTS "${CROSS_SYSROOT}/lib/ld-linux-aarch64.so.1")
    set(CMAKE_C_STANDARD_LIBRARIES
        "-lc ${CROSS_SYSROOT}/lib/ld-linux-aarch64.so.1"
        CACHE STRING "libssp 회피: SSP 심볼을 libc + rtld 에서 해결" FORCE)
else()
    set(CMAKE_C_STANDARD_LIBRARIES "-lc"
        CACHE STRING "libssp 회피(부분)" FORCE)
endif()
