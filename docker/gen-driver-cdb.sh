#!/usr/bin/env bash
# 드라이버 compile_commands.json 생성(bear) + 맥 CLion용 경로 리라이트
# 사용: bash <repo>/docker/gen-driver-cdb.sh   (레포 어디에 두든 동작)
#   전제: 컨테이너 adts 실행 중(modules_prepare 완료), 커널 헤더가 ~/kernel-src 에 복사됨.
set -euo pipefail
# 스크립트 위치(<repo>/docker)에서 레포 루트를 역산 — clone 경로에 무관
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
KSRC="$HOME/kernel-src"
CONTAINER=adts

echo ">> [1/2] 컨테이너에서 bear 로 실제 빌드 플래그 캡처(강제 재빌드)"
docker exec "$CONTAINER" bash -c '
  cd /work/driver
  rm -f *.o *.ko *.mod* .*.cmd modules.order turret_test 2>/dev/null || true
  bear -- make rpi RPI_KDIR=/usr/src/linux LOCALVERSION=
' >/dev/null

echo ">> [2/2] 컨테이너 경로 -> 맥 경로 리라이트 (CLion 로컬 인덱싱용)"
python3 - "$REPO" "$KSRC" <<'PY'
import json, os, sys
repo, ksrc = sys.argv[1], sys.argv[2]
p = os.path.join(repo, 'driver', 'compile_commands.json')
d = json.load(open(p))
def fix(s):
    return (s.replace('/usr/src/linux', ksrc)
             .replace('/work/driver', os.path.join(repo, 'driver'))
             .replace('/usr/bin/aarch64-linux-gnu-gcc', '/usr/bin/clang'))

# clang 이 모르는 GCC 전용 플래그 (인덱싱만 하면 되므로 그냥 제거).
# 있으면 clang 이 'unknown argument' 로 파싱 자체를 거부해 인덱싱이 죽는다.
GCC_ONLY_EXACT = {
    '-fno-allow-store-data-races', '-fconserve-stack',
    '-fno-var-tracking-assignments', '-femit-struct-debug-baseonly',
    '-fno-inline-functions-called-once', '-mabi=lp64',
    '-fno-stack-clash-protection', '-mstack-protector-guard=sysreg',
    '-fno-builtin-wcslen',
}
GCC_ONLY_PREFIX = (
    '-mstack-protector-guard-', '-fpatchable-function-entry',
    '-Wno-dangling-pointer', '-Wno-alloc-size-larger-than',
    '-Wno-packed-not-aligned', '-Wno-format-overflow',
    '-Wno-format-truncation', '-Wno-stringop-overflow',
    '-Wno-stringop-truncation', '-Wno-maybe-uninitialized',
    '-Wno-override-init', '-Wenum-conversion', '-Wno-unused-but-set-variable',
    '-Wimplicit-fallthrough=', '-Wno-frame-address', '-Wno-psabi',
    '-Wa,',
)
def keep(a):
    if a in GCC_ONLY_EXACT or a == '-Werror' or a.startswith('-Werror='):
        return False          # -Werror: 커널 헤더의 워닝이 에러로 승격돼 인덱싱이 죽는다
    return not a.startswith(GCC_ONLY_PREFIX)

out = []
for e in d:
    if not e['file'].startswith('/work/driver'):   # 우리 코드만 (커널 scripts 제외)
        continue
    e['directory'] = fix(e['directory'])
    e['file'] = fix(e['file'])
    raw = e.get('arguments') or e['command'].split()
    is_kernel = '-D__KERNEL__' in raw            # 커널 모듈 빌드인가?
    args = [fix(a) for a in raw if keep(a)]
    if is_kernel:
        # 커널은 리눅스(ELF) 코드 → 맥 기본 타깃(Mach-O)으로 파싱하면
        # __section(".init.text") 등에서 'mach-o section specifier' 에러가 난다.
        if not any(a.startswith('--target=') for a in args):
            args.insert(1, '--target=aarch64-linux-gnu')
    # 유저스페이스(turret_test.c)는 맥 시스템 헤더(stdio.h)를 써야 하므로 --target 을 넣지 않는다.
    if 'arguments' in e:
        e['arguments'] = args
    if 'command' in e:
        e['command'] = ' '.join(args)
    out.append(e)
json.dump(out, open(p, 'w'), indent=1)
print(f"   rewrote {len(out)} entries -> {p}")
PY
echo ">> 완료. CLion 에서 RPi/driver/compile_commands.json 을 Compilation Database 로 Open."
