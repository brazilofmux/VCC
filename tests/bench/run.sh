#!/bin/bash
# Build and run the compiled-C CPU benchmark (tests/bench/sieve.c) in
# vcc-headless. Compiles with the local cmoc podman container, writes a
# DECB disk image, points vcc-headless at a purpose-built ini via
# VCC_INI, types LOADM"SIEVE / EXEC through the scripted keyboard, and
# reports the PRIMES result plus wall-clock speed.
#
# Usage: tests/bench/run.sh [frames]
#        Env (VCC_NO_TAKEN=1, VCC_NO_JIT=1, ...) passes through.
set -euo pipefail

cd "$(dirname "$0")"
REPO="$(cd ../.. && pwd)"
WORK="${TMPDIR:-/tmp}/vcc-bench"
FRAMES="${1:-12000}"
mkdir -p "$WORK"

# Compile + write the disk when the source is newer than the image.
if [ ! -f "$WORK/sieve.dsk" ] || [ sieve.c -nt "$WORK/sieve.dsk" ]; then
    cp sieve.c "$WORK/"
    # A DECB-formatted blank disk is 35x18x256 bytes of 0xFF (what
    # DSKINI writes): all-0xFF GAT = every granule free, all-0xFF
    # directory = no entries.
    python3 -c "open('$WORK/sieve.dsk','wb').write(b'\xff' * 161280)"
    podman run --rm -v "$WORK":/work localhost/cmoc:freshen sh -c \
        'cd /work && cmoc -o sieve.bin sieve.c && writecocofile sieve.dsk sieve.bin'
fi

# Minimal machine config: 6309, 2MB, FD502 alone with the bench disk.
cat > "$WORK/bench.ini" <<EOF
[CPU]
CpuType=1
[Memory]
RamSize=2
[Misc]
ExternalBasicImage=$HOME/roms/coco3.rom
CartAutoStart=0
[Module]
OnBoot=fd502.dll
[FD-502]
DiskRom=1
Disk#0=$WORK/sieve.dsk
EOF

VCC_INI="$WORK/bench.ini" VCC_FRAMESKIP=100 \
    "$REPO/build/vcc-headless" "$HOME/roms/coco3.rom" "$FRAMES" \
    $'LOADM"SIEVE\n~~~~~~~~~~EXEC\n' 2>/dev/null \
    | grep -E 'PRIMES|BENCH|realtime|effective' || true
