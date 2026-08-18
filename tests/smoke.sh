#!/bin/sh
# Headless smoke tests (AGENTS.md conventions, docs/porting-macos.md).
#
# Boots the portable core through three deepening scenarios and greps
# the dumped text screens for expected content. Timestamps and disk
# activity vary run to run, so assertions are presence-based.
#
# Requires: a built build/vcc-headless, coco3.rom (default ~/roms),
# and the vcc.ini machine config (MPI + FD502 boot floppy + VHD) for
# the OS-9 scenarios.
#
# Usage: tests/smoke.sh [path-to-vcc-headless] [path-to-coco3.rom]

set -u

BIN="${1:-build/vcc-headless}"
ROM="${2:-$HOME/roms/coco3.rom}"
FAILS=0

run_and_expect() {
    name="$1"; frames="$2"; script="$3"; shift 3
    out=$("$BIN" "$ROM" "$frames" "$script" 2>&1)
    for expect in "$@"; do
        if printf '%s' "$out" | grep -qF "$expect"; then
            printf 'PASS  %s: %s\n' "$name" "$expect"
        else
            printf 'FAIL  %s: missing "%s"\n' "$name" "$expect"
            printf '%s\n' "$out" | tail -30
            FAILS=$((FAILS + 1))
        fi
    done
}

# 1. Disk BASIC banner (core + MPI + FD502 ROM mapping).
run_and_expect basic 600 '' \
    'DISK EXTENDED COLOR BASIC 2.1' \
    'COPR. 1982, 1986 BY TANDY'

# 2. NitrOS-9 Level 2 boot from the FD502 floppy, /DD on the VHD,
#    module directory via mdir (kernel, disk drivers, windows, clock).
run_and_expect nitros9 4200 'DOS
~~~~~~~~~~mdir
~~' \
    'NitrOS-9/6309 Level 2' \
    'Shell+' \
    'EmuDsk' \
    'KrnP2'

# 3. Basic09 from /DD/CMDS.
run_and_expect basic09 4800 'DOS
~~~~~~~~~~basic09
~~~~' \
    'BASIC09' \
    'COPYRIGHT 1980 BY MOTOROLA INC.' \
    'Ready'

if [ "$FAILS" -eq 0 ]; then
    echo 'smoke: all scenarios passed'
else
    echo "smoke: $FAILS assertion(s) FAILED"
    exit 1
fi
