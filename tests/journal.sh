#!/bin/bash
# Journal app end-to-end regression: the ChaCha20 core against the
# RFC 8439 test vector on the host, the CMOC --os9 build in the cmoc
# container, the create/seek/write syscall probe, then full journal
# write / append / read / wrong-passphrase runs in headless NitrOS-9,
# finishing with a plaintext sweep of the raw disk image.
# Needs: podman (localhost/cmoc:freshen), ~/roms/coco3.rom, and a
# vcc.ini that boots NitrOS-9 (the journal disk rides in /d1).
set -euo pipefail
cd "$(dirname "$0")/.."

W="${TMPDIR:-/tmp}/vcc-journal-test"
mkdir -p "$W"
ok()  { echo "  ok: $1"; }
bad() { echo "  FAIL: $1"; exit 1; }

# 1. ChaCha20 against the RFC 8439 section 2.3.2 vector, host-side.
cc -O2 -o "$W/chacha_host" tests/chacha_host.c apps/journal/chacha.c
"$W/chacha_host" | grep -q PASS || bad "chacha RFC vector"
ok "chacha RFC vector"

# 2. Build the modules and a fresh OS-9 disk in the cmoc container.
cp apps/journal/journal.c apps/journal/chacha.c apps/journal/chacha.h \
   apps/journal/os9sys.c apps/journal/os9sys.h apps/journal/seektest.c "$W/"
podman run --rm -v "$W":/work localhost/cmoc:freshen sh -c '
    cd /work &&
    cmoc --os9 -o seektest seektest.c os9sys.c &&
    cmoc --os9 -o journal journal.c chacha.c os9sys.c &&
    rm -f j.dsk &&
    os9 format -e -t35 -ss -dd -q j.dsk &&
    os9 makdir j.dsk,CMDS &&
    os9 copy journal j.dsk,CMDS/journal &&
    os9 copy seektest j.dsk,CMDS/seektest &&
    os9 attr -q -e -pe -r -pr j.dsk,CMDS/journal &&
    os9 attr -q -e -pe -r -pr j.dsk,CMDS/seektest' >/dev/null
ok "container build"

BOOT=$'DOS\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~chx /d1/cmds\n~~'
run() {  # run "<keys after boot>" <screen file>
    VCC_DISK1="$W/j.dsk" VCC_FRAMESKIP=100 VCC_SHOT_TEXT="$W/$2" \
        timeout 120 ./build/vcc-headless "$HOME/roms/coco3.rom" 30000 \
        "$BOOT$1" >/dev/null 2>&1
}

# 3. Syscall probe: create, size, seek, write, size on a real file.
run $'seektest\n~~~~~~~~' seek.txt
grep -q 'SIZE2 16' "$W/seek.txt" || bad "seektest (create/seek/write)"
ok "seektest (create/seek/write)"

# 4. Write an entry, append a second, read both, reject a bad pass.
# The scripted CoCo keyboard delivers lowercase, so the read-back
# greps are lowercase too.
run $'journal -d /d1\n~~~~swordfish\n~~~~FIRST SECRET ENTRY.\n~~.\n~~~~~~~~~~' w.txt
grep -q 'Saved (' "$W/w.txt" || bad "journal write"
ok "journal write"

run $'journal -d /d1\n~~~~swordfish\n~~~~SECOND SECRET ENTRY.\n~~.\n~~~~~~~~~~' a.txt
grep -q 'Saved (' "$W/a.txt" || bad "journal append"
ok "journal append"

run $'journal -d /d1 read\n~~~~swordfish\n~~~~~~~~~~' r.txt
grep -q 'first secret entry' "$W/r.txt" || bad "read: first entry"
grep -q 'second secret entry' "$W/r.txt" || bad "read: second entry"
ok "journal read (both entries)"

run $'journal -d /d1 read\n~~~~wrongpass\n~~~~~~~~~~' x.txt
grep -q '(wrong passphrase)' "$W/x.txt" || bad "wrong passphrase rejected"
if grep -q 'secret entry' "$W/x.txt"; then
    bad "wrong passphrase leaked plaintext"
fi
ok "wrong passphrase rejected"

# 5. The raw disk image must hold ciphertext only.
if strings "$W/j.dsk" | grep -qi 'secret entry'; then
    bad "plaintext found on disk image"
fi
ok "ciphertext only on disk"

echo "journal: PASS"
