#!/bin/bash
# DriveWire end-to-end regression: clone the user's NitrOS-9 VHD
# (APFS copy-on-write), bake the DriveWire bootfile into the clone
# with tools/dw-bake, serve two ToolShed-built floppy images with
# pyDriveWire, and prove file flow both ways over the becker port:
# a file written on the Mac is read from /x1 inside NitrOS-9, and a
# file written from OS-9 comes back out with ToolShed. Also proves
# the baked VHD boots cleanly with no server running.
# Needs: podman (localhost/cmoc:freshen), ~/roms/coco3.rom, a vcc.ini
# that boots the emudsk NitrOS-9 (VHD + boot floppy), and
# ~/g/pyDriveWire with its .venv. Uses port 65510 to stay clear of
# any real DriveWire server on 65504.
set -euo pipefail
cd "$(dirname "$0")/.."

W="${TMPDIR:-/tmp}/vcc-dw-test"
rm -rf "$W"; mkdir -p "$W"
ok()  { echo "  ok: $1"; }
bad() { echo "  FAIL: $1"; exit 1; }

PYDW="$HOME/g/pyDriveWire"
[ -x "$PYDW/.venv/bin/python" ] || { echo "drivewire: skip (no pyDriveWire venv)"; exit 0; }

# 1. Clone the VHD and bake DriveWire into the clone.
cp -c ~/coco/disks/NitrOS9.vhd "$W/dw.vhd" 2>/dev/null || cp ~/coco/disks/NitrOS9.vhd "$W/dw.vhd"
( cd "$W" && python3 "$OLDPWD/tools/dw-bake" "$W/dw.vhd" ) >/dev/null
python3 tools/dw-bake "$W/dw.vhd" | grep -q "already carries rbdw" \
    || bad "dw-bake not idempotent"
ok "dw-bake (bake + idempotence)"

# 2. Two served disks; drive 1 pre-loaded from the Mac side.
printf 'HELLO FROM THE MAC SIDE\n' > "$W/hello.txt"
podman run --rm -v "$W":/work localhost/cmoc:freshen sh -c '
    cd /work &&
    os9 format -e -t35 -ss -dd -q dw0.dsk &&
    os9 format -e -t35 -ss -dd -q dw1.dsk &&
    os9 copy -l hello.txt dw1.dsk,hello.txt' >/dev/null
ok "served disks built"

# 3. Baked VHD must boot cleanly with NO server running.
sed -e "s|^VHDImage=.*|VHDImage=$W/dw.vhd|" ~/.config/vcc/vcc.ini > "$W/dw.ini"
run() {  # run <screenfile> <keys-after-boot>
    VCC_DW=1 VCC_DW_PORT=65510 VCC_INI="$W/dw.ini" VCC_FRAMESKIP=100 \
        VCC_SHOT_TEXT="$W/$1" timeout 240 ./build/vcc-headless \
        "$HOME/roms/coco3.rom" 140000 \
        $'DOS\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'"$2" >/dev/null 2>&1
}
run noserv.txt $'mdir\n~~~~~~~~~~'
grep -q 'rbdw' "$W/noserv.txt" || bad "DW modules not resident after bake"
grep -q 'Shell' "$W/noserv.txt" || bad "baked VHD did not boot without server"
ok "baked VHD boots serverless, modules resident"

# 4. Start pyDriveWire and push files both ways through /x1.
# The server subshell gets its OWN stdio (never the script's pipes -
# an inherited stderr would hold the harness's output pipe open long
# after this script exits), and cleanup kills the actual server
# processes, not just the subshell wrapper.
( tail -f /dev/null | "$PYDW/.venv/bin/python" -u "$PYDW/pyDriveWire.py" \
      -a -p 65510 "$W/dw0.dsk" "$W/dw1.dsk" ) </dev/null >"$W/pydw.log" 2>&1 &
SERVER=$!
disown $SERVER 2>/dev/null || true
cleanup() {
    pkill -P $SERVER 2>/dev/null || true
    kill $SERVER 2>/dev/null || true
    pkill -f "pyDriveWire.py -a -p 65510" 2>/dev/null || true
    wait $SERVER 2>/dev/null || true
}
trap cleanup EXIT
sleep 2

run read.txt $'dir /x1\n~~~~~~list /x1/hello.txt\n~~~~~~~~~~'
grep -qi 'hello from the mac side' "$W/read.txt" || bad "Mac -> OS-9 read over DriveWire"
ok "Mac -> OS-9 (list /x1/hello.txt)"

run write.txt $'echo FROM THE COCO >/x1/note.txt\n~~~~~~dir /x1\n~~~~~~~~~~'
grep -q 'note.txt' "$W/write.txt" || bad "OS-9 note.txt not in /x1 dir"
podman run --rm -v "$W":/work localhost/cmoc:freshen sh -c '
    cd /work && os9 copy -l dw1.dsk,note.txt note.out' >/dev/null
grep -qi 'from the coco' "$W/note.out" || bad "OS-9 -> Mac write over DriveWire"
ok "OS-9 -> Mac (note.txt round-trip)"

echo "drivewire: PASS"
