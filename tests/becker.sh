#!/bin/bash
# Becker/DriveWire transport regression: start a loopback listener,
# boot with DWEnable=1, and require the status line to reach "DW OK".
# Exercises the BSD-socket half of FD502/becker.cpp including the
# connect-retry path (the listener may come up after the first
# attempt) and the retry-flag clear on success.
set -euo pipefail
cd "$(dirname "$0")/.."

WORK="${TMPDIR:-/tmp}/vcc-becker-test"
mkdir -p "$WORK"

cat > "$WORK/serv.py" <<'EOF'
import socket, threading
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(("127.0.0.1", 65504)); s.listen(4)
def handle(c):
    try:
        while c.recv(4096): pass
    except OSError: pass
while True:
    c, a = s.accept()
    threading.Thread(target=handle, args=(c,), daemon=True).start()
EOF

cat > "$WORK/becker.ini" <<EOF
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
[DW Becker]
DWEnable=1
DWServerAddr=127.0.0.1
DWServerPort=65504
EOF

python3 "$WORK/serv.py" & SERV=$!
trap 'kill $SERV 2>/dev/null || true' EXIT
sleep 1

# Enough frames that the emulator runs ~1s of wall time so the
# connection thread has real time to connect.
OUT="$(VCC_INI="$WORK/becker.ini" timeout 120 ./build/vcc-headless \
        "$HOME/roms/coco3.rom" 90000 2>/dev/null | grep 'module status')"
echo "$OUT"
case "$OUT" in
    *"DW OK"*) echo "becker: PASS" ;;
    *)         echo "becker: FAIL"; exit 1 ;;
esac
