#!/bin/bash
# GIME timer regression: runs tests/guest/gimetmr.asm (a DECB machine-code
# probe that polls the timer latch with interrupts masked) and checks
#  1. upstream VCC's timer semantics, against the line-period arithmetic:
#     MSB write restarts; LSB-only write changes the period WITHOUT
#     restarting; value 0 stops it; a later LSB-only nonzero write fires
#     at once
#  2. the JIT is exact: default (JIT + bursts) produces the SAME counts
#     as the interpreter with the same slicing
#  3. bursts agree with per-line slicing to within the dispatch budget
#     slack: a block may start up to kBudgetSlack (48) cycles past its
#     slice budget, i.e. <=3 polls, so where slice boundaries fall can move
#     which poll first sees a fire by that much - but never the totals.
# Counts are poll passes of 18 cycles (20.114 us at 0.894886 MHz); one
# scanline is 63,613 ns, so N timer lines = (N+1)*63613/20114 passes.
set -euo pipefail
cd "$(dirname "$0")/.."
ok()  { echo "  ok: $1"; }
bad() { echo "  FAIL: $1"; exit 1; }

probe() {  # env... -> "A..|B..|C..|"
    env "$@" tools/coco-run --headless --frames 2500 tests/guest/gimetmr.asm 2>&1 \
        | grep -E "^A[0-9]|^B[0-9]|^C[0-9]" | tr '\n' '|'
}
REF="$(probe VCC_NO_JIT=1 VCC_NO_BURST=1)"
INT="$(probe VCC_NO_JIT=1)"
DEF="$(probe X=1)"
echo "  interpreter, per-line:  $REF"
echo "  interpreter, bursts:    $INT"
echo "  JIT, bursts (default):  $DEF"
[ -n "$REF" ] || bad "probe produced no output"

# Pull the numbers out of the reference run: A a1..a4 | B b1..b3 | C z c1 c2
read -r a1 a2 a3 a4 b1 b2 b3 z c1 c2 <<<"$(echo "$REF" | tr -c '0-9\n' ' ')"
near() { [ "$1" -ge $(($2-6)) ] && [ "$1" -le $(($2+6)) ]; }   # +-6 passes
for v in $a1 $a2 $a3 $a4; do near "$v" 1901 || bad "A: 600-line restart period ($v, want ~1901)"; done
ok "MSB write restarts: 601-line period"
near "$b1" 1901 || bad "B: LSB-only write restarted the timer ($b1, want ~1901)"
near "$b2" 2429 && near "$b3" 2429 || bad "B: new 767-line period ($b2 $b3, want ~2429)"
ok "LSB-only write: no restart, new period from the next reload"
[ "$z" = 0 ]  || bad "C: timer fired while its value was zero"
[ "$c1" -le 2 ] || bad "C: overdue countdown did not fire at once ($c1)"
near "$c2" 206 || bad "C: 64-line period after re-enable ($c2, want ~206)"
ok "zero stops the timer; LSB-only nonzero fires at once"

[ "$DEF" = "$INT" ] || bad "JIT disagrees with the interpreter under the same slicing"
ok "JIT identical to interpreter"

nums() { echo "$1" | tr -c '0-9\n' ' '; }
read -r -a R <<<"$(nums "$REF")"; read -r -a B <<<"$(nums "$INT")"
[ "${#R[@]}" = "${#B[@]}" ] || bad "burst and per-line runs differ in shape"
sumr=0; sumb=0
for i in "${!R[@]}"; do
    d=$(( R[i] - B[i] )); [ $d -lt 0 ] && d=$(( -d ))
    [ $d -le 3 ] || bad "bursts moved interval $i by $d polls (slack bound is 3)"
    sumr=$(( sumr + R[i] )); sumb=$(( sumb + B[i] ))
done
d=$(( sumr - sumb )); [ $d -lt 0 ] && d=$(( -d ))
[ $d -le 3 ] || bad "bursts changed total elapsed time ($sumr vs $sumb polls)"
ok "bursts within budget slack of per-line slicing (totals $sumb vs $sumr)"
echo "gimetimer: PASS"
