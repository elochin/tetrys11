#!/bin/bash
# Non-regression test for the Tetrys UDP module (no sudo needed).
# Transfers a pseudo-random file over loopback with packet loss injected by an
# LD_PRELOAD shim (build/loss_shim.so, built with `make shim`), then checks
# that the received file is identical to the source.
#
# Environment options:
#   BIN_DIR (bin)  LOSS_PCT (5)  N (10)  K (8)  RUNS (3)  SIZE (300000)
#   PORT_BASE (25000)  INTERVAL_US (2000)
# BIN_DIR=bin-asan works (build it with `make asan`): the ASan runtime must be
# the first preloaded library, so it is detected with ldd and put before the shim.
# Exit code: 0 only if every run produced an identical file.
set -u

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
BIN_DIR=${BIN_DIR:-bin}
LOSS_PCT=${LOSS_PCT:-5}
N=${N:-10}
K=${K:-8}
RUNS=${RUNS:-3}
SIZE=${SIZE:-300000}
PORT_BASE=${PORT_BASE:-25000}
INTERVAL_US=${INTERVAL_US:-2000}

case $BIN_DIR in /*) ;; *) BIN_DIR=$ROOT/$BIN_DIR ;; esac
BIN=$BIN_DIR/tetrys_generate_udp
SHIM=$ROOT/build/loss_shim.so

[ -x "$BIN" ] || make -C "$ROOT" || { echo "build failed" >&2; exit 2; }
[ -f "$SHIM" ] || make -C "$ROOT" shim || { echo "shim build failed" >&2; exit 2; }
[ -x "$BIN" ] && [ -f "$SHIM" ] || { echo "missing binary or shim" >&2; exit 2; }

# With an ASan binary, the sanitizer runtime must come first in LD_PRELOAD,
# otherwise ASan aborts ("ASan runtime does not come first in initial library list").
PRELOAD=$SHIM
ASAN_LIB=$(ldd "$BIN" 2>/dev/null | awk '/libasan\.so/ { print $3; exit }')
[ -n "$ASAN_LIB" ] && PRELOAD="$ASAN_LIB $SHIM"

TMP=$(mktemp -d) || exit 2
RPID=""
cleanup() {
    [ -n "$RPID" ] && kill "$RPID" 2>/dev/null
    rm -rf "$TMP"
}
trap cleanup EXIT INT TERM

head -c "$SIZE" /dev/urandom > "$TMP/src" || exit 2

same=0
for ((i = 1; i <= RUNS; i++)); do
    port=$((PORT_BASE + i))
    out=$TMP/out$i.tr
    LD_PRELOAD="$PRELOAD" LOSS_PCT=$LOSS_PCT \
        "$BIN" -f "$out" -s 1000 -m 2 127.0.0.1 "$port" >"$TMP/rcv$i.log" 2>&1 &
    RPID=$!
    sleep 0.3
    LD_PRELOAD="$PRELOAD" LOSS_PCT=$LOSS_PCT \
        "$BIN" -f "$TMP/src" -i "$INTERVAL_US" -s 1000 -m 1 -n "$N" -k "$K" \
        127.0.0.1 "$port" >"$TMP/snd$i.log" 2>&1
    sleep 3
    kill "$RPID" 2>/dev/null
    wait "$RPID" 2>/dev/null
    RPID=""
    ssz=$(stat -c %s "$TMP/src")
    osz=$(stat -c %s "$out" 2>/dev/null || echo 0)
    if cmp -s "$TMP/src" "$out"; then
        res=SAME; same=$((same + 1))
    else
        res=DIFF
    fi
    echo "run $i: $res (src=$ssz out=$osz loss=${LOSS_PCT}% n=$N k=$K port=$port)"
done

echo "total: $same/$RUNS identical"
[ "$same" -eq "$RUNS" ]
