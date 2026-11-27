#!/bin/bash

#
# Test of the Tetrys TUN/TAP tunnel (src/modules/tuntap) with a network namespace.
#
# Topology (root namespace = Tetrys sender / client, ns1 = receiver / server):
#
#   root ns                                      ns1
#   tun0 192.168.10.1 <- tetrystunnel_udp -c      tetrystunnel_udp -s -> tun0 192.168.10.2
#                         veth1 10.1.1.1 ======== vpeer1 10.1.1.2
#                               (netem: delay + loss on veth1 egress)
#
# Tetrys is a one-way FEC flow (sender -> receiver, the receiver only sends
# ACKs). The test therefore sends a numbered UDP stream through the tunnel
# (192.168.10.1 -> 192.168.10.2) and counts how many datagrams arrive. The
# netem loss is applied on the Tetrys packets, so the loss seen at the
# application level shows what the FEC recovered.
#
# Usage: test/tetrysnetns.sh            (needs sudo, ip, tc, python3)
#        LOSS_PCT=10 K=4 N=8 test/tetrysnetns.sh
#

set -u

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
TUNNEL="${TUNNEL:-$ROOT_DIR/bin/tetrystunnel_udp}"

NS="ns1"
VETH="veth1"
VPEER="vpeer1"
VETH_ADDR="10.1.1.1"
VPEER_ADDR="10.1.1.2"

TUN="tun0"
TUN_CLIENT="192.168.10.1"
TUN_SERVER="192.168.10.2"
TUN_MTU=1280             # must stay below PLS (1304 B) so that no packet is truncated
TETRYS_PORT=30001

# FEC parameters passed to the sender (-n / -k; 1 repair every K data packets, 2 <= K < N)
N="${N:-10}"
K="${K:-8}"

# Application traffic (UDP, one-way)
APP_PORT=5001
PKT_COUNT="${PKT_COUNT:-2000}"
PKT_SIZE="${PKT_SIZE:-1000}"
PKT_INTERVAL="${PKT_INTERVAL:-0.005}"   # seconds

# Size (KB) of the receive buffer of the application sink (default 4096; 0 = kernel default).
# When a gap is filled, Tetrys releases the packets it was holding back in one burst; a small
# sink buffer then overflows and drops them after Tetrys (seen as bursts of missing datagrams).
# RCVBUF_KB=4096

# Netem impairments on the Tetrys packets
DELAY_MS="${DELAY_MS:-20}"
LOSS_PCT="${LOSS_PCT:-5}"

# pkill -x cannot be used (the kernel truncates process names to 15 characters and
# "tetrystunnel_udp" has 16), so match the command line; the [t] bracket keeps the
# pattern from matching the sudo/pkill command line that carries it.
TUNNEL_PATTERN="[t]etrystunnel_udp -i"

WORKDIR="$(mktemp -d)"
RESULT_FILE="$WORKDIR/received.txt"

# ── Checks ───────────────────────────────────────────────────────────────────

if [ ! -x "$TUNNEL" ]; then
    echo "Building $TUNNEL"
    make -C "$ROOT_DIR" all >/dev/null || { echo "make failed"; exit 1; }
fi
for tool in ip tc python3 sudo; do
    command -v "$tool" >/dev/null || { echo "Missing tool: $tool"; exit 1; }
done

# ── Cleanup ──────────────────────────────────────────────────────────────────

cleanup() {
    sudo pkill -f "$TUNNEL_PATTERN" 2>/dev/null || true
    kill $(jobs -p) 2>/dev/null || true
    wait 2>/dev/null || true
    sudo tc qdisc del dev $VETH root 2>/dev/null || true
    sudo ip netns delete $NS 2>/dev/null || true
    sudo ip link del dev $VETH 2>/dev/null || true
    sudo ip link del dev $TUN  2>/dev/null || true
    rm -rf "$WORKDIR"
}
trap cleanup EXIT

# ── Network setup ────────────────────────────────────────────────────────────

sudo pkill -f "$TUNNEL_PATTERN" 2>/dev/null || true
sudo ip netns delete $NS 2>/dev/null || true
sudo ip link del dev $VETH 2>/dev/null || true
sudo ip link del dev $TUN  2>/dev/null || true

sudo ip netns add $NS
sudo ip link add ${VETH} type veth peer name ${VPEER}
sudo ip link set ${VPEER} netns $NS
sudo ip addr add ${VETH_ADDR}/24 dev ${VETH}
sudo ip link set ${VETH} up
sudo ip netns exec $NS ip addr add ${VPEER_ADDR}/24 dev ${VPEER}
sudo ip netns exec $NS ip link set ${VPEER} up
sudo ip netns exec $NS ip link set lo up

sudo ip tuntap add ${TUN} mode tun
sudo ip link set ${TUN} mtu ${TUN_MTU} up
sudo ip addr add ${TUN_CLIENT}/24 dev ${TUN}

sudo ip netns exec $NS ip tuntap add ${TUN} mode tun
sudo ip netns exec $NS ip link set ${TUN} mtu ${TUN_MTU} up
sudo ip netns exec $NS ip addr add ${TUN_SERVER}/24 dev ${TUN}

sudo tc qdisc add dev $VETH root netem delay ${DELAY_MS}ms loss ${LOSS_PCT}%

# ── Start the tunnel processes ───────────────────────────────────────────────

echo "Receiver: ip netns exec $NS $TUNNEL -i $TUN -s -p $TETRYS_PORT"
sudo ip netns exec $NS "$TUNNEL" -i ${TUN} -s -p ${TETRYS_PORT} \
    > "$WORKDIR/server.log" 2>&1 &
sleep 1

echo "Sender:   $TUNNEL -i $TUN -c $VPEER_ADDR -p $TETRYS_PORT -n $N -k $K"
sudo "$TUNNEL" -i ${TUN} -c ${VPEER_ADDR} -p ${TETRYS_PORT} -n ${N} -k ${K} \
    > "$WORKDIR/client.log" 2>&1 &
sleep 1

# ── Application traffic ──────────────────────────────────────────────────────

# Kernel counters of the receiving namespace, to tell a loss inside Tetrys from a
# loss after it: UDP RcvbufErrors (application socket overflow) and tun0 drops.
udp_counter() {   # $1 = column name in the Udp: line of /proc/net/snmp
    sudo ip netns exec $NS awk -v col="$1" '/^Udp:/ { if (!h) { for (i = 1; i <= NF; i++) if ($i == col) c = i; h = 1 } else print $c }' /proc/net/snmp
}
RCVBUF_ERR_BEFORE=$(udp_counter RcvbufErrors)
TUN_STATS_BEFORE=$(sudo ip netns exec $NS ip -s link show $TUN | awk 'NR==4 {print "rx_dropped=" $4} NR==6 {print "tx_dropped=" $4}')

# Receiver (inside ns1): collects sequence numbers until 5 s of silence.
sudo ip netns exec $NS python3 -c '
import socket, sys
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind((sys.argv[1], int(sys.argv[2])))
if len(sys.argv) > 4 and int(sys.argv[4]) > 0:
    size = int(sys.argv[4]) * 1024
    so_rcvbufforce = getattr(socket, "SO_RCVBUFFORCE", 33)  # Linux asm-generic/socket.h
    try:
        s.setsockopt(socket.SOL_SOCKET, so_rcvbufforce, size)   # root: ignores rmem_max
    except OSError:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, size)        # capped by rmem_max
    print("application sink receive buffer: %d KB" % (s.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF) // 2048))
s.settimeout(5.0)
seen = set()
try:
    while True:
        data, _ = s.recvfrom(2048)
        seen.add(int.from_bytes(data[:4], "big"))
except socket.timeout:
    pass
with open(sys.argv[3], "w") as f:
    f.write("%d\n" % len(seen))
    f.write(" ".join(str(i) for i in sorted(seen)) + "\n")
' ${TUN_SERVER} ${APP_PORT} "$RESULT_FILE" ${RCVBUF_KB:-4096} &
RECEIVER_PID=$!
sleep 1

echo "Sending $PKT_COUNT UDP datagrams of $PKT_SIZE B every ${PKT_INTERVAL}s" \
     "(netem: ${DELAY_MS} ms, ${LOSS_PCT}% loss)"
python3 -c '
import socket, sys, time
dst, port, count, size, interval = (sys.argv[1], int(sys.argv[2]), int(sys.argv[3]),
                                    int(sys.argv[4]), float(sys.argv[5]))
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
pad = bytes(size - 4)
for i in range(count):
    s.sendto(i.to_bytes(4, "big") + pad, (dst, port))
    time.sleep(interval)
' ${TUN_SERVER} ${APP_PORT} ${PKT_COUNT} ${PKT_SIZE} ${PKT_INTERVAL}

wait $RECEIVER_PID

# ── Report ───────────────────────────────────────────────────────────────────

RECEIVED=$(head -n1 "$RESULT_FILE" 2>/dev/null || echo 0)
LOST=$((PKT_COUNT - RECEIVED))

echo
echo "── Result ────────────────────────────────────────────"
echo "Sent       : $PKT_COUNT"
echo "Received   : $RECEIVED"
awk -v s="$PKT_COUNT" -v l="$LOST" -v n="$LOSS_PCT" 'BEGIN {
    printf "Residual loss : %.2f %%  (network loss on Tetrys packets: %s %%)\n", 100.0 * l / s, n }'
RCVBUF_ERR_AFTER=$(udp_counter RcvbufErrors)
echo "Receiver socket overflows (UDP RcvbufErrors): $((RCVBUF_ERR_AFTER - RCVBUF_ERR_BEFORE))"
echo "tun0 in ns1 before: $(echo $TUN_STATS_BEFORE)"
echo "tun0 in ns1 after : $(sudo ip netns exec $NS ip -s link show $TUN | awk 'NR==4 {print "rx_dropped=" $4} NR==6 {print "tx_dropped=" $4}' | tr '\n' ' ')"
if [ "$LOST" -gt 0 ] && [ -f "$RESULT_FILE" ]; then
    echo "Missing datagram numbers (ranges):"
    python3 -c '
import sys
total = int(sys.argv[2])
with open(sys.argv[1]) as f:
    lines = f.read().split("\n")
seen = set(int(x) for x in lines[1].split()) if len(lines) > 1 else set()
missing = [i for i in range(total) if i not in seen]
ranges, start = [], None
for i, m in enumerate(missing):
    if start is None:
        start = prev = m
    elif m == prev + 1:
        prev = m
    else:
        ranges.append((start, prev)); start = prev = m
if start is not None:
    ranges.append((start, prev))
print("  " + ", ".join(("%d" % a) if a == b else ("%d-%d" % (a, b)) for a, b in ranges))
print("  %d range(s), longest burst: %d" % (len(ranges), max(b - a + 1 for a, b in ranges)))
' "$RESULT_FILE" "$PKT_COUNT"
fi
echo
echo "── Tunnel statistics (printed at shutdown) ───────────"
sudo pkill -INT -f "$TUNNEL_PATTERN" 2>/dev/null || true
sleep 1
echo "[sender]";   cat "$WORKDIR/client.log"
echo "[receiver]"; cat "$WORKDIR/server.log"

[ "$RECEIVED" -gt 0 ]
