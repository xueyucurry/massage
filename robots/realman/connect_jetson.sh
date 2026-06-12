#!/usr/bin/env bash
set -euo pipefail

IFACE="${1:-eno1}"
JETSON_IP="${2:-192.168.1.11}"
HOST_IP="${3:-192.168.1.250}"

echo "[INFO] interface: ${IFACE}"
echo "[INFO] jetson ip: ${JETSON_IP}"
echo "[INFO] host helper ip: ${HOST_IP}"

echo "[STEP] add helper IP on ${IFACE}"
sudo ip addr add "${HOST_IP}/24" dev "${IFACE}" label "${IFACE}:jetson" 2>/dev/null || true

echo "[STEP] add host route for ${JETSON_IP} via ${IFACE}"
sudo ip route replace "${JETSON_IP}/32" dev "${IFACE}" src "${HOST_IP}" metric 50

echo "[STEP] show resulting route"
ip route get "${JETSON_IP}" || true

echo "[STEP] ping test"
ping -I "${HOST_IP}" -c 3 -W 1 "${JETSON_IP}" || true

echo "[STEP] ssh port probe"
timeout 3 bash -lc "</dev/tcp/${JETSON_IP}/22" && echo "[OK] TCP/22 open" || echo "[WARN] TCP/22 closed or filtered"

echo
echo "[DONE] If ping works, you can try:"
echo "  ssh <username>@${JETSON_IP}"
