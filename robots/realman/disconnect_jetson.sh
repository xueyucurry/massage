#!/usr/bin/env bash
set -euo pipefail

IFACE="${1:-eno1}"
JETSON_IP="${2:-192.168.1.11}"
HOST_IP="${3:-192.168.1.250}"

echo "[STEP] remove host route for ${JETSON_IP}"
sudo ip route del "${JETSON_IP}/32" dev "${IFACE}" 2>/dev/null || true

echo "[STEP] remove helper IP ${HOST_IP}/24 from ${IFACE}"
sudo ip addr del "${HOST_IP}/24" dev "${IFACE}" 2>/dev/null || true

echo "[DONE] cleanup finished"
