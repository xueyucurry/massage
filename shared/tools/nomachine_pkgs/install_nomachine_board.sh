#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BOARD_HOST="${BOARD_HOST:-192.168.1.18}"
BOARD_USER="${BOARD_USER:-root}"
SSH_OPTS=(
  -o StrictHostKeyChecking=no
  -o UserKnownHostsFile=/dev/null
  -o HostKeyAlgorithms=+ssh-rsa
  -o PubkeyAcceptedAlgorithms=+ssh-rsa
)

REMOTE_ARCH="$(
  ssh "${SSH_OPTS[@]}" "${BOARD_USER}@${BOARD_HOST}" "uname -m"
)"

case "${REMOTE_ARCH}" in
  x86_64|amd64)
    PKG="${SCRIPT_DIR}/nomachine_9.4.14_1_amd64.deb"
    ;;
  aarch64|arm64)
    PKG="${SCRIPT_DIR}/nomachine_9.4.14_1_arm64.deb"
    ;;
  *)
    echo "Unsupported remote architecture: ${REMOTE_ARCH}" >&2
    exit 1
    ;;
esac

if [[ ! -f "${PKG}" ]]; then
  echo "Package not found: ${PKG}" >&2
  exit 1
fi

REMOTE_PKG="/tmp/$(basename "${PKG}")"

echo "Remote architecture: ${REMOTE_ARCH}"
echo "Uploading ${PKG} -> ${BOARD_USER}@${BOARD_HOST}:${REMOTE_PKG}"
scp "${SSH_OPTS[@]}" "${PKG}" "${BOARD_USER}@${BOARD_HOST}:${REMOTE_PKG}"

ssh "${SSH_OPTS[@]}" "${BOARD_USER}@${BOARD_HOST}" "\
  if command -v sudo >/dev/null 2>&1; then \
    sudo dpkg -i '${REMOTE_PKG}' || sudo apt-get -f install -y; \
    sudo systemctl enable nxserver --now || true; \
    sudo /usr/NX/bin/nxserver --status || true; \
  else \
    dpkg -i '${REMOTE_PKG}' || apt-get -f install -y; \
    systemctl enable nxserver --now || true; \
    /usr/NX/bin/nxserver --status || true; \
  fi"

