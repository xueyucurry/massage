#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG="${SCRIPT_DIR}/nomachine_9.4.14_1_amd64.deb"

if [[ "$(dpkg --print-architecture)" != "amd64" ]]; then
  echo "This installer expects amd64, got: $(dpkg --print-architecture)" >&2
  exit 1
fi

if [[ ! -f "${PKG}" ]]; then
  echo "Package not found: ${PKG}" >&2
  exit 1
fi

echo "Installing ${PKG}"
sudo dpkg -i "${PKG}" || sudo apt-get -f install -y
sudo systemctl enable nxserver --now || true
sudo /usr/NX/bin/nxserver --status || true

