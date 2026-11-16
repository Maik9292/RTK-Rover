#!/usr/bin/env bash
# =============================================================================
# Script: update_e220_pacer.sh
# Description: Installs E220 pacer, verification, and utility scripts & systemd unit
#              from the directory containing this script (stripping any Windows CRLF),
#              reloads systemd units, restarts e220-pacer, and tails journal logs.
# Usage (from inside the same directory):
#   ./update_e220_pacer.sh              # Deploy + follow logs
#   ./update_e220_pacer.sh --no-follow  # Deploy + print last 40 lines & exit
#   bash update_e220_pacer.sh           # Works without chmod +x
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FOLLOW_LOGS=1

install_lf_file() {
    local src="$1"
    local dst="$2"
    local mode="$3"
    local tmp
    tmp="$(mktemp)"
    tr -d '\r' < "${src}" > "${tmp}"
    sudo install -m "${mode}" "${tmp}" "${dst}"
    rm -f "${tmp}"
}

for arg in "$@"; do
    case "$arg" in
        -n|--no-follow)
            FOLLOW_LOGS=0
            ;;
        -h|--help)
            echo "Usage: ./$(basename "$0") [--no-follow]"
            exit 0
            ;;
        *)
            echo "[ERROR] Unknown option: $arg" >&2
            exit 1
            ;;
    esac
done

if [[ ! -f "${SCRIPT_DIR}/e220_pacer.py" ]]; then
    echo "[ERROR] e220_pacer.py not found in ${SCRIPT_DIR}" >&2
    exit 1
fi

echo "[UPDATE] Stopping running e220-pacer.service before replacing scripts..."
sudo systemctl stop e220-pacer || true

echo "[UPDATE] Installing from local directory: ${SCRIPT_DIR}"
for script_name in e220_pacer.py stage2_bench_verify.py bench_e220_multirate.py set_e220_power.py scan_e220_channels.py; do
    if [[ -f "${SCRIPT_DIR}/${script_name}" ]]; then
        echo "[UPDATE] Installing ${script_name} -> /usr/local/bin/${script_name}"
        install_lf_file "${SCRIPT_DIR}/${script_name}" "/usr/local/bin/${script_name}" 0755
    fi
done

if [[ -f "${SCRIPT_DIR}/e220-pacer.service" ]]; then
    echo "[UPDATE] Syncing e220-pacer.service -> /etc/systemd/system/e220-pacer.service"
    install_lf_file "${SCRIPT_DIR}/e220-pacer.service" /etc/systemd/system/e220-pacer.service 0644
fi

echo "[UPDATE] Reloading systemd unit definitions (daemon-reload)..."
sudo systemctl daemon-reload

echo "[UPDATE] Starting e220-pacer.service..."
sudo systemctl start e220-pacer

STATE="$(sudo systemctl is-active e220-pacer || true)"
echo "[UPDATE] Service state: ${STATE}"

if [[ "${FOLLOW_LOGS}" -eq 1 ]]; then
    echo "[UPDATE] Attaching to journalctl (Ctrl+C to detach)..."
    exec sudo journalctl -u e220-pacer -n 40 -f
else
    sudo journalctl -u e220-pacer -n 40 --no-pager
fi
