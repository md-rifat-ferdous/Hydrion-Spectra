#!/usr/bin/env bash
# Restart the DUBO ROV Controller systemd service cleanly.
set -euo pipefail

UNIT_NAME="rov-controller.service"

if [[ ${EUID} -ne 0 ]]; then
    exec sudo -p "sudo password for %u: " bash "$0" "$@"
fi

if ! systemctl is-enabled --quiet "${UNIT_NAME}" 2>/dev/null; then
    echo "WARNING: ${UNIT_NAME} is not installed/enabled — run scripts/start.sh first." >&2
fi

systemctl restart "${UNIT_NAME}"

sleep 4
if ! systemctl is-active --quiet "${UNIT_NAME}"; then
    echo "ERROR: service did not come back up after restart. Recent journal:" >&2
    journalctl -u "${UNIT_NAME}" -n 30 --no-pager >&2 || true
    exit 1
fi

echo "rov-controller service restarted and is running."
systemctl status "${UNIT_NAME}" --no-pager -l || true