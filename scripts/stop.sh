#!/usr/bin/env bash
# Stop the DUBO ROV Controller systemd service cleanly.
#
# Only STOPS the running service. Auto-start stays enabled (it will come back
# at next boot — that is the point). To remove auto-start completely use:
#   systemctl disable rov-controller.service
set -euo pipefail

UNIT_NAME="rov-controller.service"

if [[ ${EUID} -ne 0 ]]; then
    exec sudo -p "sudo password for %u: " bash "$0" "$@"
fi

if ! systemctl is-active --quiet "${UNIT_NAME}"; then
    echo "rov-controller service is not active (nothing to stop)."
    systemctl is-enabled "${UNIT_NAME}" 2>/dev/null || true
    exit 0
fi

systemctl stop "${UNIT_NAME}"
echo "rov-controller service stopped cleanly."
systemctl status "${UNIT_NAME}" --no-pager -l || true