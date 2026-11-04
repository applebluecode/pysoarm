#!/usr/bin/env bash
# Install the packaged udev rules so the arms get stable names.
#
#   scripts/install_udev_rules.sh /dev/ttyACM0 /dev/ttyACM1
#
# Substitutes the two adapter serials into 99-pysoarm.rules and copies it to
# /etc/udev/rules.d/, then reloads udev.
set -euo pipefail

FOLLOWER_PORT="${1:?usage: $0 <follower-tty> [leader-tty]}"
LEADER_PORT="${2:-}"

serial_of() {
  udevadm info -q property -n "$1" 2>/dev/null | sed -n 's/^ID_SERIAL=.*_//p'
}

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
rules="$(mktemp)"

sed -e "s/REPLACE_FOLLOWER_SERIAL/$(serial_of "$FOLLOWER_PORT")/" \
    "$here/../src/pysoarm/devices/udev/99-pysoarm.rules" >"$rules"

if [[ -n "$LEADER_PORT" ]]; then
  sed -i -e "s/REPLACE_LEADER_SERIAL/$(serial_of "$LEADER_PORT")/" "$rules"
else
  sed -i -e '/REPLACE_LEADER_SERIAL/d' "$rules"
fi

sudo install -m 0644 "$rules" /etc/udev/rules.d/99-pysoarm.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "installed /etc/udev/rules.d/99-pysoarm.rules"
ls -l /dev/so101_* 2>/dev/null || true
