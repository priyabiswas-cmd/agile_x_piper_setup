#!/usr/bin/env bash
# Bring up every Piper CAN interface at 1 Mbit/s with a FIXED name tied to its USB port.
# Run after every boot, re-plug, or arm power-cycle:   bash scripts/setup_can.sh
#
# Plug each USB-CAN adapter into the hub socket listed below.
# Edit PORTS if you use different ports. Find each adapter's port with:
#   bash scripts/setup_can.sh --find

BITRATE=1000000

# interface-name    USB bus-info      (verified on priyas-Victus, RPL lab)
PORTS=(
  # all 4 adapters on one powered USB hub on laptop port 3-4 (keep each adapter in the same hub socket)
  "can_follower_l   3-4.1:1.0"
  "can_leader_l     3-4.2:1.0"
  "can_leader_r     3-4.3:1.0"
  "can_follower_r   3-4.4:1.0"
)

SDK_DIR=$(python3 -c "import piper_sdk, os; print(os.path.dirname(piper_sdk.__file__))" 2>/dev/null) || {
  echo "piper_sdk not found. Run: pip3 install -r requirements.txt"; exit 1; }

if [[ "$1" == "--find" ]]; then
  bash "$SDK_DIR/find_all_can_port.sh"
  exit 0
fi

missing=()
for entry in "${PORTS[@]}"; do
  read -r name port <<< "$entry"
  bash "$SDK_DIR/can_activate.sh" "$name" "$BITRATE" "$port" || missing+=("$name (USB port $port)")
done

echo
echo "Active CAN interfaces:"
ip -brief link show type can

if (( ${#missing[@]} )); then
  echo
  echo "NOT FOUND:"
  for m in "${missing[@]}"; do echo "  - $m"; done
  echo "Check that adapter is plugged into that hub socket and the arm is powered."
  echo "Run 'bash scripts/setup_can.sh --find' to see which ports have adapters right now."
  exit 1
fi
