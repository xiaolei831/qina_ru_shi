#!/usr/bin/env bash
set -euo pipefail

SESSION_NAME="qian_sai_moveit_demo"
WORKSPACE="/home/sunrise/qian_sai"

if ! command -v screen >/dev/null 2>&1; then
  echo "screen is required to run the MoveIt demo in a detachable terminal session." >&2
  exit 1
fi

if screen -list | grep -q "[.]${SESSION_NAME}[[:space:]]"; then
  screen -S "${SESSION_NAME}" -X quit
  sleep 1
fi

screen -dmS "${SESSION_NAME}" bash -lc \
  "cd '${WORKSPACE}' && source install/setup.bash && ros2 launch robot_arm demo.launch.py"

echo "MoveIt demo started in screen session: ${SESSION_NAME}"
echo "Attach with: screen -r ${SESSION_NAME}"
