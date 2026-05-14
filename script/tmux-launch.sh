#!/usr/bin/env bash

set -eo pipefail

echo "[odin-tmux] start"

env_setup_candidates=(
    "/ubuntu/env_setup.bash"
    "/root/env_setup.bash"
)

env_setup_file=""
for candidate in "${env_setup_candidates[@]}"; do
    if [ -f "$candidate" ]; then
        env_setup_file="$candidate"
        break
    fi
done

if [ -z "$env_setup_file" ]; then
    echo "[odin-tmux] error: no env setup file found"
    exit 1
fi

if tmux has-session -t odin 2>/dev/null; then
    echo "[odin-tmux] old session found, restarting"
    tmux kill-session -t odin
fi

tmux new-session -d -s odin -n driver \
    "bash -lc 'source \"$env_setup_file\" && ros2 launch odin_ros_driver odin1_ros2.launch.py'"

echo "[odin-tmux] ready (tmux attach -t odin)"
