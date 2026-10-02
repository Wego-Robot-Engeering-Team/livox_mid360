#!/usr/bin/env bash
set -eo pipefail
repository_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
workspace_dir="$(cd "$repository_dir/../.." && pwd -P)"
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
source "$workspace_dir/install/setup.bash"
export PATH="/usr/bin:$PATH"
set -u
cd "$workspace_dir"
colcon test --packages-select livox_mid360 --event-handlers console_direct+ \
  --return-code-on-test-failure
colcon test-result --test-result-base build/livox_mid360 --verbose
/usr/bin/python3 "$repository_dir/scripts/smoke_test.py"
ros2 launch livox_mid360 raw.launch.py --show-args > /dev/null
ros2 launch livox_mid360 leveled.launch.py --show-args > /dev/null
