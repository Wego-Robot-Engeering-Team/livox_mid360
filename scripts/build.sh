#!/usr/bin/env bash
# Optional convenience command. After setup, plain colcon build also works.
set -eo pipefail
repository_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
workspace_dir="$(cd "$repository_dir/../.." && pwd -P)"
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
export PATH="/usr/bin:$PATH"
set -u
"$repository_dir/scripts/setup_dependencies.sh"
cd "$workspace_dir"
colcon build "$@"
