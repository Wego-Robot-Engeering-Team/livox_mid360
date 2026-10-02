#!/usr/bin/env bash
set -euo pipefail
repository_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
cd "$repository_dir"
export PATH="/usr/bin:$PATH"
if [[ ! -d third_party/Livox-SDK2/.git || ! -d third_party/livox_ros_driver2/.git ]]; then
  vcs import --skip-existing . < "$repository_dir/dependencies.repos"
fi
/usr/bin/python3 - "$repository_dir/dependencies.repos" <<'PY'
from pathlib import Path
import subprocess
import sys
import yaml

repositories = yaml.safe_load(Path(sys.argv[1]).read_text())['repositories']
for path, entry in repositories.items():
    actual = subprocess.check_output(['git', '-C', path, 'rev-parse', 'HEAD'], text=True).strip()
    if actual != entry['version']:
        raise SystemExit(f'{path}: expected {entry["version"]}, found {actual}; checkout the pinned commit')
PY
