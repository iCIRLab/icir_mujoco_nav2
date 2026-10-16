#!/usr/bin/env bash
# Usage: bash scripts/setup_humble_venv.sh [workspace_root]
# Default layout: <workspace>/src/<repository>/scripts/setup_humble_venv.sh
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_root="${1:-$(cd "$repo_root/../.." && pwd)}"
venv_dir="$workspace_root/.venv-mobile-openarm"
export PYTHONNOUSERSITE=1

/usr/bin/python3 -c 'import sys; assert sys.version_info[:2] == (3, 10), "ROS 2 Humble requires system Python 3.10 for this setup"'

if [[ ! -x "$venv_dir/bin/python" ]]; then
  echo "Creating venv: $venv_dir"
  /usr/bin/python3 -m venv --system-site-packages "$venv_dir"
else
  echo "Using workspace venv: $venv_dir"
fi

"$venv_dir/bin/python" - "$venv_dir" <<'PY'
from pathlib import Path
import sys

venv = Path(sys.argv[1]).resolve()
assert Path(sys.prefix).resolve() == venv, "Python is not using the workspace venv"
assert sys.version_info[:2] == (3, 10), "The existing venv must use Python 3.10"
config = (venv / "pyvenv.cfg").read_text()
assert "include-system-site-packages = true" in config, "The venv must share system ROS packages (--system-site-packages)"
PY

touch "$venv_dir/COLCON_IGNORE"
"$venv_dir/bin/python" -m pip install -r "$repo_root/requirements-humble-venv.txt"

"$venv_dir/bin/python" - <<'PY'
import cv2
import mujoco
import numpy
import scipy
import sys

assert hasattr(cv2.aruco, "ArucoDetector"), "OpenCV ArUco support is missing"
print("Python:", sys.executable)
print("MuJoCo:", mujoco.__version__)
print("NumPy:", numpy.__version__)
print("SciPy:", scipy.__version__)
print("OpenCV:", cv2.__version__)
PY

printf '\nActivate this workspace venv with:\nsource "%s/bin/activate"\n' "$venv_dir"
