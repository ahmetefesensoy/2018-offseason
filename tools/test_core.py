"""Run the ROS-independent Python tests on Windows or Linux."""
import os
from pathlib import Path
import subprocess
import sys

root = Path(__file__).resolve().parents[1] / "jetson" / "ros2_ws" / "src"
env = os.environ.copy()
packages = [p for p in sorted(root.iterdir()) if (p / "setup.py").exists()]
# Existing strategy tests share fixtures via sibling test modules. Include the
# test folders explicitly so collection is identical with and without colcon.
env["PYTHONPATH"] = os.pathsep.join(str(p) for p in packages + [p / "test" for p in packages if (p / "test").exists()])
raise SystemExit(subprocess.call([sys.executable, "-m", "pytest", str(root), "-q", "--import-mode=importlib"], env=env))
