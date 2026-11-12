"""Run the deterministic core demo without ROS, NT or robot hardware."""
from pathlib import Path
import sys

source = Path(__file__).resolve().parents[1] / "jetson" / "ros2_ws" / "src"
sys.path[:0] = [str(p) for p in sorted(source.iterdir()) if (p / "setup.py").exists()]
from frc_bringup.offline_demo import main

if __name__ == "__main__":
    main()
