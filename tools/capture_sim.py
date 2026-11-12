"""Observe real ROS messages and fail unless a simulated robot actually moves."""
import argparse
import json
import math
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from frc_autonomy_msgs.msg import AutonomyStatus, NavigationState

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--seconds", type=float, default=15.0)
parser.add_argument("--out", type=Path, required=True)
args = parser.parse_args()
rclpy.init()
node = Node("portfolio_sim_observer")
statuses = []
navigation = []
node.create_subscription(AutonomyStatus, "/autonomy/status", statuses.append, 100)
node.create_subscription(NavigationState, "/navigation/state", navigation.append, 100)
deadline = time.monotonic() + args.seconds
try:
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
finally:
    node.destroy_node()
    rclpy.shutdown()
distance = sum(math.hypot(b.pose.x - a.pose.x, b.pose.y - a.pose.y)
               for a, b in zip(statuses, statuses[1:]))
result = {"status_messages": len(statuses), "navigation_messages": len(navigation),
          "active_messages": sum(s.command_active for s in statuses),
          "distance_travelled_m": round(distance, 4),
          "planned_path_messages": sum(bool(s.planned_path) for s in navigation),
          "rejection_reasons": sorted({s.reject_reason for s in statuses}),
          "game_specific_messages": sorted({s.game_specific_message for s in statuses}),
          "navigation_reasons": sorted({s.obstacle_reason for s in navigation})}
args.out.parent.mkdir(parents=True, exist_ok=True)
args.out.write_text(json.dumps(result, indent=2) + "\n")
print(json.dumps(result, indent=2))
if not statuses or not navigation or result["active_messages"] == 0 or distance < 0.1:
    raise SystemExit("Live ROS simulation did not demonstrate movement")
