"""Regression check for ROS string-array overrides in both real node adapters."""
import rclpy
from rclpy.parameter import Parameter
from frc_navigation.navigation_node import NavigationNode
from frc_strategy.strategy_node import StrategyNode


def check(args):
    rclpy.init(args=args)
    nodes = []
    try:
        for cls, name in ((StrategyNode, "field_targets"),
                          (NavigationNode, "static_rectangles")):
            node = cls()
            nodes.append(node)
            parameter = node.get_parameter_or(name, Parameter(name, Parameter.Type.STRING_ARRAY, []))
            assert parameter.type_ == Parameter.Type.STRING_ARRAY, parameter
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    check([])
    check(["--ros-args", "-p", "field_targets:=['line,CROSS_LINE,3.2,2.0,0.0,drive,5.0,2.0,0.0,HOLD']",
           "-p", "static_rectangles:=['3.95,3.45,4.35,4.78']"])
    print("ROS startup: default and YAML-compatible string arrays passed")
