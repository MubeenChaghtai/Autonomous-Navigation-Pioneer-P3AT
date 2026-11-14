import csv
import math
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def pose(navigator, x, y, yaw):
    msg = PoseStamped()
    msg.header.frame_id = "map"
    msg.header.stamp = navigator.get_clock().now().to_msg()
    msg.pose.position.x = float(x)
    msg.pose.position.y = float(y)
    msg.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.orientation.w = math.cos(yaw / 2.0)
    return msg


def main(args=None):
    rclpy.init(args=args)
    navigator = BasicNavigator()
    navigator.declare_parameter("goals_file", "")
    navigator.declare_parameter("results_file", "/tmp/p3at_goal_results.csv")
    navigator.declare_parameter("timeout", 180.0)
    with open(navigator.get_parameter("goals_file").value, "r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    start = config["initial_pose"]
    navigator.setInitialPose(pose(navigator, start["x"], start["y"], start.get("yaw", 0.0)))
    navigator.waitUntilNav2Active()
    rows = []
    timeout = float(navigator.get_parameter("timeout").value)
    for goal in config["goals"]:
        target = pose(navigator, goal["x"], goal["y"], goal.get("yaw", 0.0))
        begin = time.monotonic()
        navigator.goToPose(target)
        recoveries = 0
        distance = float("nan")
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback is not None:
                recoveries = feedback.number_of_recoveries
                distance = feedback.distance_remaining
            if time.monotonic() - begin > timeout:
                navigator.cancelTask()
            time.sleep(0.1)
        result = navigator.getResult()
        rows.append({"goal": goal.get("name", f"{goal['x']},{goal['y']}"), "result": result.name if hasattr(result, "name") else str(result), "succeeded": result == TaskResult.SUCCEEDED, "seconds": round(time.monotonic() - begin, 2), "recoveries": recoveries, "last_distance_remaining": distance})
        navigator.get_logger().info(str(rows[-1]))
    with open(navigator.get_parameter("results_file").value, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    succeeded = sum(r["succeeded"] for r in rows)
    navigator.get_logger().info(f"{succeeded}/{len(rows)} goals reached")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
