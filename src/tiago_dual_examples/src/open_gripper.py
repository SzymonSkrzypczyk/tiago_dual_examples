#!/usr/bin/env python3
import rospy
import argparse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def control_gripper(gripper_side, action):
    """
    Control TIAGo Dual grippers (open or close).
    
    :param gripper_side: "left" or "right"
    :param action: "open" or "close"
    """
    if gripper_side not in ["left", "right"]:
        raise ValueError("gripper_side must be 'left' or 'right'")
    if action not in ["open", "close"]:
        raise ValueError("action must be 'open' or 'close'")

    # Joint names for the selected gripper
    joint_names = [
        f"gripper_{gripper_side}_left_finger_joint",
        f"gripper_{gripper_side}_right_finger_joint"
    ]

    # Open = 0.0, Close = 0.045 (might need to be adjusted)
    if action == "open":
        positions = [0.0, 0.0]
    else:
        positions = [0.045, 0.045]

    # Publisher
    pub = rospy.Publisher(
        f"/gripper_{gripper_side}_controller/command",
        JointTrajectory,
        queue_size=10
    )
    rospy.sleep(1)  # wait for connection

    traj = JointTrajectory()
    traj.joint_names = joint_names

    point = JointTrajectoryPoint()
    point.positions = positions
    point.time_from_start = rospy.Duration(2.0)
    traj.points.append(point)

    rospy.loginfo(f"Sending {action} command to {gripper_side} gripper")
    pub.publish(traj)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Open or close TIAGo Dual grippers"
    )
    parser.add_argument(
        "side",
        choices=["left", "right"],
        help="Which gripper to control"
    )
    parser.add_argument(
        "action",
        choices=["open", "close"],
        help="Action to perform",
        default="open"
    )

    args = parser.parse_args()

    rospy.init_node("tiago_gripper_control")

    control_gripper(args.side, args.action)

    rospy.sleep(3)
