#!/usr/bin/env python3
import rospy
import argparse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def move_head(x_angle, y_angle):
    """
    Move TIAGo Dual head to the given X and Y axis angles.

    :param x_angle: rotation around vertical axis (rad)
    :param y_angle: up/down tilt (rad)
    """
    joint_names = ["head_1_joint", "head_2_joint"]

    # Publisher to the head controller
    pub = rospy.Publisher(
        "/head_controller/command",
        JointTrajectory,
        queue_size=10
    )
    rospy.sleep(1)  # wait for connection

    traj = JointTrajectory()
    traj.joint_names = joint_names

    point = JointTrajectoryPoint()
    point.positions = [x_angle, y_angle]
    point.time_from_start = rospy.Duration(2.0)
    traj.points.append(point)

    rospy.loginfo(f"Moving head to x_angle={x_angle:.2f}, y_angle={y_angle:.2f}")
    pub.publish(traj)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Move TIAGo Dual head to a given position (yaw, pitch in radians)"
    )
    parser.add_argument(
        "yaw",
        type=float,
        help="Yaw (left/right rotation) in radians"
    )
    parser.add_argument(
        "pitch",
        type=float,
        help="Pitch (up/down tilt) in radians"
    )

    args = parser.parse_args()

    rospy.init_node("tiago_head_move")

    move_head(args.yaw, args.pitch)

    rospy.sleep(3)
