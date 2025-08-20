#!/usr/bin/env python3
import rospy
import argparse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def move_torso(height):
    """
    Move TIAGo torso to a given height.

    :param height: desired torso height in meters (up to 0.3m should have effect)
    """
    joint_name = ["torso_lift_joint"]

    # Publisher to the torso controller
    pub = rospy.Publisher(
        "/torso_controller/command",
        JointTrajectory,
        queue_size=10
    )
    rospy.sleep(1)  # wait for connection

    traj = JointTrajectory()
    traj.joint_names = joint_name

    point = JointTrajectoryPoint()
    point.positions = [height]
    point.time_from_start = rospy.Duration(3.0) 
    traj.points.append(point)

    rospy.loginfo(f"Moving torso to height: {height:.2f} m")
    pub.publish(traj)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Move TIAGo torso to a given height"
    )
    parser.add_argument(
        "height",
        type=float,
        help="Torso height in meters"
    )

    args = parser.parse_args()

    rospy.init_node("tiago_torso_move")

    move_torso(args.height)

    rospy.sleep(4)
