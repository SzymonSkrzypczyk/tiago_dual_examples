#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def move_tiago_arm(arm_name, joint_values):
    """
    Move TIAGo Dual arm to a given 7-DOF configuration.
    
    :param arm_name: "arm_left" or "arm_right"
    :param joint_values: list of 7 joint values (rad)
    """
    if arm_name not in ["arm_left", "arm_right"]:
        raise ValueError("arm_name must be 'arm_left' or 'arm_right'")
    if len(joint_values) != 7:
        raise ValueError("joint_values must have length 7")

    # Build joint names for the selected arm
    joint_names = [f"{arm_name}_{i}_joint" for i in range(1, 8)]

    # Publisher to the arm controller
    pub = rospy.Publisher(f"/{arm_name}_controller/command", JointTrajectory, queue_size=10)
    rospy.sleep(1)  # wait for connection

    traj = JointTrajectory()
    traj.joint_names = joint_names

    point = JointTrajectoryPoint()
    point.positions = joint_values
    point.time_from_start = rospy.Duration(3.0)  # 3s motion
    traj.points.append(point)

    rospy.loginfo(f"Sending trajectory to {arm_name}: {joint_values}")
    pub.publish(traj)


if __name__ == "__main__":
    rospy.init_node("tiago_arm_move")

    # Example usage: move left arm
    # Replace these with your desired joint values
    joint_goals = [0.0, -0.5, 0.3, 1.2, 0.0, -0.7, 0.2]
    move_tiago_arm("arm_left", joint_goals)

    rospy.sleep(4)  # wait until motion completes
