#!/usr/bin/env python3
import rospy
import argparse
from geometry_msgs.msg import Twist

def move_base(direction, speed, duration):
    """
    Move TIAGo base.

    :param direction: "forward", "backward", "left", "right"
    :param speed: linear speed (m/s) for forward/backward, angular speed (rad/s) for turning
    :param duration: time to move in seconds
    """
    pub = rospy.Publisher("/mobile_base_controller/cmd_vel", Twist, queue_size=10)
    rospy.sleep(1)

    twist = Twist()
    if direction == "forward":
        twist.linear.x = speed
    elif direction == "backward":
        twist.linear.x = -speed
    elif direction == "left":
        twist.angular.z = speed
    elif direction == "right":
        twist.angular.z = -speed
    else:
        raise ValueError("Invalid direction. Use forward/backward/left/right.")

    rospy.loginfo(f"Moving {direction} with speed {speed} for {duration} seconds")

    rate = rospy.Rate(10)  # 10 Hz
    start_time = rospy.get_time()
    while rospy.get_time() - start_time < duration:
        pub.publish(twist)
        rate.sleep()

    # Stop the robot
    twist = Twist()
    pub.publish(twist)

    rospy.sleep(1)

    rospy.loginfo("Movement finished, stopping TIAGo.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Move TIAGo base forward/backward and turn left/right"
    )
    parser.add_argument(
        "direction",
        choices=["forward", "backward", "left", "right"],
        help="Direction to move"
    )
    parser.add_argument(
        "speed",
        type=float,
        help="Linear speed (m/s) for forward/backward, angular speed (rad/s) for turning"
    )
    parser.add_argument(
        "duration",
        type=float,
        help="Duration to move in seconds"
    )

    args = parser.parse_args()

    rospy.init_node("tiago_base_move")

    move_base(args.direction, args.speed, args.duration)

    rospy.sleep(4)
