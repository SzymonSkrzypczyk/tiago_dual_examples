#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraViewer:
    def __init__(self, topic_name):
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(topic_name, Image, self.image_callback)
        self.cv_image = None

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr(f"CV Bridge error: {e}")

    def show(self):
        if self.cv_image is not None:
            cv2.imshow("TIAGo Camera", self.cv_image)
            cv2.waitKey(1)


if __name__ == "__main__":
    rospy.init_node("camera_viewer")

    # most probable camera topic
    topic = rospy.get_param("~camera_topic", "/xtion/rgb/image_raw")
    rospy.loginfo(f"Subscribing to camera topic: {topic}")

    viewer = CameraViewer(topic)

    rate = rospy.Rate(30)  # 30 Hz refresh
    while not rospy.is_shutdown():
        viewer.show()
        rate.sleep()

    cv2.destroyAllWindows()
