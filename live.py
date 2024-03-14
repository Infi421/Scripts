#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

class OpenCVVideoPublisher:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher('video_output', Image, queue_size=10)
        self.rate = rospy.Rate(30)

    def publish_video(self, frame):
        try:
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.image_pub.publish(ros_image)
        except CvBridgeError as e:
            print(e)

    def run(self):
        cap = cv2.VideoCapture(0)

        while not rospy.is_shutdown():
            ret, frame = cap.read()
            if ret:
                self.publish_video(frame)
                self.rate.sleep()

        cap.release()

if __name__ == '__main__':
    rospy.init_node('opencv_video_publisher', anonymous=True)
    node = OpenCVVideoPublisher()
    node.run()