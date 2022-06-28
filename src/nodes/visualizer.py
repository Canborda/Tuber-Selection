#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2 as cv


class Visualizer:

    def __init__(self):
        # OpenCV attributes
        self.bridge = CvBridge()
        # ROS attributes
        self.visualizer_sub = rospy.Subscriber('/visualizer_topic', Image, self.imageCallback)

    def imageCallback(self, img_msg):
        image = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        cv.namedWindow('Camera', cv.WINDOW_NORMAL)
        cv.setWindowProperty('Camera', cv.WND_PROP_FULLSCREEN, cv.WINDOW_FULLSCREEN)
        cv.imshow('Camera', image)
        cv.waitKey(1)
        print('Timestamp:', rospy.get_rostime(), end='\r')


if __name__ == '__main__':
    # Create node
    node_name = 'visualizer_node'
    rospy.init_node(node_name, anonymous=False)
    rospy.loginfo(f'>> STATUS: Node \"{node_name}\" initialized.')
    # Create visualizer class
    visualizer = Visualizer()
    rospy.spin()
