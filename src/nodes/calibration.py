#!/usr/bin/env python3

import os

import rospy
from std_msgs.msg import Int8MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2 as cv

from models.menus.enums import CalibrationMenu, MainMenu


class Calibration:

    def __init__(self):
        self.SELECTED = False
        self.option = CalibrationMenu(1).name
        # OpenCV attributes
        self.bridge = CvBridge()
        self.hue_palette = cv.imread(os.path.join(os.path.dirname(__file__), '../../assets/hue_palette.png'), cv.IMREAD_COLOR)
        # ROS attributes
        self.menu_sub = rospy.Subscriber('/menu_topic', Int8MultiArray, self.menuCallback)
        self.camera_sub = rospy.Subscriber('/cv_camera/image_raw', Image, self.cameraCallback)
        self.visualizer_pub = rospy.Publisher('/visualizer_topic', Image, queue_size=0)

    # region Callback methods

    def menuCallback(self, list_msg: Int8MultiArray):
        self.SELECTED = len(list_msg.data) > 1 and list_msg.data[0] == MainMenu.Calibration.value
        if self.SELECTED:
            self.option = CalibrationMenu(list_msg.data[1])


    def cameraCallback(self, img_msg: Image):
        if self.SELECTED:
            # Convert image message
            frame = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
            # TODO Add image preprocessing
            if rospy.get_param('/calibration/flip'): frame = cv.flip(frame, -1)
            # Add calibration title
            text = 'C A L I B R A T I N G . . .'
            textsize = cv.getTextSize(text, cv.FONT_HERSHEY_COMPLEX, 2, 4)[0]
            cv.putText(frame, text, (10,textsize[1]+10), cv.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 6)
            # Add calibration visualization
            if self.option ==  CalibrationMenu.Flip: self.calibrateFlip(frame)
            # Send image to visualizer node
            img_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            self.visualizer_pub.publish(img_msg)
    
    # endregion

    # region Calibration methods

    def calibrateFlip(self, frame):
        # Get loaded param
        flip = rospy.get_param('/calibration/flip')
        # Draw param info
        text = 'Flip = ' + str(flip)
        textsize = cv.getTextSize(text, cv.FONT_HERSHEY_COMPLEX, 1, 2)[0]
        cv.rectangle(frame, (10, 100), (textsize[0]+10, textsize[1]+100), (255, 0, 0), cv.FILLED)
        cv.putText(frame, text, (10,textsize[1]+100), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)


    # endregion


if __name__ == '__main__':
    # Create node
    node_name = 'calibration_node'
    rospy.init_node(node_name, anonymous=False)
    rospy.loginfo(f'>> STATUS: Node \"{node_name}\" initialized.')
    # Create calibration class
    calibration = Calibration()
    rospy.spin()
