#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int8MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import os
import math
import cv2 as cv
import numpy as np

from models.menus.enums import CalibrationMenu, MainMenu


class Calibration:

    def __init__(self):
        self.SELECTED = False
        self.option = CalibrationMenu(1).name
        # OpenCV attributes
        self.bridge = CvBridge()
        self.hue_palette = cv.resize(cv.imread(os.path.join(os.path.dirname(__file__), '../../assets/hue_palette.png'), cv.IMREAD_COLOR),(300,300))
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
            # Add calibration title
            text = ' CALIBRATING...'
            textsize = cv.getTextSize(text, cv.FONT_HERSHEY_COMPLEX, 2, 4)[0]
            cv.putText(frame, text, (10,textsize[1]+10), cv.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 6)
            # Add calibration visualization
            if self.option == CalibrationMenu.Flip: frame = self.calibrateFlip(frame)
            if self.option == CalibrationMenu.Blur: frame = self.calibrateBlur(frame)
            if self.option == CalibrationMenu.Hue: frame = self.calibrateHue(frame)
            if self.option == CalibrationMenu.Region: frame = self.calibrateRegion(frame)
            # Send image to visualizer node
            img_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            self.visualizer_pub.publish(img_msg)
    
    # endregion

    # region Calibration methods

    def calibrateFlip(self, frame):
        # Get loaded param
        flip = rospy.get_param('/img/calibration/flip')
        # Draw param info
        text = 'Flip = ' + str(flip)
        cv.putText(frame, text, (10, 100), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        # Return transformed image
        return frame

    def calibrateBlur(self, frame):
        # Get loaded param
        blur = rospy.get_param('/img/calibration/blur')
        # Draw param info
        text = 'Blur = ' + str(blur)
        cv.putText(frame, text, (10, 100), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        # Return transformed image
        return frame

    def calibrateHue(self, frame):
        h, w, c = self.hue_palette.shape
        M = 10
        RADIUS = h//2
        # Get loaded params
        hue_min = rospy.get_param('/img/calibration/hue/min')
        hue_max = rospy.get_param('/img/calibration/hue/max')
        # Draw palette image
        img = np.zeros(frame.shape, dtype=np.uint8) * 255
        img[-h-M:-M,M:w+M,:] = self.hue_palette
        frame = cv.addWeighted(img, 1.0, frame, 1.0, 0.0)
        # Draw palette lines
        CENTER = (w//2+M, img.shape[0]-h//2-M)
        ang_min = -hue_min * 2
        ang_max = -hue_max * 2
        pt_min = (
            CENTER[0] + int(RADIUS * math.cos(ang_min * math.pi / 180)),
            CENTER[1] + int(RADIUS * math.sin(ang_min * math.pi / 180))
        )
        pt_max = (
            CENTER[0] + int(RADIUS * math.cos(ang_max * math.pi / 180)),
            CENTER[1] + int(RADIUS * math.sin(ang_max * math.pi / 180))
        )
        cv.line(frame, CENTER, pt_min, (100, 100, 100), 2)
        cv.line(frame, CENTER, pt_max, (100, 100, 100), 2)
        # Draw params info
        text = 'Hue min = ' + str(hue_min)
        cv.putText(frame, text, (10, 100), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        text = 'Hue max = ' + str(hue_max)
        cv.putText(frame, text, (10, 150), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        # Return transformed image
        return frame
    
    def calibrateRegion(self, frame):
        # Get loaded params
        region_center = rospy.get_param('/img/calibration/region/center')
        region_size = rospy.get_param('/img/calibration/region/size')
        # Draw region lines
        h = int(frame.shape[0] * region_center)
        s = int(frame.shape[0] * region_size/2)
        cv.line(frame, (0,h), (frame.shape[1], h), (0,0,255), 2, cv.LINE_4)
        cv.line(frame, (0,h-s), (frame.shape[1], h-s), (0,255,0), 2, cv.LINE_4)
        cv.line(frame, (0,h+s), (frame.shape[1], h+s), (0,255,0), 2, cv.LINE_4)
        # Draw params info
        text = 'Region center = ' + str(round(region_center, 4))
        cv.putText(frame, text, (10, 100), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        text = 'Region size = ' + str(round(region_size, 4))
        cv.putText(frame, text, (10, 150), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        # Return transformed image
        return frame

    # endregion


if __name__ == '__main__':
    # Create node
    node_name = 'calibration_node'
    rospy.init_node(node_name, anonymous=False)
    rospy.loginfo(f'>> STATUS: Node \"{node_name}\" initialized.')
    # Create calibration class
    calibration = Calibration()
    rospy.spin()
