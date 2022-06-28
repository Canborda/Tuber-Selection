#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2 as cv
import numpy as np


class Preprocess:

    def __init__(self):
        # OpenCV attributes
        self.bridge = CvBridge()
        # ROS attributes
        self.camera_sub = rospy.Subscriber('/cv_camera/image_raw', Image, self.cameraCallback)

    # region Callback methods

    def cameraCallback(self, img_msg: Image):
        # Convert image message
        frame = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        # Preprocess image
        images = self.hsv_threshold(frame)
        # Send images to preprocess topic
        for key, value in images.items():
            encoding = 'mono8' if len(value.shape) < 3 else 'bgr8'
            img_msg = self.bridge.cv2_to_imgmsg(value, encoding)
            pub = rospy.Publisher('/preprocess/' + key, Image, queue_size=0)
            pub.publish(img_msg)
    
    # endregion

    # region Preprocess methods

    def hsv_threshold(self, frame):
        # 1. Flip image
        flip = rospy.get_param('/img/calibration/flip')
        flip_img = cv.flip(frame, -1) if flip else frame
        # 2. Blur image
        blur = rospy.get_param('/img/calibration/blur')
        blur_img = cv.blur(flip_img, (blur, blur), 0)
        # 3. Convert into HSV space
        hue_img = cv.cvtColor(blur_img, cv.COLOR_BGR2HSV)[:,:,0]
        # 4. Filter hue range
        hue_min = rospy.get_param('/img/calibration/hue/min')
        hue_max = rospy.get_param('/img/calibration/hue/max')
        mask_min = hue_img >= hue_min
        mask_max = hue_img <= hue_max
        mask = (mask_min * mask_max) if hue_min < hue_max else (mask_min + mask_max)
        mask = mask.astype(np.uint8)
        # 5. Apply threshold
        _, th_img = cv.threshold(mask, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)
        # ~. Return images
        return {'flipped': flip_img, 'blurred': blur_img, 'hue': hue_img, 'threshold': th_img}

    # endregion


if __name__ == '__main__':
    # Create node
    node_name = 'preprocess_node'
    rospy.init_node(node_name, anonymous=False)
    rospy.loginfo(f'>> STATUS: Node \"{node_name}\" initialized.')
    # Create preprocess class
    preprocess = Preprocess()
    rospy.spin()
