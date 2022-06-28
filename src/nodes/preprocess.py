#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge

import cv2 as cv
import numpy as np


class Preprocess:

    def __init__(self):
        # OpenCV attributes
        self.bridge = CvBridge()
        # ROS attributes
        self.camera_sub = rospy.Subscriber('/cv_camera/image_raw', Image, self.cameraCallback)
        self.contours_pub = rospy.Publisher('/preprocess/contours', Image, queue_size=0)
        self.bboxes_pub = rospy.Publisher('/preprocess/bboxes', Int32MultiArray, queue_size=0)

    # region Callback methods

    def cameraCallback(self, img_msg: Image):
        # Convert image message
        frame = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        # Preprocess image
        images = self.hueThreshold(frame)
        # Detect objects
        contours_img, bboxes = self.objectDetection(images['erosion'], images['blurred'])
        images['contours'] = contours_img
        # Send images to preprocess topic
        for key, value in images.items():
            encoding = 'mono8' if len(value.shape) < 3 else 'bgr8'
            img_msg = self.bridge.cv2_to_imgmsg(value, encoding)
            pub = rospy.Publisher('/preprocess/' + key, Image, queue_size=0)
            pub.publish(img_msg)
        # Find and send contours & boxes
        self.publishMessage(bboxes)
    
    def publishMessage(self, bboxes):
        bboxes_msg = Int32MultiArray()
        bboxes_msg.data = np.array(bboxes).flatten()
        self.bboxes_pub.publish(bboxes_msg)

    # endregion

    # region Preprocess methods

    def hueThreshold(self, frame):
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
        # 6. Morphological filter
        kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (9, 9))
        erosion = cv.erode(th_img, kernel, iterations = 1)
        # ~. Return images
        return {'flipped': flip_img, 'blurred': blur_img, 'hue': hue_img, 'threshold': th_img, 'erosion': erosion}

    def objectDetection(self, th_img, src):
        # Create a copy of the source image
        contours_img = np.copy(src)
        # Find all contours
        contours, _ = cv.findContours(th_img, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        # Filter contours
        min_points = rospy.get_param('/img/calibration/pixels')
        margin = rospy.get_param('/img/classification/bb_margin')
        filtered_boxes = []
        # TODO add region filter
        for cont in contours:
            bbox = cv.boundingRect(cont)
            (x, y, w, h) = bbox
            if w > min_points and h > min_points:
                if x - margin > 0 and x + w + margin < th_img.shape[1]:
                    filtered_boxes.append(bbox)
                    cv.drawContours(contours_img, [cont], 0, (0,0,255), 2)
        # Build contours mask
        cv.putText(contours_img, f'Objects: {len(filtered_boxes)}', (src.shape[1]-100, src.shape[0]-10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)
        # Return modified image
        return contours_img, filtered_boxes

    # endregion


if __name__ == '__main__':
    # Create node
    node_name = 'preprocess_node'
    rospy.init_node(node_name, anonymous=False)
    rospy.loginfo(f'>> STATUS: Node \"{node_name}\" initialized.')
    # Create preprocess class
    preprocess = Preprocess()
    rospy.spin()
