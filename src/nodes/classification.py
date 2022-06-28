#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int8MultiArray, Int32MultiArray
from cv_bridge import CvBridge

import cv2 as cv
import numpy as np
import tensorflow as tf

from models.cnn.network import loadModel


class Classification:

    def __init__(self):
        self.current_img = np.zeros((480,640, 3), dtype=np.uint8)
        # TensorFlow attributes
        self.model = loadModel()
        # OpenCV attributes
        self.bridge = CvBridge()
        # ROS attributes
        self.camera_sub = rospy.Subscriber('/preprocess/flipped', Image, self.cameraCallback)
        self.bboxes_sub = rospy.Subscriber('/preprocess/bboxes', Int32MultiArray, self.bboxesCallback)
        self.predictions_pub = rospy.Publisher('/classification/predictions', Image, queue_size=0)
        self.crop_pub = rospy.Publisher('/classification/cropped', Image, queue_size=0)

    # region Callback methods

    def cameraCallback(self, img_msg: Image):
        self.current_img = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')

    def bboxesCallback(self, bbox_msg: Int32MultiArray):
        bboxes = np.reshape(np.array(bbox_msg.data), (-1, 4))
        objects = self.extractImages(bboxes)
        labels = self.predictImages(objects)
        image = self.drawPredictions(bboxes, labels)
        # Publish image
        img_msg = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        self.predictions_pub.publish(img_msg)

    # endregion

    # region Classification methods

    def extractImages(self, bboxes):
        images = []
        if self.current_img.all():
            margin = rospy.get_param('/img/classification/bb_margin')
            for bbox in bboxes:
                x, y, w, h = bbox
                pt1 = [x - margin, y - margin]
                pt2 = [x + w + margin, y + h + margin]
                img = self.current_img.copy()[pt1[1]:pt2[1], pt1[0]:pt2[0], :]
                images.append(img)
                cv.rectangle(self.current_img, pt1, pt2, (255,0,0), 2)
            # TODO remove temporary publisher
            if len(images):
                img_msg = self.bridge.cv2_to_imgmsg(images[0], 'bgr8')
                self.crop_pub.publish(img_msg)
        return images
    
    def predictImages(self, images):
        predictions = []
        for img in images:
            res_img = cv.resize(img, (250, 250))
            pred = tf.keras.utils.img_to_array(res_img) / 255
            pred = np.expand_dims(pred, axis=0)
            imgs = np.vstack([pred])
            classes = self.model.predict(imgs, batch_size=1)
            predictions.append('Healthy' if classes[0, 0] > 0.5 else 'Damaged')
        return predictions
    
    def drawPredictions(self, bboxes, labels):
        if len(bboxes) and len(labels) and len(bboxes) == len(labels):
            margin = rospy.get_param('/img/classification/bb_margin')
            image = self.current_img.copy()
            for box, label in zip(bboxes, labels):
                x, y, w, h = box
                pt1 = [x - margin, y - margin]
                pt2 = [x + w + margin, y + h + margin]
                color = (0,255,0) if label == 'Healthy' else (0,0,255)
                cv.rectangle(image, pt1, pt2, color, 2)
                cv.putText(image, label, (pt1[0], pt1[1]-10), cv.FONT_HERSHEY_COMPLEX, 1, color, 2)
            return image
        else:
            return self.current_img

    # endregion


if __name__ == '__main__':
    # Create node
    node_name = 'classification_node'
    rospy.init_node(node_name, anonymous=False)
    rospy.loginfo(f'>> STATUS: Node \"{node_name}\" initialized.')
    # Create classification class
    classification = Classification()
    rospy.spin()
