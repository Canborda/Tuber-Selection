#!/usr/bin/env python3

import os
import yaml

import rospy
from std_msgs.msg import Int8MultiArray

from models.menus.enums import CalibrationMenu, MainMenu


class Storage:

    def __init__(self):
        self.SELECTED = False
        self.prev_path = None
        # File attributes
        self.image_params = self.loadImageParams()
        # ROS attributes
        self.menu_sub = rospy.Subscriber('/menu_topic', Int8MultiArray, self.menuCallback)

    # region Callback method

    def menuCallback(self, list_msg: Int8MultiArray):
        self.SELECTED = len(list_msg.data) > 1 \
            and list_msg.data[0] == MainMenu.Calibration.value \
            and list_msg.data[1] == CalibrationMenu.Save_params.value
        if self.SELECTED and list_msg.data == self.prev_path:
            self.saveImageParams()
        self.prev_path = list_msg.data

    # endregion

    # region Storage methods

    def buildDict(self, prefix: str, src: dict, dst: dict):
        for key, value in src.items():
            name = prefix + '/' + key
            if isinstance(value, dict):
                dst[key] = {}
                self.buildDict(name, value, dst[key])
            else:
                dst[key] = rospy.get_param(name)

    def loadImageParams(self):
        file = open(os.path.join(os.path.dirname(__file__), '../config/image_params.yaml'))
        return yaml.load(file, Loader=yaml.FullLoader)

    def saveImageParams(self):
        # Build dictionary
        print(self.image_params)
        new_params = {}
        self.buildDict('/img', self.image_params, new_params)
        print('UPDATED PARAMS')
        print(new_params)
        # Write file
        file = open(os.path.join(os.path.dirname(__file__), '../config/image_params.yaml'), 'w')
        self.yaml.dump(new_params, file)
        # Log
        rospy.logwarn('Stored new params in /config/image_params.yaml')
        rospy.logwarn(new_params)

    # endregion


if __name__ == '__main__':
    # Create node
    node_name = 'storage_node'
    rospy.init_node(node_name, anonymous=False)
    rospy.loginfo(f'>> STATUS: Node \"{node_name}\" initialized.')
    # Create storage class
    storage = Storage()
    rospy.spin()
