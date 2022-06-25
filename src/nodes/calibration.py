#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int8

def menuCallback(data):
    a = rospy.get_param('/calibration/img_size')
    print(a)

if __name__ == '__main__':
    # Create node
    node_name = 'calibration_node'
    rospy.init_node(node_name, anonymous=False)
    rospy.loginfo(f'>> STATUS: Node \"{node_name}\" initialized.')
    # Create basic subscriber
    rospy.Subscriber('/menu_topic', Int8, menuCallback)
    rospy.spin()