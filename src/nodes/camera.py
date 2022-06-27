#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64

TEMP = 1

def captureImage():
    time = rospy.get_time()
    return time

def publishMessage(rate: rospy.Rate):
    # Declare and fill message
    msg = Float64()
    msg.data = captureImage()
    # Create publisher and publish current image
    pub = rospy.Publisher('/img_topic', Float64, queue_size=0)
    pub.publish(msg)
    rate.sleep()

if __name__ == '__main__':
    # Create node
    node_name = 'camera'
    rospy.init_node(node_name)
    rospy.loginfo(f'>> STATUS: Node \"{node_name}\" initialized.')
    # Create a time rate
    rate = rospy.Rate(1)
    try:
        while not rospy.is_shutdown():
            publishMessage(rate)
    except rospy.ROSInterruptException:
        pass