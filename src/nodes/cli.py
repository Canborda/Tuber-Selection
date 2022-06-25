#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int8

import os, sys
from py_console import console, bgColor, textColor
from pynput.keyboard import Key, Listener

CURRENT_OPTION = 1
OPTIONS = ['1 - Calibration', '2 - Classification', '3 - Exit']

def updateScreen(key=None):
    # Restart screen
    os.system('clear')
    # Show start message
    console.log('-'*65)
    console.log(f" Press {console.highlight('UP')} and {console.highlight('DOWN')} arrows to move between modes")
    console.log(f" Press {console.highlight('ENTER')} to select mode")
    console.log('-'*65)
    console.warn(f"Pressed key: {key}" if key else "Press a key!")
    console.log('-'*65)
    # Highlight selected option
    for option in OPTIONS:
        bg = bgColor.GREEN if int(option[0]) == CURRENT_OPTION else ''
        console.log(console.highlight(option, bgColor=bg, textColor=textColor.WHITE))
    console.log('-'*65)

def updateOption(key):
    global CURRENT_OPTION
    if key == Key.down: CURRENT_OPTION = CURRENT_OPTION + 1 if CURRENT_OPTION < len(OPTIONS) else 1
    if key == Key.up: CURRENT_OPTION = CURRENT_OPTION - 1 if CURRENT_OPTION > 1 else len(OPTIONS)

def publishMessage():
    # Declare and fill message
    msg = Int8()
    msg.data = CURRENT_OPTION
    # Create publisher and publish current option
    pub = rospy.Publisher('/menu_topic', Int8, queue_size=0)
    pub.publish(msg)

def keyPressed(key):
    # Seek events
    if key == Key.enter and CURRENT_OPTION == 3: sys.exit()
    if key == Key.enter: publishMessage()
    # Run events
    updateOption(key)
    updateScreen(key)

def keyReleased(key):
    pass

if __name__ == '__main__':
    # Run node
    node_name = 'cli_node'
    rospy.init_node(node_name)
    rospy.loginfo(f'>> STATUS: Node \"{node_name}\" initialized.')
    rospy.sleep(1)
    # Disable timestamp for logger
    console.setShowTimeDefault(False)
    # Print first screen
    updateScreen()
    # Start listener
    try:
        with Listener(on_press=keyPressed, on_release=keyReleased) as listener:
            listener.join()

    except:
        rospy.logwarn('Program finished...')