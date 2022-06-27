#!/usr/bin/env python3

import sys
import rospy
from std_msgs.msg import Int8MultiArray

from models.menus.enums import *
from models.menus.submenu import Submenu

from pynput.keyboard import Key, Listener


class Cli:

    def __init__(self):
        self.mainMenu = self.__buildMenu()
        self.publisher = rospy.Publisher('/menu_topic', Int8MultiArray, queue_size=0)

    def __buildMenu(self):
        # Main menu
        menu = Submenu('MAIN MENU', MainMenu)
        # Calibration menu
        menu.attachSubmenu(MainMenu.Calibration, Submenu('Calibration Options', CalibrationMenu))
        menu.getSubmenu(MainMenu.Calibration) \
            .attachParam(CalibrationMenu.Flip, 'flip') \
            .attachParam(CalibrationMenu.Blur, 'blur') \
            .attachSubmenu(CalibrationMenu.Hue, Submenu('Hue Limits', HueMenu)) \
            .attachSubmenu(CalibrationMenu.Region, Submenu('Centroid Region Limits', RegionMenu)) \
            .attachParam(CalibrationMenu.Bounding_box_margin, 'bb_margin')
        menu.getSubmenu(MainMenu.Calibration).getSubmenu(CalibrationMenu.Hue) \
            .attachParam(HueMenu.Min, 'hue_min') \
            .attachParam(HueMenu.Max, 'hue_max')
        menu.getSubmenu(MainMenu.Calibration).getSubmenu(CalibrationMenu.Region) \
            .attachParam(RegionMenu.Center, 'region_center') \
            .attachParam(RegionMenu.Size, 'region_size')
        # Classification menu
        menu.attachSubmenu(MainMenu.Classification, Submenu('Classification Options', ClassificationMenu))
        menu.getSubmenu(MainMenu.Classification) \
            .attachSubmenu(ClassificationMenu.Set_network, Submenu('Convolutional Neural Network Options', SetNetworkMenu))
        # Settings menu
        menu.attachSubmenu(MainMenu.Settings, Submenu('Global Settings', SettingsMenu))
        menu.getSubmenu(MainMenu.Settings) \
            .attachParam(SettingsMenu.Image_size, 'img_size')
        # Return built menu
        return menu

    def publishMessage(self):
        msg = Int8MultiArray()
        msg.data = list(map(lambda x: x[1].value, self.mainMenu.getActiveMenu().getPath()))
        msg.data.append(self.mainMenu.getActiveMenu().getCurrent().value)
        self.publisher.publish(msg)

    def keyPressed(self, key):
        # Seek events
        if key == Key.esc: sys.exit()
        elif key == Key.enter:
            self.mainMenu.getActiveMenu().enterSubmenu()
            self.publishMessage()
        elif key == Key.backspace:
            self.mainMenu.getActiveMenu().exitSubmenu()
            self.publishMessage()
        elif key == Key.up:
            self.mainMenu.getActiveMenu().decrease()
        elif key == Key.down:
            self.mainMenu.getActiveMenu().increase()
        elif key == Key.left:
            # TODO add decrease param value
            self.publishMessage()
        elif key == Key.right:
            # TODO add increase param value
            self.publishMessage()
        # Show pressed key
        self.mainMenu.getActiveMenu().show(f"{str(key)} pressed")

    def keyReleased(self, key):
        pass


if __name__ == '__main__':
    # Run node
    node_name = 'cli_node'
    rospy.init_node(node_name)
    rospy.loginfo(f'>> STATUS: Node \"{node_name}\" initialized.')
    rospy.sleep(1)
    # Start listener
    cli = Cli()
    cli.mainMenu.show()
    #FIXME clear terminal memory when exit program
    try:
        with Listener(on_press=cli.keyPressed, on_release=cli.keyReleased) as listener:
            listener.join()
    except Exception as e:
        rospy.logwarn('Program finished...')
        rospy.logerr(e)
