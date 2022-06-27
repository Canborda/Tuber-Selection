#!/usr/bin/env python3

import sys
import rospy
from std_msgs.msg import Int8MultiArray

from models.menus.enums import *
from models.menus.submenu import Submenu
from models.menus.params import BoolParam, FloatParam, IntegerParam, EnumParam

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
            .attachParam(BoolParam(CalibrationMenu.Flip, '/img/calibration/flip')) \
            .attachParam(IntegerParam(CalibrationMenu.Blur, '/img/calibration/blur', min=3, max=50, step=2)) \
            .attachSubmenu(CalibrationMenu.Hue, Submenu('Hue Limits', HueMenu)) \
            .attachSubmenu(CalibrationMenu.Region, Submenu('Centroid Region Limits', RegionMenu)) \
            .attachParam(IntegerParam(CalibrationMenu.Bounding_box_margin, '/img/calibration/bb_margin', min=0, max=50, step=5))
        menu.getSubmenu(MainMenu.Calibration).getSubmenu(CalibrationMenu.Hue) \
            .attachParam(IntegerParam(HueMenu.Min, '/img/calibration/hue/min', min=0, max=179, step=1)) \
            .attachParam(IntegerParam(HueMenu.Max, '/img/calibration/hue/max', min=0, max=179, step=1))
        menu.getSubmenu(MainMenu.Calibration).getSubmenu(CalibrationMenu.Region) \
            .attachParam(FloatParam(RegionMenu.Center, '/img/calibration/region/center', min=0.4, max=0.6, step=0.01)) \
            .attachParam(FloatParam(RegionMenu.Size, '/img/calibration/region/size', min=0.1, max=0.5, step=0.01))
        # Classification menu
        menu.attachSubmenu(MainMenu.Classification, Submenu('Classification Options', ClassificationMenu))
        menu.getSubmenu(MainMenu.Classification) \
            .attachParam(EnumParam(ClassificationMenu.Set_network, '/img/classification/network', enum=NetworkType))
        # Settings menu
        menu.attachSubmenu(MainMenu.Settings, Submenu('Global Settings', SettingsMenu))
        menu.getSubmenu(MainMenu.Settings) \
            .attachParam(IntegerParam(SettingsMenu.Image_size, '/img/classification/img_size', min=150, max=300, step=5))
        # Return built menu
        return menu

    def publishMessage(self):
        msg = Int8MultiArray()
        msg.data = list(map(lambda x: x[1].value, self.mainMenu.getActiveMenu().getPath()))
        msg.data.append(self.mainMenu.getActiveMenu().getCurrent().value)
        self.publisher.publish(msg)

    def keyPressed(self, key):
        # Get objects
        menu = self.mainMenu.getActiveMenu()
        param = menu.getCurrentParam()
        # Seek events
        if key == Key.esc: sys.exit()
        elif key == Key.enter:
            menu.enterSubmenu()
        elif key == Key.backspace:
            menu.exitSubmenu()
        elif key == Key.up:
            menu.decrease()
        elif key == Key.down:
            menu.increase()
        elif key == Key.left:
            if param: param.decrease()
        elif key == Key.right:
            if param: param.increase()
        # Publish menu path message
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
