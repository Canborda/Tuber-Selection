import os
from enum import Enum

from py_console import console, bgColor, textColor


class BaseMenu:

    def __init__(self, name: str, options: Enum, isActive: bool = True):
        # Disable timestamp for logger
        console.setShowTimeDefault(False)
        self.__name = name
        self.__isActive = isActive
        self.__options = options
        self.__current = 1
        self.__errorMsg = None

    # region Operation methods

    def increase(self):
        self.__current = self.__current + 1 if self.__current < len(self.__options) else 1
        return self

    def decrease(self):
        self.__current = self.__current - 1 if self.__current > 1 else len(self.__options)
        return self

    def setActive(self, isActive: bool):
        self.__isActive = isActive

    def isActive(self):
        return self.__isActive

    def getCurrent(self) -> Enum:
        return self.__options(self.__current)

    # endregion

    # region Display methods

    def getName(self):
        return self.__name

    def optionStr(self, option: Enum):
        return str(option.value) + ' - ' + option.name.replace('_', ' ')

    def addError(self, message):
        self.__errorMsg = message

    def show(self, infoMsg: str = None):
        # Display params
        W = 70
        V = 15
        # Restart screen
        os.system('clear')
        # Show start message
        message = ""
        message += f"\n Press {console.highlight('UP')} and {console.highlight('DOWN')} arrows to move between options"
        message += f"\n Press {console.highlight('RIGHT')} and {console.highlight('LEFT')} arrows to change option value (if apply)"
        message += f"\n Press {console.highlight('ENTER')} to select mode or {console.highlight('BACKSPACE')} to exit mode"
        console.log('-'*W)
        console.log(message)
        console.log('-'*W)
        console.log('\033[1m' + self.__name.center(W) + '\033[0m')
        console.log('')
        # Highlight selected option
        for option in self.__options:
            bg = bgColor.GREEN if self.__current == option.value else ''
            fc = textColor.LIGHTGREEN_EX if self.__current == option.value else textColor.LIGHTBLACK_EX
            name = '    ' + str(option.value) + ' - ' + option.name.replace('_', ' ') + '    '
            value = '← ' + str(self.getParam(option).get()).center(V) + ' →' if self.getParam(option) else '↳'
            value = ' ' * (W//2 - len(name)) + value.replace('_', ' ')
            console.log(
                console.highlight(name, bgColor=bg, textColor=textColor.WHITE) +
                console.highlight(value, bgColor='', textColor=fc)
                )
        console.log('-'*W)
        # Display info message (if exists)
        console.info(' > INFO: ' + infoMsg if infoMsg else '')
        # Display error message (if exists)
        console.error(' > MENU ERROR: ' + self.__errorMsg if self.__errorMsg else '')
        self.__errorMsg = None

    # endregion
