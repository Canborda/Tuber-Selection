import os
from enum import Enum

from py_console import console, bgColor, textColor


class Menu:

    W = 80

    def __init__(self, name: str, options: Enum):
        console.setShowTimeDefault(False)
        self.__name = name
        self.__options = options
        self.__current = 1

    def increase(self):
        self.__current = self.__current + 1 if self.__current < len(self.__options) else 1

    def decrease(self):
        self.__current = self.__current - 1 if self.__current > 1 else len(self.__options)

    def getCurrentName(self):
        return self.__options(self.__current).name

    def getCurrentValue(self):
        return self.__options(self.__current).value
    
    def show(self):
        # Restart screen
        os.system('clear')
        # Show start message
        message = ""
        message += f"\n Press {console.highlight('UP')} and {console.highlight('DOWN')} arrows to move between options"
        message += f"\n Press {console.highlight('RIGHT')} and {console.highlight('LEFT')} arrows to change option value (if apply)"
        message += f"\n Press {console.highlight('ENTER')} to select mode"
        console.log('-'*self.W)
        console.log(message)
        console.log('-'*self.W)
        console.log('\033[1m' + self.__name.center(self.W) + '\033[0m')
        # Highlight selected option
        for option in self.__options:
            bg = bgColor.GREEN if self.__current == option.value else ''
            line = '  ' + str(option.value) + ' - ' + option.name
            console.log(console.highlight(line, bgColor=bg, textColor=textColor.WHITE))
        console.log('-'*self.W)
