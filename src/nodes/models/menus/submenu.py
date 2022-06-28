from enum import Enum

from models.menus.base import BaseMenu
from models.menus.params import BaseParam


class Submenu(BaseMenu):

    def __init__(self, name: str, options: Enum):
        super().__init__(name, options)
        self.__subumenus = {0: self}
        self.__params = {}
        self.__path = []

    # region Building menu methods

    def attachSubmenu(self, option: Enum, child: BaseMenu):
        child.setActive(False)
        self.__subumenus[option.value] = child
        return self

    def getSubmenu(self, option: Enum):
        try:
            return self.__subumenus[option.value]
        except KeyError:
            self.addError(f"Option '{self.optionStr(option)}' not a submenu!")

    def attachParam(self, param: BaseParam):
        self.__params[param.option.value] = param
        return self

    def getParam(self, option: Enum) -> BaseParam:
        try:
            return self.__params[option.value]
        except KeyError:
            pass

    # endregion

    # region Operation methods

    def enterSubmenu(self):
        try:
            self.getSubmenu(self.getCurrent()).setActive(True)
            self.setActive(False)
            self.getSubmenu(self.getCurrent()).__path = self.__path + [self.__subumenus[0]]
        except KeyError:
            self.addError(f"Option '{self.optionStr(self.getCurrent())}' not a submenu!")
        except AttributeError:
            pass

    def exitSubmenu(self):
        if len(self.__path):
            self.__path.pop().setActive(True)
            self.setActive(False)
        else:
            self.addError("Already on Root Menu")

    def getActiveMenu(self):
        if self.isActive():
            return self
        else:
            return self.__subumenus[self.getCurrent().value].getActiveMenu()
    
    def getCurrentParam(self) -> BaseParam:
        try:
            return self.__params[self.getCurrent().value]
        except KeyError:
            pass

    # endregion

    # region Getters

    def getSubmenus(self):
        return [self.__subumenus[key].getName() for key in self.__subumenus]

    def getParams(self):
        return self.__params

    def getPath(self):
        return [(submenu.getName(), submenu.getCurrent()) for submenu in self.__path]

    # endregion
