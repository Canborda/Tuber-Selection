from enum import Enum

import rospy


class BaseParam:

    def __init__(self, option: Enum, name: str):
        self.option = option
        self.name = name
        self.__value = self.getROSParam()

    # region Inheritance methods

    def increase(self):
        pass

    def decrease(self):
        pass

    def get(self):
        return self.__value

    # endregion

    # region ROS methods

    def getROSParam(self):
        return rospy.get_param(self.name)

    def setROSParam(self, value):
        rospy.set_param(self.name, value)

    # endregion


class BoolParam(BaseParam):

    def __init__(self, option: Enum, name: str):
        super().__init__(option, name)
        self.__value: bool = self.getROSParam()

    def increase(self):
        self.__value = not self.__value
        self.setROSParam(self.__value)
        return self

    def decrease(self):
        self.__value = not self.__value
        self.setROSParam(self.__value)
        return self

    def get(self):
        return self.__value


class IntegerParam(BaseParam):

    def __init__(self, option: Enum, name: str, **kwargs):
        super().__init__(option, name)
        self.__value: int = self.getROSParam()
        self.__min: int = kwargs['min']
        self.__max: int = kwargs['max']
        self.__step: int = kwargs['step']

    def increase(self):
        self.__value = self.__value + self.__step if self.__value < self.__max else self.__min
        self.setROSParam(self.__value)
        return self

    def decrease(self):
        self.__value = self.__value - self.__step if self.__value > self.__min else self.__max
        self.setROSParam(self.__value)
        return self

    def get(self):
        return self.__value


class FloatParam(BaseParam):

    def __init__(self, option: Enum, name: str, **kwargs):
        super().__init__(option, name)
        self.__value: float = self.getROSParam()
        self.__min: float = kwargs['min']
        self.__max: float = kwargs['max']
        self.__step: float = kwargs['step']

    def increase(self):
        self.__value = self.__value + self.__step if self.__value < self.__max else self.__max
        self.setROSParam(self.__value)
        return self

    def decrease(self):
        self.__value = self.__value - self.__step if self.__value > self.__min else self.__min
        self.setROSParam(self.__value)
        return self

    def get(self):
        return self.__value


class EnumParam(BaseParam):

    def __init__(self, option: Enum, name: str, **kwargs):
        super().__init__(option, name)
        self.__value: int = self.getROSParam()
        self.__enum: Enum = kwargs['enum']

    def increase(self):
        self.__value = self.__value + 1 if self.__value < len(self.__enum) else 1
        self.setROSParam(self.__value)
        return self

    def decrease(self):
        self.__value = self.__value - 1 if self.__value > 1 else len(self.__enum)
        self.setROSParam(self.__value)
        return self

    def get(self):
        return self.__enum(self.__value).name
