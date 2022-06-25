from enum import Enum, auto


class MainMenu(Enum):
    Calibration = auto()
    Classification = auto()
    Configuration = auto()
    Exit = auto()


class CalibrationMenu(Enum):
    Flip = auto()
    Blur = auto()
