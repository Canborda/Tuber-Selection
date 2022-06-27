from enum import Enum, auto

# region FIRST level menu


class MainMenu(Enum):
    Start = auto()
    Calibration = auto()
    Classification = auto()
    Settings = auto()
    Exit = auto()

# endregion

# region SECOND level menus


class CalibrationMenu(Enum):
    Flip = auto()
    Blur = auto()
    Hue = auto()
    Region = auto()
    Bounding_box_margin = auto()
    Save_params = auto()


class ClassificationMenu(Enum):
    Set_network = auto()
    Show_network_summary = auto()


class SettingsMenu(Enum):
    Image_size = auto()
    OPTION_2 = auto()
    OPTION_3 = auto()

# endregion

# region THIRD level menus


class HueMenu(Enum):
    Min = auto()
    Max = auto()


class RegionMenu(Enum):
    Center = auto()
    Size = auto()


class SetNetworkMenu(Enum):
    Binary = auto()
    Multiclass = auto()

# endregion
