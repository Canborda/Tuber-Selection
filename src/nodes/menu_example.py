from models.menus.enums import *
from models.menus.submenu import Submenu
from models.menus.params import BoolParam, FloatParam, IntegerParam, EnumParam

# ----- EXAMPLE OF BUILD -----

menu = Submenu('MAIN MENU', MainMenu)

menu.attachSubmenu(MainMenu.Calibration, Submenu('Calibration Options', CalibrationMenu))
menu.getSubmenu(MainMenu.Calibration) \
    .attachParam(BoolParam(CalibrationMenu.Flip, '/calibration/flip')) \
    .attachParam(IntegerParam(CalibrationMenu.Blur, '/calibration/blur', min=3, max=50, step=2)) \
    .attachSubmenu(CalibrationMenu.Hue, Submenu('Hue Limits', HueMenu)) \
    .attachSubmenu(CalibrationMenu.Region, Submenu('Centroid Region Limits', RegionMenu)) \
    .attachParam(IntegerParam(CalibrationMenu.Bounding_box_margin, '/calibration/bb_margin', min=0, max=50, step=5))

menu.getSubmenu(MainMenu.Calibration).getSubmenu(CalibrationMenu.Hue) \
    .attachParam(IntegerParam(HueMenu.Min, '/calibration/hue/min', min=0, max=179, step=1)) \
    .attachParam(IntegerParam(HueMenu.Max, '/calibration/hue/max', min=0, max=179, step=1))
menu.getSubmenu(MainMenu.Calibration).getSubmenu(CalibrationMenu.Region) \
    .attachParam(FloatParam(RegionMenu.Center, '/calibration/region/center', min=0.4, max=0.6, step=0.01)) \
    .attachParam(FloatParam(RegionMenu.Size, '/calibration/region/size', min=0.1, max=0.5, step=0.01))


menu.attachSubmenu(MainMenu.Classification, Submenu('Classification Options', ClassificationMenu))
menu.getSubmenu(MainMenu.Classification) \
    .attachParam(EnumParam(ClassificationMenu.Set_network, '/classification/network', enum=NetworkType))

menu.attachSubmenu(MainMenu.Settings, Submenu('Global Settings', SettingsMenu))

# ----- SUBMENUS EXAMPLE OF USAGE -----

menu.getActiveMenu().increase().enterSubmenu()

menu.getActiveMenu().increase().increase().enterSubmenu()

menu.getActiveMenu().increase().enterSubmenu()

menu.getActiveMenu().exitSubmenu()

print('current active:', menu.getActiveMenu().getName())
print('submenus:', menu.getActiveMenu().getSubmenus())
print('params:', menu.getActiveMenu().getParams())
print('path:', menu.getActiveMenu().getPath())

# ----- PARAMS EXAMPLE OF USAGE -----

menu.getActiveMenu().decrease()

print(menu.getActiveMenu().getParams())
print(menu.getActiveMenu().getCurrentParam().get())

menu.getActiveMenu().getCurrentParam().increase()
menu.getActiveMenu().getCurrentParam().increase()
menu.getActiveMenu().getCurrentParam().increase()
menu.getActiveMenu().getCurrentParam().increase()

print(menu.getActiveMenu().getCurrentParam().get())

# ----- ENUM EXAMPLE OF USAGE -----

menu.getActiveMenu().exitSubmenu()

menu.getActiveMenu().increase().enterSubmenu()

print(menu.getActiveMenu().getCurrentParam().get())
menu.getActiveMenu().getCurrentParam().decrease()
print(menu.getActiveMenu().getCurrentParam().get())
