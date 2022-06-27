from models.menus.enums import *
from models.menus.submenu import Submenu

# ----- EXAMPLE OF BUILD -----

menu = Submenu('MAIN MENU', MainMenu)

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


menu.attachSubmenu(MainMenu.Classification, Submenu('Classification Options', ClassificationMenu))
menu.attachSubmenu(MainMenu.Settings, Submenu('Global Settings', SettingsMenu))

# ----- EXAMPLE OF USAGE -----

menu.getActiveMenu().increase().enterSubmenu()

menu.getActiveMenu().increase().increase().enterSubmenu()

menu.getActiveMenu().increase().enterSubmenu()

menu.getActiveMenu().exitSubmenu()

print('current active:', menu.getActiveMenu().getName())
print('submenus:', menu.getActiveMenu().getSubmenus())
print('params:', menu.getActiveMenu().getParams())

print(menu.getActiveMenu().getPath())