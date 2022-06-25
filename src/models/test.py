from menu import *
from enums import *

a = Menu('MAIN MENU', MainMenu)

# print(a.__name)
a.increase()
a.increase()
a.increase()
a.increase()
a.increase()

print('current option:', a.getCurrentName(), a.getCurrentValue())

a.show()

# b = Menu('Calibration settings', CalibrationMenu)

# b.show()