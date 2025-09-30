import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/agustifelo/universidad_agusti/tercero/proyecto/sprint_1/prii3_ws/install/prii3_turtlesim'
