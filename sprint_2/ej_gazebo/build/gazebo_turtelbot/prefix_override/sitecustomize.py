import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/agustifelo/universidad_agusti/tercero/proyecto/sprint_2/ej_gazebo/install/gazebo_turtelbot'
