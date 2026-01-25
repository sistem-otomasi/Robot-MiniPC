import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/root/Otomasi/Sistem-Otomasi-Robot/Robot/ros2_ws/install/robot_example'
