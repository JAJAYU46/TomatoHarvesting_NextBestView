import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jajayu/TomatoHarvesting_NextBestView/nbv_code/ros2_ws2/install/my_robot_cam'
