import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jajayu/TomatoHarvesting_NextBestView/nbv_code/nbv_for_local_computer/ros2_ws2_local/install/my_robot_cam'
