docker exec -it nbv_container2_nbv_image_fullinstall4 bash
source ./install/setup.bash

ros2 launch realsense2_camera rs_pointcloud_launch.py

ros2 run my_robot_cam cam_cloudrate_transformer

ros2 run my_robot_nbv nbv_tom_detect
ros2 run my_robot_nbv nbv_tompcd_filter
ros2 launch octomap_server2 octomap_server_launch.py  resolution:=0.003 frame_id:=base input_cloud_topic:=/nbv/tompcd_ICP
ros2 run my_robot_nbv_cmake BestViewModel 2>&1 | grep -v "WARNING: Coordinate hit bounds"

 
ros2 run my_robot_nbv nbv_system_controller
rviz2 -d src/my_bot/config/camera_bot3.rviz


ros2 run my_robot_arm arm_move_controller
---
cd thr_ws
# rmml05@ubuntu:~/thr_ws$
ros2 launch thr_robot_bringup teleoperation.launch.py 
ros2 run tm_driver tm_driver robot_ip:=192.168.1.100



ros2 run my_robot_nbv nbv_temp_isMovingPublisher
ros2 topic echo /nbv/status_communicator
