
# from launch import LaunchDescription
# from launch_ros.actions import Node

# ros2 run my_robot_cam cam_cloudrate_transformer
# ros2 run my_robot_nbv nbv_tom_detect 
# ros2 run my_robot_nbv nbv_tompcd_filter 
# ros2 launch octomap_server2 octomap_server_launch.py  resolution:=0.01 frame_id:=odom input_cloud_topic:=/nbv/tompcd_ICP
# export Open3D_DIR=/home/jajayu/open3d_install/lib/cmake/Open3D
# export LD_LIBRARY_PATH=/home/jajayu/open3d_install/lib:$LD_LIBRARY_PATH
# ros2 run my_robot_nbv_cmake BestViewModel 2>&1 | grep -v "WARNING: Coordinate hit bounds"

from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare

# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package='turtlesim',
#             namespace='turtlesim1',
#             executable='turtlesim_node',
#             name='sim'
#         ),
#         Node(
#             package='turtlesim',
#             namespace='turtlesim2',
#             executable='turtlesim_node',
#             name='sim'
#         ),
#         Node(
#             package='turtlesim',
#             executable='mimic',
#             name='mimic',
#             remappings=[
#                 ('/input/pose', '/turtlesim1/turtle1/pose'),
#                 ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
#             ]
#         )
#     ])
def generate_launch_description():
    # Set Open3D environment variables
    os.environ['Open3D_DIR'] = '/home/jajayu/open3d_install/lib/cmake/Open3D'
    os.environ['LD_LIBRARY_PATH'] = '/home/jajayu/open3d_install/lib:' + os.environ['LD_LIBRARY_PATH']
    octomap_server_launch_file = os.path.join(
            FindPackageShare('octomap_server2').find('octomap_server2'), 'launch', 'octomap_server_launch.py'
        )
    # Define nodes to launch
    return LaunchDescription([
        Node(
            package='my_robot_cam',
            executable='cam_cloudrate_transformer',
            name='cam_cloudrate_transformer_node',
            output='screen'
        ),
        Node(
            package='my_robot_nbv',
            executable='nbv_tom_detect',
            name='nbv_tom_detect_node',
            output='screen'
        ),
        Node(
            package='my_robot_nbv',
            executable='nbv_tompcd_filter',
            name='nbv_tompcd_filter_node',
            output='screen'
        ),
        IncludeLaunchDescription(
            octomap_server_launch_file,
            launch_arguments={
                'resolution': '0.01',
                'frame_id': 'odom',
                'input_cloud_topic': '/nbv/tompcd_ICP'
            }.items()
        ),
        # Node(
        #     package='octomap_server2',
        #     executable='octomap_server_launch.py',
        #     name='octomap_server_node',
        #     parameters=[{'resolution': 0.01, 'frame_id': 'odom', 'input_cloud_topic': '/nbv/tompcd_ICP'}],
        #     output='screen'
        # ),
        Node(
            package='my_robot_nbv_cmake',
            executable='BestViewModel',
            name='best_view_model_node',
            output='screen'
        ),
        # LogInfo for any command output if needed
        LogInfo(
            condition=None,  # Replace with actual condition if required
            msg="ROS 2 launch started"
        )
    ])


# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package='turtlesim',
#             namespace='turtlesim1',
#             executable='turtlesim_node',
#             name='sim'
#         ),
#         Node(
#             package='turtlesim',
#             namespace='turtlesim2',
#             executable='turtlesim_node',
#             name='sim'
#         ),
#         Node(
#             package='turtlesim',
#             executable='mimic',
#             name='mimic',
#             remappings=[
#                 ('/input/pose', '/turtlesim1/turtle1/pose'),
#                 ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
#             ]
#         )
#     ])




# import launch
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, LogInfo, SetEnvironmentVariable, ExecuteProcess
# from launch.substitutions import LaunchConfiguration

# def generate_launch_description():
#     return LaunchDescription([
#         # Set environment variables
#         SetEnvironmentVariable('Open3D_DIR', '/home/jajayu/open3d_install/lib/cmake/Open3D'),
#         SetEnvironmentVariable('LD_LIBRARY_PATH', '/home/jajayu/open3d_install/lib:$LD_LIBRARY_PATH'),

#         # Run the processes
#         ExecuteProcess(
#             cmd=['ros2', 'run', 'my_robot_cam', 'cam_cloudrate_transformer'],
#             output='screen'
#         ),
#         ExecuteProcess(
#             cmd=['ros2', 'run', 'my_robot_nbv', 'nbv_tom_detect'],
#             output='screen'
#         ),
#         ExecuteProcess(
#             cmd=['ros2', 'run', 'my_robot_nbv', 'nbv_tompcd_filter'],
#             output='screen'
#         ),
#         ExecuteProcess(
#             cmd=['ros2', 'launch', 'octomap_server2', 'octomap_server_launch.py', 'resolution:=0.01', 'frame_id:=odom', 'input_cloud_topic:=/nbv/tompcd_ICP'],
#             output='screen'
#         ),
#         ExecuteProcess(
#             cmd=['ros2', 'run', 'my_robot_nbv_cmake', 'BestViewModel', '2>&1', '|', 'grep', '-v', 'WARNING: Coordinate hit bounds'],
#             output='screen'
#         )
#     ])
