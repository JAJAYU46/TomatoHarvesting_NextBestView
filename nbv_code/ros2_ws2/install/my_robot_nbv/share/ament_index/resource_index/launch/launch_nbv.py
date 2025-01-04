import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Set environment variables
        SetEnvironmentVariable('Open3D_DIR', '/home/jajayu/open3d_install/lib/cmake/Open3D'),
        SetEnvironmentVariable('LD_LIBRARY_PATH', '/home/jajayu/open3d_install/lib:$LD_LIBRARY_PATH'),

        # Run the processes
        ExecuteProcess(
            cmd=['ros2', 'run', 'my_robot_cam', 'cam_cloudrate_transformer'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'my_robot_nbv', 'nbv_tom_detect'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'my_robot_nbv', 'nbv_tompcd_filter'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'launch', 'octomap_server2', 'octomap_server_launch.py', 'resolution:=0.01', 'frame_id:=odom', 'input_cloud_topic:=/nbv/tompcd_ICP'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'my_robot_nbv_cmake', 'BestViewModel', '2>&1', '|', 'grep', '-v', 'WARNING: Coordinate hit bounds'],
            output='screen'
        )
    ])
