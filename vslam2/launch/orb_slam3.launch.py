import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package share directory
    vslam2_dir = get_package_share_directory('vslam2')

    # Define launch file paths
    orb_slam3_mono_launch = os.path.join(vslam2_dir, 'launch/orb_slam3_.launch.py')
    rviz_config = os.path.join(vslam2_dir, 'config/orb_slam3_no_imu.rviz')

    # Create launch description
    ld = LaunchDescription()

    # Launch ORB-SLAM3 ROS wrapper node
    orb_slam3_mono_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(orb_slam3_mono_launch),
        launch_arguments={
            'voc_file': LaunchConfiguration('voc_file'),
            'settings_file': LaunchConfiguration('settings_file'),
            'world_frame_id': LaunchConfiguration('world_frame_id'),
            'cam_frame_id': LaunchConfiguration('cam_frame_id'),
            'enable_pangolin': LaunchConfiguration('enable_pangolin'),
            'world_roll': LaunchConfiguration('world_roll'),
            'world_pitch': LaunchConfiguration('world_pitch'),
            'world_yaw': LaunchConfiguration('world_yaw')
        }.items()
    )

    ld.add_action(orb_slam3_mono_node)

    # Launch RViz
    rviz_node = launch.actions.ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config],
        output='screen'
    )

    ld.add_action(rviz_node)

    # Launch Hector Trajectory Server
    trajectory_server_node = launch.actions.ExecuteProcess(
        cmd=['ros2', 'run', 'hector_trajectory_server', 'hector_trajectory_server', '--ros-args', '--params-file', os.path.join(orb_slam3_ros_wrapper_dir, 'config/hector_trajectory_server.yaml')],
        output='screen'
    )

    ld.add_action(trajectory_server_node)

    return ld

if __name__ == '__main__':
    generate_launch_description()
