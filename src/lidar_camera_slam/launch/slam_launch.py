from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    pkg_dir = get_package_share_directory('lidar_camera_slam')

    # Include the Gazebo launch file with our custom world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': os.path.join(pkg_dir, 'worlds', 'simple_world.world')
        }.items()
    )

    # Spawn the robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'slam_bot', '-file', os.path.join(pkg_dir, 'urdf', 'robot.urdf')],
        output='screen'
    )

    # SLAM node
    slam_node = Node(
        package='lidar_camera_slam',
        executable='slam_node',
        name='slam_node',
        output='screen'
    )

    # Object detection node
    object_detection_node = Node(
        package='lidar_camera_slam',
        executable='object_detection_node',
        name='object_detection_node',
        output='screen'
    )

    # RGB camera node
    rgb_camera_node = Node(
        package='lidar_camera_slam',
        executable='rgb_camera_node',
        name='rgb_camera_node',
        output='screen'
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'slam.rviz')],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        spawn_robot,
        slam_node,
        object_detection_node,
        rgb_camera_node,
        rviz_node
    ])

