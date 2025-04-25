from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('lidar_camera_slam')
    
    # Launch Gazebo with our world
    world_file = os.path.join(pkg_share, 'worlds', 'test_room.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file}.items()
    )

    # Spawn our robot in Gazebo (with delay to ensure Gazebo is ready)
    spawn_robot = TimerAction(
        period=5.0,  # delay in seconds
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-entity', 'my_robot',
                          '-file', os.path.join(pkg_share, 'urdf', 'robot.urdf'),
                          '-x', '0',
                          '-y', '0',
                          '-z', '0.1'],
                output='screen'
            )
        ]
    )

    # Launch RViz
    rviz_config = os.path.join(pkg_share, 'config', 'slam.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Our SLAM and Object Detection nodes
    slam_node = Node(
        package='lidar_camera_slam',
        executable='slam_node',
        name='slam_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    object_detection_node = Node(
        package='lidar_camera_slam',
        executable='object_detection_node',
        name='object_detection_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Image visualization node
    image_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='image_view',
        arguments=['/camera/image_raw']
    )

    return LaunchDescription([
        gazebo,
        spawn_robot,
        rviz,
        slam_node,
        object_detection_node,
        image_view
    ]) 