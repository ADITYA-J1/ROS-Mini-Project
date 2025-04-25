from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # SLAM Node
        Node(
            package='lidar_camera_slam',
            executable='slam_node',
            name='slam_node',
            output='screen',
            parameters=[{
                'use_sim_time': False,
            }]
        ),
        
        # Object Detection Node
        Node(
            package='lidar_camera_slam',
            executable='object_detection_node',
            name='object_detection_node',
            output='screen',
            parameters=[{
                'use_sim_time': False,
            }]
        ),
    ]) 