from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Path to the robot URDF
    urdf_path = PathJoinSubstitution([
        FindPackageShare('lidar_camera_slam'),
        'urdf',
        'robot.urdf'
    ])

    # Path to the world file
    world_path = PathJoinSubstitution([
        FindPackageShare('lidar_camera_slam'),
        'worlds',
        'test_room.world'
    ])

    # Start Gazebo with our custom world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'verbose': 'true',
            'pause': 'false',
            'world': world_path
        }.items()
    )

    # Spawn the robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'wheeled_robot',
            '-file', urdf_path,
            '-x', '0',
            '-y', '0',
            '-z', '0.1',  # Height of wheel radius
            '-R', '0',    # Roll
            '-P', '0',    # Pitch
            '-Y', '0'     # Yaw
        ],
        output='screen',
        respawn=False
    )

    # Point cloud converter node
    point_cloud_converter = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/scan'),
            ('scan', '/point_cloud')
        ],
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.01,
            'min_height': -1.0,  # Allow detection of objects below the robot
            'max_height': 2.0,   # Increased to detect taller objects
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0087,
            'scan_time': 0.1,
            'range_min': 0.1,
            'range_max': 10.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }]
    )

    # RViz node for visualization
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('lidar_camera_slam'),
            'rviz',
            'robot.rviz'
        ])]
    )

    return LaunchDescription([
        gazebo,
        spawn_robot,
        point_cloud_converter,
        rviz
    ]) 