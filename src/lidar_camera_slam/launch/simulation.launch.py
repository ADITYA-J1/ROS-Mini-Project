from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Path to the world file
    world_path = PathJoinSubstitution([
        FindPackageShare('lidar_camera_slam'),
        'worlds',
        'test_room.world'
    ])

    # Path to the robot URDF
    urdf_path = PathJoinSubstitution([
        FindPackageShare('lidar_camera_slam'),
        'urdf',
        'robot.urdf'
    ])

    # Path to RViz config
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('lidar_camera_slam'),
        'rviz',
        'slam.rviz'
    ])

    # Start Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world_path,
            'verbose': 'true'
        }.items()
    )

    # Spawn the robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'robot',
            '-file', urdf_path,
            '-x', '0',
            '-y', '0',
            '-z', '0.1'
        ],
        output='screen'
    )

    # Start RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # Start SLAM node
    slam_node = Node(
        package='lidar_camera_slam',
        executable='slam_node',
        name='slam_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gazebo,
        spawn_robot,
        rviz,
        slam_node
    ]) 