from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get paths to robot URDF and world files
    robot_urdf = PathJoinSubstitution([
        FindPackageShare('lidar_camera_slam'),
        'urdf',
        'robot.urdf'
    ])
    
    world_file = PathJoinSubstitution([
        FindPackageShare('lidar_camera_slam'),
        'worlds',
        'test_room.world'
    ])
    
    # Start Gazebo with custom world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world_file,
            'verbose': 'true'
        }.items()
    )
    
    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'robot',
            '-file', robot_urdf,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )
    
    # Start LiDAR mapper node
    lidar_mapper = Node(
        package='lidar_camera_slam',
        executable='lidar_mapper.py',
        name='lidar_mapper',
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        spawn_robot,
        lidar_mapper
    ]) 