from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    pkg_share = FindPackageShare('lidar_camera_slam')
    world_file = PathJoinSubstitution([pkg_share, 'worlds', 'test_room.world'])
    
    # Declare use_sim_time parameter
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Launch Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world_file,
            'verbose': 'true',
        }.items()
    )

    # Launch Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', PathJoinSubstitution([pkg_share, 'urdf', 'robot.urdf'])]),
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # Spawn Robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )

    # Launch SLAM Toolbox with optimized parameters for accurate mapping
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[{
            'use_sim_time': use_sim_time,
            'max_laser_range': 20.0,
            'resolution': 0.05,
            'map_update_interval': 0.5,
            'transform_timeout': 0.2,
            'map_frame': 'map',
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'scan_topic': 'scan',
            'map_file_name': '',
            'map_start_pose': [0.0, 0.0, 0.0],
            'map_start_at_dock': True,
            'minimum_time_interval': 0.2,
            'transform_publish_period': 0.02,
            'use_scan_matching': True,
            'use_scan_barycenter': True,
            'resolution_closure_threshold': 0.05,
            'minimum_distance_threshold': 0.1,
            'publish_period': 0.5
        }],
        output='screen'
    )

    # Map Server for saving
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': 'maps/room_map.yaml',
            'topic_name': 'map',
            'frame_id': 'map'
        }]
    )

    # Create an event handler to spawn the robot after robot_state_publisher starts
    spawn_event = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_state_publisher,
            on_start=[spawn_robot]
        )
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher,
        spawn_event,
        slam_toolbox,
        map_server
    ]) 