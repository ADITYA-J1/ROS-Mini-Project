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
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'robot.rviz'])
    
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

    # Spawn Robot - now using event handler to ensure robot_state_publisher is running first
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

    # Point Cloud to Laser Scan
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.01,
            'min_height': -1.0,
            'max_height': 2.0,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'range_min': 0.1,
            'range_max': 20.0,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # Launch SLAM Toolbox
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[{
            'use_sim_time': use_sim_time,
            'max_laser_range': 20.0,
            'resolution': 0.05,
            'map_update_interval': 1.0,
            'transform_timeout': 0.2,
            'map_frame': 'map',
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'scan_topic': 'scan'
        }],
        output='screen'
    )

    # Launch RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
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
        pointcloud_to_laserscan,
        slam_toolbox,
        rviz
    ]) 