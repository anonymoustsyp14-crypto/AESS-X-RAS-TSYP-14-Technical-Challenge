from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('tsyp_simulation')
    world = PathJoinSubstitution([pkg, 'worlds', 'rescue_building.world'])
    writer_robot = PathJoinSubstitution([pkg, 'models', 'waffle_writer.sdf'])
    executor_robot = PathJoinSubstitution([pkg, 'models', 'waffle_executor.sdf'])
    gz = PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
    gazebo_models = PathJoinSubstitution([FindPackageShare('turtlebot3_gazebo'), 'models'])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz),
        launch_arguments={'gz_args': ['-r ', world]}.items(),
    )

    spawn_robots = TimerAction(period=5.0, actions=[
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-file', writer_robot, '-name', 'writer_robot', '-x', '5.8', '-y', '0.0', '-z', '0.12']),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-file', executor_robot, '-name', 'executor_robot', '-x', '8.2', '-y', '0.0', '-z', '0.12']),
    ])

    bridges = TimerAction(period=6.0, actions=[
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             arguments=[
                 '/model/writer_robot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                 '/model/executor_robot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                 '/model/writer_robot/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                 '/model/executor_robot/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                 '/model/writer_robot/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                 '/model/executor_robot/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                 '/model/writer_robot/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
                 '/model/executor_robot/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
             ])
    ])

    scenario_nodes = TimerAction(period=7.0, actions=[
        Node(package='tsyp_simulation', executable='network_emulator_node.py', output='screen'),
        Node(package='tsyp_simulation', executable='beacon_manager_node.py', output='screen'),
        Node(package='tsyp_simulation', executable='gateway_node.py', output='screen'),
        Node(package='tsyp_simulation', executable='writer_robot_node.py', output='screen'),
        Node(package='tsyp_simulation', executable='executor_robot_node.py', output='screen'),
        Node(package='tsyp_simulation', executable='command_post_dashboard_node.py', output='screen'),
        Node(package='tsyp_simulation', executable='scenario_orchestrator_node.py', output='screen'),
    ])

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gazebo_models),
        gazebo, spawn_robots, bridges, scenario_nodes
    ])
