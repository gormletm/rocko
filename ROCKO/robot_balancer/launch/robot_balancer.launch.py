import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get the package directory
    pkg_share = FindPackageShare('robot_balancer').find('robot_balancer')
    
    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': PathJoinSubstitution([pkg_share, 'urdf', 'robot.urdf']),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # Controller manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            PathJoinSubstitution([pkg_share, 'config', 'controllers.yaml']),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )
    
    # Hardware interface
    hardware_interface = Node(
        package='robot_balancer',
        executable='robot_hardware_interface',
        name='robot_hardware_interface',
        output='screen'
    )
    
    # IMU node
    imu_node = Node(
        package='robot_balancer',
        executable='imu_node',
        name='imu_node',
        output='screen'
    )
    
    # Balance controller node
    balance_controller = Node(
        package='robot_balancer',
        executable='balance_controller_node',
        name='balance_controller',
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time,
        robot_state_publisher,
        controller_manager,
        hardware_interface,
        imu_node,
        balance_controller
    ]) 