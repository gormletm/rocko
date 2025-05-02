from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_balancer',
            executable='imu_node.py',
            name='imu_node',
            output='screen'
        ),
        Node(
            package='robot_balancer',
            executable='motor_control_node.py',
            name='motor_control_node',
            output='screen'
        ),
        Node(
            package='robot_balancer',
            executable='balance_controller_node.py',
            name='balance_controller_node',
            output='screen'
        )
    ]) 