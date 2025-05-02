#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import numpy as np

class PIDController:
    def __init__(self, kp, ki, kd, dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral = 0.0
        self.previous_error = 0.0
        self.integral_limit = 100.0  # Limit integral windup

    def compute(self, error):
        # Proportional term
        p = self.kp * error
        
        # Integral term with anti-windup
        self.integral += error * self.dt
        self.integral = np.clip(self.integral, -self.integral_limit, self.integral_limit)
        i = self.ki * self.integral
        
        # Derivative term
        derivative = (error - self.previous_error) / self.dt
        d = self.kd * derivative
        
        # Update previous error
        self.previous_error = error
        
        # Compute output
        output = p + i + d
        
        return output

class BalanceControllerNode(Node):
    def __init__(self):
        super().__init__('balance_controller_node')
        
        # Create PID controllers for pitch and roll
        self.pitch_pid = PIDController(kp=1.0, ki=0.0, kd=0.1, dt=0.01)
        # self.roll_pid = PIDController(kp=1.0, ki=0.0, kd=0.1, dt=0.01)  # Commented out roll controller
        
        # Create subscribers for IMU data and encoder counts
        self.imu_sub = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            10)
        self.left_encoder_sub = self.create_subscription(
            Float32,
            'left_encoder/count',
            self.left_encoder_callback,
            10)
        self.right_encoder_sub = self.create_subscription(
            Float32,
            'right_encoder/count',
            self.right_encoder_callback,
            10)
        
        # Create publishers for motor commands
        self.left_drive_pub = self.create_publisher(Float32, 'left_drive/command', 10)
        self.right_drive_pub = self.create_publisher(Float32, 'right_drive/command', 10)
        
        # Target angles (in radians)
        self.target_pitch = 0.0  # Upright position
        # self.target_roll = 0.0   # Upright position  # Commented out roll target
        
        # Encoder counts
        self.left_encoder_count = 0
        self.right_encoder_count = 0
        
        # Balance parameters
        self.max_motor_speed = 0.8  # Maximum motor speed (0-1)
        self.balance_threshold = 0.1  # Angle threshold for balance control
        
        self.get_logger().info('Balance controller node started')

    def left_encoder_callback(self, msg):
        self.left_encoder_count = msg.data

    def right_encoder_callback(self, msg):
        self.right_encoder_count = msg.data

    def imu_callback(self, msg):
        # Extract orientation from IMU message
        # Note: This assumes the IMU is mounted with X forward, Y left, Z up
        pitch = msg.orientation.x  # Pitch angle
        # roll = msg.orientation.y   # Roll angle  # Commented out roll reading
        
        # Only apply balance control if we're within the threshold
        if abs(pitch) < self.balance_threshold:  # Removed roll check
            # Compute errors
            pitch_error = self.target_pitch - pitch
            # roll_error = self.target_roll - roll  # Commented out roll error
            
            # Compute PID outputs
            pitch_output = self.pitch_pid.compute(pitch_error)
            # roll_output = self.roll_pid.compute(roll_error)  # Commented out roll output
            
            # Combine outputs for motor control
            # This is a simple mixing strategy - you may need to adjust based on your robot's configuration
            left_command = pitch_output  # Removed roll contribution
            right_command = pitch_output  # Removed roll contribution
            
            # Limit motor commands
            left_command = np.clip(left_command, -self.max_motor_speed, self.max_motor_speed)
            right_command = np.clip(right_command, -self.max_motor_speed, self.max_motor_speed)
        else:
            # If we're outside the threshold, stop the motors
            left_command = 0.0
            right_command = 0.0
        
        # Create and publish motor commands
        left_msg = Float32()
        left_msg.data = left_command
        self.left_drive_pub.publish(left_msg)
        
        right_msg = Float32()
        right_msg.data = right_command
        self.right_drive_pub.publish(right_msg)

def main(args=None):
    rclpy.init(args=args)
    balance_controller_node = BalanceControllerNode()
    rclpy.spin(balance_controller_node)
    balance_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 