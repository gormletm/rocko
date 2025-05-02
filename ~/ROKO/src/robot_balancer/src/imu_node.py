#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import board
import busio
import adafruit_bno055

class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        
        # Initialize I2C bus and BNO055
        i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_bno055.BNO055_I2C(i2c)
        
        # Create publisher for IMU data
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        
        # Create timer for reading IMU data
        timer_period = 0.01  # 100Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('IMU node started')

    def timer_callback(self):
        msg = Imu()
        
        # Get orientation data
        orientation = self.sensor.euler
        if orientation is not None:
            msg.orientation.x = orientation[0]
            msg.orientation.y = orientation[1]
            msg.orientation.z = orientation[2]
        
        # Get angular velocity
        gyro = self.sensor.gyro
        if gyro is not None:
            msg.angular_velocity.x = gyro[0]
            msg.angular_velocity.y = gyro[1]
            msg.angular_velocity.z = gyro[2]
        
        # Get linear acceleration
        accel = self.sensor.linear_acceleration
        if accel is not None:
            msg.linear_acceleration.x = accel[0]
            msg.linear_acceleration.y = accel[1]
            msg.linear_acceleration.z = accel[2]
        
        # Set header timestamp
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    imu_node = IMUNode()
    rclpy.spin(imu_node)
    imu_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 