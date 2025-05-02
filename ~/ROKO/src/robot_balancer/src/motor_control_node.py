#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import time

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        
        # Motor control pins (BCM numbering)
        # Drive motors
        self.LEFT_DRIVE_PWM = 11   # GPIO 11
        self.LEFT_DRIVE_DIR = 8    # GPIO 8
        self.RIGHT_DRIVE_PWM = 9   # GPIO 9
        self.RIGHT_DRIVE_DIR = 25  # GPIO 25
        
        # Shoulder motors (to be configured)
        self.LEFT_SHOULDER_PWM = None
        self.LEFT_SHOULDER_DIR = None
        self.RIGHT_SHOULDER_PWM = None
        self.RIGHT_SHOULDER_DIR = None
        
        # Encoder pins
        self.LEFT_ENCODER_A = 26   # GPIO 26
        self.LEFT_ENCODER_B = 20   # GPIO 20
        self.RIGHT_ENCODER_A = 19  # GPIO 19
        self.RIGHT_ENCODER_B = 16  # GPIO 16
        
        # Encoder state
        self.left_encoder_count = 0
        self.right_encoder_count = 0
        self.left_encoder_last_state = None
        self.right_encoder_last_state = None
        
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        
        # Setup drive motor pins
        GPIO.setup(self.LEFT_DRIVE_PWM, GPIO.OUT)
        GPIO.setup(self.LEFT_DRIVE_DIR, GPIO.OUT)
        GPIO.setup(self.RIGHT_DRIVE_PWM, GPIO.OUT)
        GPIO.setup(self.RIGHT_DRIVE_DIR, GPIO.OUT)
        
        # Setup encoder pins
        GPIO.setup(self.LEFT_ENCODER_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.LEFT_ENCODER_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.RIGHT_ENCODER_A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.RIGHT_ENCODER_B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        # Setup PWM for drive motors
        self.left_drive_pwm = GPIO.PWM(self.LEFT_DRIVE_PWM, 1000)  # 1kHz frequency
        self.right_drive_pwm = GPIO.PWM(self.RIGHT_DRIVE_PWM, 1000)
        self.left_drive_pwm.start(0)
        self.right_drive_pwm.start(0)
        
        # Setup encoder interrupts
        GPIO.add_event_detect(self.LEFT_ENCODER_A, GPIO.BOTH, callback=self.left_encoder_callback)
        GPIO.add_event_detect(self.RIGHT_ENCODER_A, GPIO.BOTH, callback=self.right_encoder_callback)
        
        # Create subscribers for motor commands
        self.left_drive_sub = self.create_subscription(
            Float32,
            'left_drive/command',
            self.left_drive_callback,
            10)
        self.right_drive_sub = self.create_subscription(
            Float32,
            'right_drive/command',
            self.right_drive_callback,
            10)
        
        # Create publishers for encoder counts
        self.left_encoder_pub = self.create_publisher(Float32, 'left_encoder/count', 10)
        self.right_encoder_pub = self.create_publisher(Float32, 'right_encoder/count', 10)
        
        # Create timer for publishing encoder counts
        self.encoder_timer = self.create_timer(0.01, self.publish_encoder_counts)
        
        self.get_logger().info('Motor control node started')

    def left_encoder_callback(self, channel):
        a_state = GPIO.input(self.LEFT_ENCODER_A)
        b_state = GPIO.input(self.LEFT_ENCODER_B)
        
        if self.left_encoder_last_state is not None:
            if a_state != self.left_encoder_last_state:
                if a_state == b_state:
                    self.left_encoder_count += 1
                else:
                    self.left_encoder_count -= 1
        
        self.left_encoder_last_state = a_state

    def right_encoder_callback(self, channel):
        a_state = GPIO.input(self.RIGHT_ENCODER_A)
        b_state = GPIO.input(self.RIGHT_ENCODER_B)
        
        if self.right_encoder_last_state is not None:
            if a_state != self.right_encoder_last_state:
                if a_state == b_state:
                    self.right_encoder_count += 1
                else:
                    self.right_encoder_count -= 1
        
        self.right_encoder_last_state = a_state

    def publish_encoder_counts(self):
        left_msg = Float32()
        left_msg.data = float(self.left_encoder_count)
        self.left_encoder_pub.publish(left_msg)
        
        right_msg = Float32()
        right_msg.data = float(self.right_encoder_count)
        self.right_encoder_pub.publish(right_msg)

    def left_drive_callback(self, msg):
        # Convert command to PWM duty cycle (0-100)
        duty_cycle = abs(msg.data) * 100
        if duty_cycle > 100:
            duty_cycle = 100
        
        # Set direction
        GPIO.output(self.LEFT_DRIVE_DIR, msg.data >= 0)
        
        # Set PWM
        self.left_drive_pwm.ChangeDutyCycle(duty_cycle)

    def right_drive_callback(self, msg):
        # Convert command to PWM duty cycle (0-100)
        duty_cycle = abs(msg.data) * 100
        if duty_cycle > 100:
            duty_cycle = 100
        
        # Set direction
        GPIO.output(self.RIGHT_DRIVE_DIR, msg.data >= 0)
        
        # Set PWM
        self.right_drive_pwm.ChangeDutyCycle(duty_cycle)

    def __del__(self):
        self.left_drive_pwm.stop()
        self.right_drive_pwm.stop()
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    motor_control_node = MotorControlNode()
    rclpy.spin(motor_control_node)
    motor_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 