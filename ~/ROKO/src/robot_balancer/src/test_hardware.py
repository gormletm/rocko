#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time
import numpy as np

class HardwareTester(Node):
    def __init__(self):
        super().__init__('hardware_tester')
        
        # Motor control pins (BCM numbering)
        self.LEFT_DRIVE_PWM = 11   # GPIO 11
        self.LEFT_DRIVE_DIR = 8    # GPIO 8
        self.RIGHT_DRIVE_PWM = 9   # GPIO 9
        self.RIGHT_DRIVE_DIR = 25  # GPIO 25
        
        # Encoder pins
        self.LEFT_ENCODER_A = 26   # GPIO 26
        self.LEFT_ENCODER_B = 20   # GPIO 20
        self.RIGHT_ENCODER_A = 19  # GPIO 19
        self.RIGHT_ENCODER_B = 16  # GPIO 16
        
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
        
        # Encoder state
        self.left_encoder_count = 0
        self.right_encoder_count = 0
        self.left_encoder_last_state = None
        self.right_encoder_last_state = None
        
        # Setup encoder interrupts
        GPIO.add_event_detect(self.LEFT_ENCODER_A, GPIO.BOTH, callback=self.left_encoder_callback)
        GPIO.add_event_detect(self.RIGHT_ENCODER_A, GPIO.BOTH, callback=self.right_encoder_callback)
        
        self.get_logger().info("Hardware tester initialized")

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

    def test_motors(self):
        self.get_logger().info("Testing motors...")
        
        # Test left motor forward
        self.get_logger().info("Left motor forward at 50%")
        GPIO.output(self.LEFT_DRIVE_DIR, True)
        self.left_drive_pwm.ChangeDutyCycle(50)
        time.sleep(2)
        self.left_drive_pwm.ChangeDutyCycle(0)
        
        # Test left motor reverse
        self.get_logger().info("Left motor reverse at 50%")
        GPIO.output(self.LEFT_DRIVE_DIR, False)
        self.left_drive_pwm.ChangeDutyCycle(50)
        time.sleep(2)
        self.left_drive_pwm.ChangeDutyCycle(0)
        
        # Test right motor forward
        self.get_logger().info("Right motor forward at 50%")
        GPIO.output(self.RIGHT_DRIVE_DIR, True)
        self.right_drive_pwm.ChangeDutyCycle(50)
        time.sleep(2)
        self.right_drive_pwm.ChangeDutyCycle(0)
        
        # Test right motor reverse
        self.get_logger().info("Right motor reverse at 50%")
        GPIO.output(self.RIGHT_DRIVE_DIR, False)
        self.right_drive_pwm.ChangeDutyCycle(50)
        time.sleep(2)
        self.right_drive_pwm.ChangeDutyCycle(0)
        
        self.get_logger().info("Motor test complete")

    def test_encoders(self):
        self.get_logger().info("Testing encoders...")
        self.get_logger().info("Move the wheels manually and check the counts")
        
        try:
            while True:
                self.get_logger().info(f"Left encoder: {self.left_encoder_count}, Right encoder: {self.right_encoder_count}")
                time.sleep(0.5)
        except KeyboardInterrupt:
            self.get_logger().info("Encoder test complete")

    def cleanup(self):
        self.left_drive_pwm.stop()
        self.right_drive_pwm.stop()
        GPIO.cleanup()
        self.get_logger().info("Hardware cleanup complete")

def main(args=None):
    rclpy.init(args=args)
    tester = HardwareTester()
    
    try:
        # Test motors
        tester.test_motors()
        
        # Test encoders
        tester.test_encoders()
        
    except KeyboardInterrupt:
        pass
    finally:
        tester.cleanup()
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 