#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from hardware_interface import SystemInterface, ActuatorInterface, SensorInterface
from controller_interface import ControllerInterface
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import numpy as np

class RobotHardwareInterface(SystemInterface):
    def __init__(self):
        super().__init__()
        
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
        
        # Joint states
        self.joint_positions = [0.0, 0.0]  # [left_wheel, right_wheel]
        self.joint_velocities = [0.0, 0.0]
        self.joint_efforts = [0.0, 0.0]
        
        # Joint commands
        self.joint_commands = [0.0, 0.0]
        
        # Setup encoder interrupts
        GPIO.add_event_detect(self.LEFT_ENCODER_A, GPIO.BOTH, callback=self.left_encoder_callback)
        GPIO.add_event_detect(self.RIGHT_ENCODER_A, GPIO.BOTH, callback=self.right_encoder_callback)

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

    def read(self):
        # Update joint states from encoders
        # Convert encoder counts to radians (assuming 20 counts per revolution)
        self.joint_positions[0] = (self.left_encoder_count / 20.0) * 2.0 * np.pi
        self.joint_positions[1] = (self.right_encoder_count / 20.0) * 2.0 * np.pi
        
        # TODO: Calculate velocities from position changes
        # For now, we'll just set them to 0
        self.joint_velocities = [0.0, 0.0]
        
        return self.joint_positions, self.joint_velocities, self.joint_efforts

    def write(self):
        # Convert joint commands to PWM duty cycles
        left_duty_cycle = abs(self.joint_commands[0]) * 100
        right_duty_cycle = abs(self.joint_commands[1]) * 100
        
        # Limit duty cycles
        left_duty_cycle = np.clip(left_duty_cycle, 0, 100)
        right_duty_cycle = np.clip(right_duty_cycle, 0, 100)
        
        # Set directions
        GPIO.output(self.LEFT_DRIVE_DIR, self.joint_commands[0] >= 0)
        GPIO.output(self.RIGHT_DRIVE_DIR, self.joint_commands[1] >= 0)
        
        # Set PWM
        self.left_drive_pwm.ChangeDutyCycle(left_duty_cycle)
        self.right_drive_pwm.ChangeDutyCycle(right_duty_cycle)

    def __del__(self):
        self.left_drive_pwm.stop()
        self.right_drive_pwm.stop()
        GPIO.cleanup() 