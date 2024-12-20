#!/usr/bin/python3
import rospy
from my_robot.msg import xy
from geometry_msgs.msg import Twist

import struct

class ArmManager:
    def __init__(self, serial_mng):
        self.serial_mng = serial_mng
        
        self.motors_positions = [
            512, # Center
            512, # Right
            512, # Left
        ]

        self.START_BYTE = b'\xFF'
        self.ARM_MESSAGE_FLAG = b'\xE1'
        self.ARM_MOVE_MESSAGE_FLAG = b'\xA0'
        self.ARM_KEEP_POSE_MESSAGE_FLAG = b'\xA1'
        self.PADDING_BYTE = b'\xEE'

        self.MOTOR_MOVEMENT_STEP = 10

        self.CENTER_MOTOR_UP_LIMIT = 210
        self.CENTER_MOTOR_DOWN_LIMIT = 540

        self.ARM_OPEN_LIMIT = 232
        self.ARM_CLOSE_LIMIT = 662

        self.ARM_DIFFERENCE_OPEN_MAX = 560
        self.ARM_DIFFERENCE_CLOSE_MAX = 300

    def send_uint16_values(self, values, arm_message_type):
        # Pack the three uint16 values into 8 bytes in little-endian format
        data = self.START_BYTE + self.ARM_MESSAGE_FLAG + arm_message_type + struct.pack('3H', *values) + self.PADDING_BYTE
        self.serial_mng.put_message(data)

    def send_arm_move_message(self, data):
        step_x = int(abs(data.x) * self.MOTOR_MOVEMENT_STEP)
        step_y = int(abs(data.y) * self.MOTOR_MOVEMENT_STEP)
        
        # move arm up
        if data.y > 0:
            # chech if motor has not already reached limit up position
            if self.motors_positions[0] != self.CENTER_MOTOR_UP_LIMIT:
                # check if moving the motor by 'motor_movement_step' units upwards
                # would not exceed the motor up limit
                if self.CENTER_MOTOR_UP_LIMIT <= self.motors_positions[0] - step_y:
                    self.motors_positions[0] -= step_y
                else:
                    self.motors_positions[0] = self.CENTER_MOTOR_UP_LIMIT

        # move arm down
        if data.y < 0:
            # chech if motor has not already reached limit down position
            if self.motors_positions[0] != self.CENTER_MOTOR_DOWN_LIMIT:
                # check if moving the motor by 'motor_movement_step' units
                # downwards would not exceed the motor down limit
                if self.CENTER_MOTOR_DOWN_LIMIT >= self.motors_positions[0] + step_y:
                    self.motors_positions[0] += step_y
                else:
                    self.motors_positions[0] = self.CENTER_MOTOR_DOWN_LIMIT

        # open the arm
        if data.x > 0:
            # chech if arm has not already reached limit open position
            if self.motors_positions[1] != self.ARM_OPEN_LIMIT:
                # check if moving the motor by 'motor_movement_step' units
                # would not exceed the motor limit
                if self.ARM_OPEN_LIMIT <= self.motors_positions[1] - step_x:
                    self.motors_positions[1] -= step_x
                    self.motors_positions[2] += step_x
                else:
                    self.motors_positions[1] = self.ARM_OPEN_LIMIT
                    self.motors_positions[2] = self.ARM_OPEN_LIMIT + self.ARM_DIFFERENCE_OPEN_MAX

        # close the arm
        if data.x < 0:
            # chech if arm has not already reached limit open position
            if self.motors_positions[1] != self.ARM_CLOSE_LIMIT:
                # check if moving the motor by 'motor_movement_step' units
                # would not exceed the motor limit
                if self.ARM_CLOSE_LIMIT >= self.motors_positions[1] + step_x:
                    self.motors_positions[1] += step_x
                    self.motors_positions[2] -= step_x
                else:
                    self.motors_positions[1] = self.ARM_CLOSE_LIMIT
                    self.motors_positions[2] = self.ARM_CLOSE_LIMIT - self.ARM_DIFFERENCE_CLOSE_MAX

        rospy.loginfo("motor positions: C {C}, R {R}, L {L}".format(C=self.motors_positions[0], R=self.motors_positions[1], L=self.motors_positions[2]))

        self.send_uint16_values(self.motors_positions, self.ARM_MOVE_MESSAGE_FLAG)

    def send_arm_keep_position_message(self):
        self.send_uint16_values(self.motors_positions, self.ARM_KEEP_POSE_MESSAGE_FLAG)

    def on_message(self, data):
        rospy.loginfo("x: " + str(data.x) + ", y: " + str(data.y))

        if abs(data.x) == 0 and abs(data.y) == 0:
            self.send_arm_keep_position_message()
        else:
            self.send_arm_move_message(data)

    def listen(self):
        rospy.Subscriber("my_robot/arm/xy", xy, self.on_message)
