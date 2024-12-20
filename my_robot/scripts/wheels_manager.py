#!/usr/bin/python3
import rospy
from my_robot.msg import xyz
import binascii

import struct

MOTOR_MAX = 1023
MAX_LINEAR_VEL = 1.0
MAX_ANGULAR_VEL = 1.0

L = 1
W = 1

class WheelsManager:
    def __init__(self, serial_mng):
        self.serial_mng = serial_mng

        self.motors_speeds = [
            0, # FR
            0, # FL
            0, # RR
            0  # RL
        ]

        self.START_BYTE = b'\xFF'
        self.MOVEMENT_MESSAGE_FLAG = b'\xE0'

    def calculate_motor_command(self, speed):
        if speed >= 0:
            return abs(int(min(speed * 1023, MOTOR_MAX) + 1024))  # Counterclockwise
        else:
            return abs(int(min(speed * 1023, MOTOR_MAX)))  # Clockwise

    def send_uint16_values(self, values):
        # Pack the four uint16 values into 8 bytes in little-endian format
        data = self.START_BYTE + self.MOVEMENT_MESSAGE_FLAG  + struct.pack('4H', *values)
        print(binascii.hexlify(data))
        self.serial_mng.put_message(data)

    def on_message(self, data):
        vx = data.x * MAX_LINEAR_VEL      # Forward/backward
        vy = data.y * MAX_LINEAR_VEL      # Left/right
        omega = data.z * MAX_ANGULAR_VEL  # Rotation

        v_FL = -vx - vy - ((L + W) * omega / 2)
        v_FR = vx + vy + ((L + W) * omega / 2)
        v_RL = vx + vy - ((L + W) * omega / 2)
        v_RR = -vx - vy + ((L + W) * omega / 2)

        # Map wheel speeds to motor commands
        motor_FL = self.calculate_motor_command(v_FL)
        motor_FR = self.calculate_motor_command(v_FR)
        motor_RL = self.calculate_motor_command(v_RL)
        motor_RR = self.calculate_motor_command(v_RR)

        self.motors_speeds[0] =  motor_FL
        self.motors_speeds[1] =  motor_FR
        self.motors_speeds[2] =  motor_RL
        self.motors_speeds[3] =  motor_RR
        rospy.loginfo("motor speeds: FL: " +str(motor_FL)+ ", FR: " +str(motor_FR)+ ", RL: " +str(motor_RL)+", RR: "+str(motor_RR))
        self.send_uint16_values(self.motors_speeds)

    def listen(self):
        rospy.Subscriber("my_robot/movement/xyz", xyz, self.on_message)