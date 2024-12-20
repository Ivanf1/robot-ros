#!/usr/bin/python3
import sys, signal
import rospy
from arm_manager import ArmManager
from wheels_manager import WheelsManager
from serial_manager import SerialManager

serial_port = '/dev/ttyAMA0'
baud_rate = 9600

def signal_handler(signal, frame):
    print("\nexiting")
    sys.exit(0)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)

    rospy.init_node("my_robot_listener", anonymous=False)
    rospy.loginfo("Node started")

    serial_mng = SerialManager(serial_port, baud_rate)

    wheels_mng = WheelsManager(serial_mng)
    wheels_mng.listen()

    arm_mng = ArmManager(serial_mng)
    arm_mng.listen()

    while not rospy.is_shutdown():
        serial_mng.process_messages()
