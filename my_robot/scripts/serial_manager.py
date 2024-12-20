import rospy
import serial
import queue

class SerialManager:
    def __init__(self, port, baud_rate):
        self.ser = serial.Serial(port, baud_rate)
        self.message_queue = queue.Queue()

    def put_message(self, message):
        self.message_queue.put(message)

    def process_messages(self):
        while True:
            # block until a message is in the queue
            msg = self.message_queue.get(block=True, timeout=None)
            rospy.loginfo(msg)
            self.ser.write(msg)

            # wait for a response from arduino
            while not self.ser.in_waiting:
                line = self.ser.readline()
                rospy.loginfo("arduino: " + str(line) + "\n")
                break