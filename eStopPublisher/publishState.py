#!/usr/bin/env python
import serial
import rospy
from time import sleep
import os
from std_msgs.msg import Int32

def main():
    port = "/dev/ttyUSB0"

    # Wait for Arduino to be connected
    while not os.path.exists(port):
        print("Waiting for Arduino to be connected...")
        sleep(1)  # Wait for 1 second before checking again

    # Initialise ROS node
    rospy.init_node("arduino_serial_to_ros", anonymous=True)

    # Create ROS publisher
    pub = rospy.Publisher("eStop_state", Int32, queue_size=10, latch=True)

    # Initialise serial port with baudrate 9600
    ser = serial.Serial(port, 9600)

    msg = 0  # Initialize msg

    # Loop to keep reading from Arduino and publishing to ROS
    rate = rospy.Rate(5)  # Publish at 5 Hz
    while not rospy.is_shutdown():
        if ser.in_waiting > 0:
            print("message rceived...")
            arduino_data = ser.readline().decode("utf-8").strip()
            if arduino_data == "stopped":
                print("STOPPED")
                msg = 1  # Stopped
            else:
                print("RUNNING")
                msg = 0  # Running

            # Publish the data to the ROS topic
            pub.publish(msg)

        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
