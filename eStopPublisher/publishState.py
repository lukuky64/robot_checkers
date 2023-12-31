# This class collects information from the Arduino via USB serial communication and publishes it to ROS topics

#!/usr/bin/env python
import serial
import rospy
from time import sleep
import os
from std_msgs.msg import UInt8, Int32


#--- codes for target_safety_status on dobot magician ---
# 0 = INVALID
# 1 = DISCONNECTED
# 2 = INITIALISING
# 3 = ESTOPPED
# 4 = OPERATING
# 5 = PAUSED
# 6 = STOPPED


def main():
    port = "/dev/ttyUSB1"  # "/dev/ttyUSB0" "/dev/ttyACM0"

    # Wait for Arduino to be connected
    while not os.path.exists(port):
        print("Waiting for Arduino to be connected...")
        sleep(1)  # Wait for 1 second before checking again

    # Initialise ROS node
    rospy.init_node("arduino_serial_to_ros", anonymous=True)

    # Create ROS publisher for game class
    pub = rospy.Publisher("eStop_state", UInt8, queue_size=10, latch=True)

    # Create ROS publisher for real DoBot Magician
    pub_DoBot = rospy.Publisher(
        "/dobot_magician/target_safety_status", UInt8, queue_size=10, latch=True
    )

    # Initialise serial port with baudrate 9600
    ser = serial.Serial(port, 9600, timeout=0.1)

    msg = 0  # Initialize msg
    doBot_msg = 0  # Initialize msg2

    # Loop to keep reading from Arduino and publishing to ROS
    rate = rospy.Rate(5)  # Publish at 5 Hz
    while not rospy.is_shutdown():
        if ser.in_waiting > 0:
            arduino_data = ser.readline().decode("utf-8", errors="ignore").strip()
            try:
                arduino_data_int = int(arduino_data)
                if arduino_data_int == 3:
                    print("STOPPED")
                    msg = 1
                    doBot_msg = 3
                elif arduino_data_int == 2:
                    print("STOPPED")
                    doBot_msg = 3
                elif arduino_data_int == 1:
                    print("OPERATING")
                    msg = 0
                    doBot_msg = 4

                pub.publish(msg)
                pub_DoBot.publish(doBot_msg)
            except ValueError:
                print(f"Received invalid data: {arduino_data}")

        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
