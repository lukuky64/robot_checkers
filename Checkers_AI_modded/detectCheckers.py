import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time

class CheckerboardDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)
        self.locations = []
        self.process_image = False
        self.callbackCount = 0

    def callback(self, data):
        if self.process_image:
            self.callbackCount += 1
            if self.callbackCount >= 5:
                self.process_image = False
                self.callbackCount = 0

            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
            except CvBridgeError as e:
                print(e)
                return

            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray_image, (7, 7), None)

            if ret:
                print("Checkerboard found!")
                top_left = corners[0][0]
                bottom_right = corners[-1][0]
                grid_width = (bottom_right[0] - top_left[0]) / 7.0
                grid_height = (bottom_right[1] - top_left[1]) / 7.0

                hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
                h, s, v = cv2.split(hsv)

                lower_bound = np.array([0, 150, 100])
                upper_bound = np.array([50, 255, 255])
                mask_combined = cv2.inRange(hsv, lower_bound, upper_bound)

                kernel = np.ones((4,4), np.uint8)
                opening = cv2.morphologyEx(mask_combined, cv2.MORPH_OPEN, kernel)
                sure_bg = cv2.dilate(opening, None, iterations=3)
                ret, sure_fg = cv2.threshold(opening, 0.7 * opening.max(), 255, 0)
                unknown = cv2.subtract(sure_bg, sure_fg)
                markers = cv2.connectedComponents(sure_fg)[1]
                markers += 1
                markers[unknown == 255] = 0

                cv2.watershed(cv_image, markers)
                cv_image[markers == -1] = [0, 0, 255]

                unique_markers = np.unique(markers)

                for marker in unique_markers:
                    if marker <= 1:
                        continue

                    coords = np.argwhere(markers == marker)
                    y, x = coords.mean(axis=0)

                    x_grid = int(round((x - (top_left[0] - grid_width) - (grid_width / 2)) / grid_width))
                    y_grid = int(round((y - (top_left[1] - grid_height) - (grid_height / 2)) / grid_height))

                    self.locations.append((x_grid, y_grid))

                    draw_x = int(round(x))
                    draw_y = int(round(y))

                    cv2.circle(cv_image, (draw_x, draw_y), radius=14, color=(0, 255, 0), thickness=-1)

                cv2.imshow('Checkers Located', cv_image) # This works
                cv2.waitKey(0)
                
                self.locations = list(set(self.locations))
                print("Locations of blue checkers: ", self.locations)
                self.locations.clear()

                self.process_image = False
                self.callbackCount = 0



if __name__ == '__main__':
    rospy.init_node('checkerboard_detector', anonymous=True)
    detector = CheckerboardDetector()

    try:
        exit_flag = False
        while True:  # Loop to wait for Enter key
            while(not detector.process_image):
                user_input = input("Press Enter to capture or type 'exit' to quit: ")

                if user_input.lower() == 'exit':  # Exit the program
                    print("Exiting...")
                    exit_flag = True
                    break

                detector.process_image = True  # Set the flag to True when Enter is pressed
            
            if exit_flag:
                break

    except KeyboardInterrupt:
        print("Ctrl+C detected. Shutting down.")

    rospy.signal_shutdown('Exit requested')  # Shutdown the ROS node