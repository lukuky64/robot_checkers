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
        self.process_image = False  # Flag to control when to process the image
        self.callbackCount = 0

    def callback(self, data):
        
        if self.process_image:
            self.callbackCount += 1
            if self.callbackCount >= 5:
                self.process_image = False  # Reset the flag

            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
                cv2.waitKey(0)
            except CvBridgeError as e:
                print(e)
                return

            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray_image, (7, 7), None)
            
            if ret:
                print("Checkerboard found!")

                # Find the extent of the checkerboard
                top_left = corners[0][0]
                bottom_right = corners[-1][0]
                
                # Calculate grid dimensions
                grid_width = (bottom_right[0] - top_left[0]) / 7.0
                grid_height = (bottom_right[1] - top_left[1]) / 7.0

                hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

                # Split the HSV image into individual channels
                h, s, v = cv2.split(hsv)

                # Display the individual channels
                # cv2.imshow('Hue Channel', h)
                # cv2.imshow('Saturation Channel', s)
                # cv2.imshow('Value Channel', v)
                # cv2.waitKey(0)


                # Define range of blue-purple color in HSV
                # lower_blue = np.array([120, 50, 50])
                # upper_blue = np.array([150, 255, 255])

                # Define the lower and upper bounds for each channel
                lower_bound = np.array([0, 150, 100])
                upper_bound = np.array([50, 255, 255])

                # Threshold the HSV image to get blue-purple colors
                mask_combined = cv2.inRange(hsv, lower_bound, upper_bound)
                # cv2.imshow('MASKED', mask_combined)
                # cv2.waitKey(0)

                # Apply watershed algorithm on mask
                kernel = np.ones((4,4), np.uint8)
                opening = cv2.morphologyEx(mask_combined, cv2.MORPH_OPEN, kernel)
                sure_bg = cv2.dilate(opening, None, iterations=3)
                ret, sure_fg = cv2.threshold(opening, 0.7 * opening.max(), 255, 0)
                unknown = cv2.subtract(sure_bg, sure_fg)
                markers = cv2.connectedComponents(sure_fg)[1]
                markers += 1
                markers[unknown == 255] = 0
                
                cv2.watershed(cv_image, markers)

                # Marking the boundaries with red color for visualising
                cv_image[markers == -1] = [0, 0, 255]  
                # cv2.imshow('Watershed Segmentation', cv_image)
                # cv2.waitKey(0)


                unique_markers = np.unique(markers)
                
                for marker in unique_markers:
                    if marker <= 1:  # Skip background and boundary markers
                        continue
                    
                    coords = np.argwhere(markers == marker)
                    y, x = coords.mean(axis=0)

                    x_grid = int(round((x - top_left[0] - (grid_width / 2)) / grid_width))
                    y_grid = int(round((y - top_left[1] - (grid_height/ 2)) / grid_height))

                    self.locations.append((x_grid+1, y_grid))

                    
                    #print("top left: " + str(top_left))

                    # Calculate pixel coordinates for drawing
                    draw_x = int(top_left[0] + (x_grid * grid_width) + (grid_width / 2))
                    draw_y = int(top_left[1] + (y_grid * grid_height) + (grid_height / 2))
                    
                    # Draw a circle at the calculated location
                    cv2.circle(cv_image, (draw_x, draw_y), radius=14, color=(0, 255, 0), thickness=-1)
                
                # Show the modified image
                # cv2.imshow('Checkers Located', cv_image)
                # cv2.waitKey(0)


                self.locations = list(set(self.locations))
                print("Locations of blue checkers: ", self.locations)

                self.locations.clear()

                self.process_image = False  # Reset the flag


if __name__ == '__main__':
    rospy.init_node('checkerboard_detector', anonymous=True)
    detector = CheckerboardDetector()

    try:
        exit_flag = False
        while True:  # Loop to wait for Enter key
            wh(not detector.process_image):
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

