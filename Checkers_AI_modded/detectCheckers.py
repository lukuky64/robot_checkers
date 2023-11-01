import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import copy
from threading import Event

class detectCheckers:
    # ------------------------------------------------------------------
    # This class identifies the coordinates of blue checkers on an 8x8
    # checkerboard in an image feed. Assumes the board is perpendicular
    # to the camera's principal axis. Top left is origin point (0,0).
    # ------------------------------------------------------------------

    def __init__(self):
        self.bridge = CvBridge()
        self.locations = []
        self.process_image = False
        self.callbackCount = 0
        self.image_sub = None
        self.previous_capture = None
        self.firstMove = True

        self.movedFrom = None
        self.movedTo = None
        self.process_done = Event()

        # Check if topic is available
        try:
            topic_list = rospy.get_published_topics()
            if [t for t, _ in topic_list if t == '/usb_cam/image_raw']:
                self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)
            else:
                print("Topic '/usb_cam/image_raw' is not available. Exiting.")
                rospy.signal_shutdown('Topic not found')
        
        except rospy.ROSException as e:
            print(f"An error occurred: {e}")
            rospy.signal_shutdown('An error occurred')

    
    def detect_change(self, current_capture):
        # initialise local variables
        from_position = None
        to_position = None
        
        # store the previous capture and identify what checkers have changed
        if not self.firstMove:
            # Identify the moved checker
            for position in current_capture:
                if position not in self.previous_capture:
                    to_position = position
                    
            for position in self.previous_capture:
                if position not in current_capture:
                    from_position = position
            
            print("Fr: ", from_position, "\nTo: ", to_position)
            
            self.movedFrom = from_position
            self.movedTo = to_position
        
        # setting previous capture to current for next iteration
        self.previous_capture = copy.deepcopy(current_capture)

    

    def callback(self, data):
        if self.process_image:

            # Attempt to identify gameboard 3 times before next user input
            if self.callbackCount >= 3:
                self.process_image = False
                self.callbackCount = 0

            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
            except CvBridgeError as e:
                print(e)
                return

            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            # cv2.imshow("gray", gray_image)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()

            # return if checkerboard is found + location of inner corners of board
            ret, corners = cv2.findChessboardCorners(gray_image, (7, 7), None)

            if ret:
                print("Checkerboard found!")
                top_left = corners[0][0]
                bottom_right = corners[-1][0]

                # Inner corner positions describe 6 grid cells
                grid_width = (bottom_right[0] - top_left[0]) / 6.0 
                grid_height = (bottom_right[1] - top_left[1]) / 6.0

                # ensure that top_left and button_right are correct
                if (grid_width > 0) and (grid_height > 0):
                    # image processing - blurring + dilation + watershed
                    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)


                    # Split the HSV image into individual channels
                    h, s, v = cv2.split(hsv)

                    # Display the individual channels
                    # cv2.imshow("Hue", h)
                    # cv2.imshow("Saturation", s)
                    # cv2.imshow("Value", v)

                    # Wait for a key press and close the image windows
                    # cv2.waitKey(0)
                    # cv2.destroyAllWindows()


                    lower_bound = np.array([0, 100, 100])
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

                    # convert markers to gameboard cooirdnates
                    for marker in unique_markers:
                        if marker <= 1:
                            continue

                        coords = np.argwhere(markers == marker)
                        y, x = coords.mean(axis=0)

                        dist_x = (x - (top_left[0] - grid_width) - (grid_width / 2))
                        dist_y = (y - (top_left[1] - grid_height) - (grid_height / 2))

                        x_grid = int(round(dist_x / grid_width))
                        y_grid = int(round(dist_y / grid_height))

                        # if the values are within the bounds of the checkerboard, save them
                        if 0 <= x_grid <= 7 and 0 <= y_grid <= 7:
                            self.locations.append((x_grid, y_grid))
                            cv2.circle(cv_image, (int(round(x)), int(round(y))), radius=10, color=(0, 255, 0), thickness=-1)

                    # visualise
                    cv2.imshow('Checkers Located', cv_image)
                    cv2.waitKey(1000)  # Wait for 3000 milliseconds (3 seconds)
                    cv2.destroyAllWindows()  # Close all OpenCV windows
                    
                    # convert a unique list, removing duplicates
                    self.locations = list(set(self.locations))
                    # print("Locations of blue checkers: ", self.locations)

                    self.detect_change(self.locations)

                    self.locations.clear()

                    
                    self.process_image = False
                    self.callbackCount = 0
                    self.firstMove = False
            
            self.callbackCount += 1
            self.process_done.set()  # Signal that the processing is done


    def capture(self):
        while True:
            input("Press Enter to capture: ")
            self.process_image = True

            self.process_done.wait()  # Wait until the event is set in callback
            self.process_done.clear()  # Reset the event for the next round

            localMovedFrom = self.movedFrom
            localMovedTo = self.movedTo

            if self.movedFrom is not None:
                self.movedFrom = None
                self.movedTo = None
                return [localMovedFrom, localMovedTo]


# if __name__ == '__main__':
#     rospy.init_node('checkerboard_detector', anonymous=True)
#     detector = detectCheckers()

#     if detector.image_sub is None:  # Add this check
#         exit(1)

#     try:
#         exit_flag = False
#         while True:  # Loop to wait for Enter key
#             while(not detector.process_image):
#                 user_input = input("Press Enter to capture or type 'exit' to quit: ")

#                 if user_input.lower() == 'exit':  # Exit the program
#                     print("Exiting...")
#                     exit_flag = True
#                     break

#                 detector.process_image = True  # Set the flag to True when Enter is pressed
            
#             if exit_flag:
#                 break

#     except KeyboardInterrupt:
#         print("Ctrl+C detected. Shutting down.")

#     rospy.signal_shutdown('Exit requested')  # Shutdown the ROS node