import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class CheckerboardDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)
        self.locations = []
        self.process_image = False  # Flag to control when to process the image

    def callback(self, data):
        if self.process_image:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print(e)
                return

            ret, corners = cv2.findChessboardCorners(cv_image, (7, 7), None)
            
            if ret:
                print("Checkerboard found!")
                
                # Isolate blue channel
                blue_channel = cv_image[:,:,0] 
                
                # Threshold to identify blue regions
                _, blue_thresh = cv2.threshold(blue_channel, 128, 255, cv2.THRESH_BINARY)
                
                # Apply watershed algorithm on blue_thresh
                kernel = np.ones((3,3), np.uint8)
                opening = cv2.morphologyEx(blue_thresh, cv2.MORPH_OPEN, kernel)
                sure_bg = cv2.dilate(opening, None, iterations=3)
                ret, sure_fg = cv2.threshold(opening, 0.7 * opening.max(), 255, 0)
                unknown = cv2.subtract(sure_bg, sure_fg)
                
                markers = cv2.connectedComponents(sure_fg)[1]
                markers += 1
                markers[unknown == 255] = 0
                
                cv2.watershed(cv_image, markers)

                # cv_image[markers == -1] = [0, 0, 255]  # Marking the boundaries with red color
                # cv2.imshow('Watershed Segmentation', cv_image)
                # cv2.waitKey(0)


                unique_markers = np.unique(markers)
                
                for marker in unique_markers:
                    if marker <= 1:  # Skip background and boundary markers
                        continue
                    
                    coords = np.argwhere(markers == marker)
                    y, x = coords.mean(axis=0)
                    
                    x_grid = int(7 * (x / cv_image.shape[1]))
                    y_grid = int(7 * (1 - (y / cv_image.shape[0])))
                    
                    self.locations.append((x_grid, y_grid))

                    # Highlight checker in the original image
                    for coord in coords:
                        cv_image[coord[0], coord[1]] = [0, 255, 0]  # Green color
                
                # Display the modified image
                cv2.imshow('Highlighted Checkers', cv_image)
                cv2.waitKey(0)
                
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
            while(not detector.process_image == True):
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

