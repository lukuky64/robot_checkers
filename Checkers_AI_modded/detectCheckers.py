# This class will detect the locations of the blue checkers (user is assumed to play as blue) on the board
# given an RGB imagefeed through a ROS node. It will then publish the locations of the blue checkers to a ROS topic.

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

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        # Detect 8x8 checkerboard using open CV function
        ret, corners = cv2.findChessboardCorners(cv_image, (8,8), None)
        
        if ret:
            # Isolate blue channel and convert to greyscale
            blue_channel = cv_image[:,:,0] # BGR, blue channel is index 0
            grey_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Apply watershed algorithm
            _, thresh = cv2.threshold(grey_image, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
            sure_bg = cv2.dilate(thresh, None, iterations=3)
            ret, sure_fg = cv2.threshold(thresh, 0.7 * thresh.max(), 255, 0)
            unknown = cv2.subtract(sure_bg, sure_fg)
            
            markers = cv2.connectedComponents(sure_fg)[1]
            markers += 1
            markers[unknown==255] = 0
            
            cv2.watershed(cv_image, markers)
            
            # Get centre of each blue checker
            unique_markers = np.unique(markers)
            for marker in unique_markers:
                if marker == 0:
                    continue
                
                coords = np.argwhere(markers == marker)
                y, x = coords.mean(axis=0)
                
                # Represent its location on the 8x8 grid
                x_grid = int(7 * (x / cv_image.shape[1]))
                y_grid = int(7 * (1 - (y / cv_image.shape[0])))
                
                self.locations.append((x_grid, y_grid))
            
            print("Locations of blue checkers: ", self.locations)
            self.locations.clear()

if __name__ == '__main__':
    rospy.init_node('checkerboard_detector', anonymous=True)
    detector = CheckerboardDetector()
    input("Press Enter to capture")
    rospy.spin()
