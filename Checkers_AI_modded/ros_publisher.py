"""
class will publish a board state to a ROS topic named /board_state. 
It employs the ROS Python package, rospy, for this purpose. 
The board state is represented as an Int32MultiArray
"""

import rospy
from std_msgs.msg import Int32MultiArray

class ros_publisher:
	"""docstring for ros_publisher"""
	def __init__(self, arg=None):
		super(ros_publisher, self).__init__()
		self.arg = arg
		# ROS specific initialisation
		rospy.init_node('board_state_publisher', anonymous=True)
		self.pub = rospy.Publisher('/board_state', Int32MultiArray, queue_size=10, latch=True)
		self.rate = rospy.Rate(10)  # 10 Hz
		self.array_to_publish = Int32MultiArray()
		self.last_published_array = None

	def rosPrint(self, x_array, y_array, colour_array):
		# Flatten the array
		flat_array = []
		for i in range(len(x_array)):
			flat_array.append(x_array[i])
			flat_array.append(y_array[i])
			flat_array.append(colour_array[i])

		# Check if the array is different from the last published one
		if flat_array != self.last_published_array:
			if not rospy.is_shutdown():
				self.array_to_publish.data = flat_array
				self.pub.publish(self.array_to_publish)
				self.rate.sleep()

			# Update the last published array
			self.last_published_array = flat_array

	