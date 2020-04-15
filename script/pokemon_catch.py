#!/usr/bin/env python

import roslib
roslib.load_manifest("pokemon")
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError
import math
from geometry_msgs.msg import Twist
import threading
import numpy as np

is_stop = False
 
class image_converter:
 
	def __init__(self, rate):
		self.cv_image = None
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)
		self.scan_sub = rospy.Subscriber("/scan",LaserScan,self.scan_callback)
		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
		self.rate = rate
		self.turn = 0

	def scan_callback(self, data):
		ranges = data.ranges
		ranges_size = len(ranges)//3
		distance, count = 0, 0
		for dis in ranges[ranges_size:2*ranges_size]:
			if not math.isnan(dis):
				distance += dis
				count += 1
		distance = distance/count

		if distance > 1:
			move_cmd = Twist()
			# let's go forward at 0.2 m/s
			move_cmd.linear.x = 0.2
			# let's turn at 0 radians/s
			move_cmd.angular.z = self.turn
			for x in range(10):
				self.cmd_vel.publish(move_cmd)
				self.rate.sleep()
			rospy.loginfo("%.3f meter away from pokemon."%(distance))
		else:
			global is_stop
			cv2.imwrite("Pokemon.jpg", self.cv_image)
			rospy.loginfo("I catch a pokemon.")
			is_stop = True


	def callback(self,data):
		try:
			self.cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
		except CvBridgeError as e:
			print(e)

		(height,width,channels) = self.cv_image.shape
	
		thresh = cv2.Canny(self.cv_image, 128, 256)
		thresh, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		
		if height > 60 and width > 60:
			cv2.rectangle(self.cv_image, (width/3, height/6), (2*width/3, 5*height/6), (0,0,255));

		count = 0
		all_point = np.zeros((1,2))
		for cnt in contours:
			for point in cnt:
				if point[0][0] > width/4 and point[0][0] < 2*width/3 \
					and point[0][1] > height/6 and point[0][1] < 4*height/6:
					all_point += point[0]
					count += 1
				else:
					break
		all_point = all_point / count
		mind_point = all_point[0][0]
		if not math.isnan(mind_point):
			self.turn = (width/2 - mind_point) / 200
		else:
			self.turn = 0
		rospy.loginfo("Turn angle: %.2f"%(self.turn))

		

def thread_job():
	rospy.spin()
 
def main(args):
	rospy.init_node('image_converter', anonymous=True)
	rate = rospy.Rate(100);
	ic = image_converter(rate)
	try:
		add_thread = threading.Thread(target = thread_job)
    		add_thread.start()
		while not is_stop:
			pass
		threading.Thread._Thread__stop(add_thread)
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()
 
if __name__ == '__main__':
	main(sys.argv)