#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('pokemon')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class image_converter:

	def __init__(self):
		self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=1)

		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)

	def callback(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		(height,width,channels) = cv_image.shape
		
		thresh = cv2.Canny(cv_image, 128, 256)
		thresh, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		im2 = draw_min_rect_circle(cv_image, contours)
		count = 0
		all_point = np.zeros((1,2))
		for cnt in contours:
			for point in cnt:
				if point[0][0] > width/4+1 and point[0][0] < 2*width/3-1 \
					and point[0][1] > height/6+1 and point[0][1] < 4*height/6-1:
					all_point += point[0]
					count += 1
				else:
					break
		all_point = all_point / count
		print(all_point)
		cv2.imshow("contours", thresh)
		cv2.waitKey(3)
			
		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
		except CvBridgeError as e:
			print(e)

def draw_contours(img, cnts):  # conts = contours
	img = np.copy(img)
	img = cv2.drawContours(img, cnts, -1, (0, 255, 0), 2)
	return img


def draw_min_rect_circle(img, cnts):  # conts = contours
	img = np.copy(img)

	for cnt in cnts:
		x, y, w, h = cv2.boundingRect(cnt)
		cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)  # blue

		min_rect = cv2.minAreaRect(cnt)  # min_area_rectangle
		min_rect = np.int0(cv2.boxPoints(min_rect))
		cv2.drawContours(img, [min_rect], 0, (0, 255, 0), 2)  # green

		(x, y), radius = cv2.minEnclosingCircle(cnt)
		center, radius = (int(x), int(y)), int(radius)  # center and radius of minimum enclosing circle
		img = cv2.circle(img, center, radius, (0, 0, 255), 2)  # red
	return img

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
	rospy.spin()
  except KeyboardInterrupt:
	print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)