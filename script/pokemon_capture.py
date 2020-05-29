#!/usr/bin/env python

import numpy as np
import threading
from geometry_msgs.msg import Twist
import math
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
import cv2
import rospy
import roslib
import sys
# import select
# import termios
# import tty

roslib.load_manifest("pokemon")


'''
[LaserScan]
angle_min: -0.521567881107
angle_max: 0.524276316166
angle_increment: 0.00163668883033
time_increment: 0.0
scan_time: 0.0329999998212
range_min: 0.449999988079
range_max: 10.0
'''


class state:
    IDLE = 1
    ROTATE = 2
    TRANSLATE = 4


def boundingBox(img, cnt, color=(0, 0, 255)):
    x, y, w, h = cv2.boundingRect(cnt)
    img = cv2.rectangle(img, (x, y), (x+w, y+h), color)
    return img


def he(img, use_clahe=True):
    channels = cv2.split(img)
    if use_clahe:
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        return cv2.merge([clahe.apply(c) for c in channels])
    else:
        return cv2.merge([cv2.equalizeHist(c) for c in channels])


def gamma_correction(img, gamma):
    lookUpTable = np.empty((1, 256), np.uint8)
    for i in range(256):
        lookUpTable[0, i] = np.clip(pow(i / 255.0, gamma) * 255.0, 0, 255)
    res = cv2.LUT(img, lookUpTable)
    return res


def harris_detect(img):
    height, width, channels = img.shape
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    dst = cv2.cornerHarris(gray, 5, 3, 0.06)
    dst = cv2.dilate(dst, cv2.getStructuringElement(
        cv2.MORPH_RECT, (5, 5)), iterations=1)
    mask = np.zeros((height, width), dtype=np.uint8)
    mask[dst > 0.01*dst.max()] = 255
    corners = np.where(mask == 255)
    xs = []
    ys = []
    for y, x in zip(*corners):
        if y > height*370/480 or y < height*10/480:
            continue
        xs.append(x)
        ys.append(y)

    center=None
    if len(xs)>0:
        x1 = min(xs)
        y1 = min(ys)
        x2 = max(xs)
        y2 = max(ys)
        img = cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255))

        center=((x1+x2)//2,(y1+y2)//2)
    
    # print(x1,y1,x2,y2)
    return img,center
    


def canny_detect(img):
    height, width, channels = img.shape
    mask = cv2.Canny(img, 100, 200)
    mask = cv2.dilate(mask, cv2.getStructuringElement(
        cv2.MORPH_RECT, (3, 3)), 1)

    _, counters, hierarchy = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = []
    for cnt in counters:
        drop_flag = False
        for p in cnt:
            if p[0][1] > height*410/480 or p[0][1] < height*10/480:
                drop_flag = True
                break
        if not drop_flag:
            cnts.append(cnt)

    center=None
    if len(cnts) > 0:
        areas = np.array([cv2.contourArea(cnt) for cnt in cnts])
        cnt = cnts[np.argmax(areas)]
        x, y, w, h = cv2.boundingRect(cnt)
        img = cv2.rectangle(img, (x, y), (x+w, y+h), (0, 0, 255))
        center=(x+w//2,y+h//2)
    
    return img,center

class pokemon_capture:
    def __init__(self, capture_distance, rate=200, move_speed=0.2, rotate_speed=0.1, dis_eps=0.05, pixel_eps=5):
        self.cv_image = None
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/camera/rgb/image_raw", Image, self.img_callback)
        self.imgae_pub = rospy.Publisher(
            '/pokemon/image_raw', Image, queue_size=10)
        self.scan_sub = rospy.Subscriber(
            "/scan", LaserScan, self.scan_callback)
        self.cmd_vel = rospy.Publisher(
            'cmd_vel_mux/input/teleop', Twist, queue_size=10)
        self.capture_distance = capture_distance
        self.turn = 0
        self.vel = 0
        self.move_speed = move_speed
        self.rotate_speed = rotate_speed
        self.pokemon_cnt = 0
        self.state = state.IDLE
        self.start_capture_flag = False
        self.detection_method = 'canny'
        self.dis_eps = dis_eps
        self.pixel_eps = pixel_eps
        self.action_rate = rospy.Rate(rate)
        self.action_pub_thread = threading.Thread(target=self.action)
        self.action_pub_thread.setDaemon(True)
        self.action_pub_thread.start()

    def close(self):
        self.imgae_pub.publish()
        self.cmd_vel.publish()

    def img_callback(self, data):
        # rate=20
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        height, width, channels = img.shape
        # 480,640,3 BGR

        if self.detection_method == 'canny':
            img,center=canny_detect(img)
        else:
            img,center=harris_detect(img)

        if center is not None:
            # set turn angle to center the pokemon
            if self.state == state.ROTATE:
                center_x = center[0]
                print('center_x:%d' % center_x)
                if center_x >= width//2+self.pixel_eps:
                    self.turn = -self.rotate_speed
                elif center_x <= width//2-self.pixel_eps:
                    self.turn = self.rotate_speed
                else:
                    self.turn = 0
                    self.state = state.TRANSLATE

        self.cv_image = img
        # cv2.imshow("cv_img",img)
        try:
            image_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        self.imgae_pub.publish(image_msg)

    def start_capture(self, method='canny'):
        rospy.loginfo("start pokemon capture")
        self.start_capture_flag = True
        self.state = state.ROTATE
        self.detection_method = method

    def save_img(self):
        cv2.imwrite("Pokemon%d.jpg" % self.pokemon_cnt, self.cv_image)

    def action(self):
        while True:
            if self.start_capture_flag:
                move_cmd = Twist()
                move_cmd.linear.x = self.vel
                move_cmd.angular.z = self.turn
                self.cmd_vel.publish(move_cmd)
                self.action_rate.sleep()

    def init_param(self):
        self.turn = 0
        self.vel = 0
        self.state = state.IDLE
        self.start_capture_flag = False
        self.detection_method = 'canny'

    def scan_callback(self, data):
        if not self.start_capture_flag:
            return
        # rate=9
        ranges = data.ranges  # tuple

        # range: (-30,30) num: 640
        # center: (-10,10)
        ranges_size = len(ranges)//3
        distance, count = 0, 0
        for dis in ranges[ranges_size:2*ranges_size]:
            if not math.isnan(dis):
                distance += dis
                count += 1
        distance = distance/count
        rospy.loginfo("%.3f meter away from pokemon." % (distance))

        if self.state == state.TRANSLATE:
            if distance > self.capture_distance+self.dis_eps:
                # let's go forward at 0.2 m/s
                self.vel = self.move_speed
            elif distance < self.capture_distance-self.dis_eps:
                self.vel = -self.move_speed
            else:
                self.init_param()

                self.pokemon_cnt += 1
                self.save_img()
                rospy.loginfo("I catch a pokemon. total:%d" % self.pokemon_cnt)


def main(args):
    rospy.init_node('pokemon_capture', anonymous=True)

    pc = pokemon_capture(capture_distance=0.8)
    rospy.loginfo(
        '\npress any key to capture pokemon.\nenter "exit" to exit node.\nenter "a" to use another method to capture pokemon.\n')
    try:
        # rospy.spin()
        while True:
            input_text = raw_input('open poke ball to capture pokemon?\n')
            # print(input_text)
            if input_text == 'exit':
                break
            elif input_text == 'a':
                pc.start_capture(method='harris')
            else:
                pc.start_capture(method='canny')

        # rospy.spin()
        pc.close()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
