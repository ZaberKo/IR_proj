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
    TRANSLATE1 = 4
    TRANSLATE2 = 8


def boundingBox(img, x, y, w, h, color=(0, 0, 255),thickness=2):
    img = cv2.rectangle(img, (x, y), (x+w, y+h), color,thickness=2)
    return img


# def he(img, use_clahe=True):
#     channels = cv2.split(img)
#     if use_clahe:
#         clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
#         return cv2.merge([clahe.apply(c) for c in channels])
#     else:
#         return cv2.merge([cv2.equalizeHist(c) for c in channels])


# def gamma_correction(img, gamma):
#     lookUpTable = np.empty((1, 256), np.uint8)
#     for i in range(256):
#         lookUpTable[0, i] = np.clip(pow(i / 255.0, gamma) * 255.0, 0, 255)
#     res = cv2.LUT(img, lookUpTable)
#     return res


def check_ratio(w,h,ratio_threshold=0.9):
    w=float(w)
    h=float(h)
    if w>h:
        # print(h/w)
        return h/w>=ratio_threshold
    else:
        # print(w/h)
        return w/h>=ratio_threshold


def canny_detect(img,area_eps=0.05,ratio_threshold=0.9):
    height, width, channels = img.shape
    mask = cv2.Canny(img, 100, 200)
    mask = cv2.dilate(mask, cv2.getStructuringElement(
        cv2.MORPH_RECT, (3, 3)), 1)

    _, counters, hierarchy = cv2.findContours(
        mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    cnts = []
    for cnt in counters:
        drop_flag = False
        for p in cnt:
            p_y=p[0][1]
            if p_y > height*1000/1080 or p_y < height*50/1080:
                drop_flag = True
                break
        if not drop_flag:
            cnts.append(cnt)

    # cv2.drawContours(img,cnts,-1,(255,255,0),2)

    center=None
    if len(cnts) >=2:
        areas = np.array([cv2.contourArea(cnt) for cnt in cnts])
        indices=np.argsort(areas)[::-1]

        cnt_r=cnts[indices[0]]
        area_r=areas[indices[0]]
        for idx in indices[1:]:
            # nonMaximumSuppression
            if areas[idx]/area_r>1-area_eps:
                continue

            cnt_g=cnts[idx]
            break
        xr, yr, wr, hr=cv2.boundingRect(cnt_r)
        xg, yg, wg, hg=cv2.boundingRect(cnt_g)
        
        # double check green rect is in red rect
        if xr<xg and yr<yg and xr+wr>xg+wg and yr+hr>yg+hg:
            # check rect ratio
            if check_ratio(wr,hr,ratio_threshold) and check_ratio(wg,hg,ratio_threshold-0.1):
                img=boundingBox(img,xr, yr, wr, hr,color=(0,0,255))
                img=boundingBox(img,xg, yg, wg, hg,color=(0,255,0))
                center=(xr+wr//2,yr+hr//2)

    return img,center



def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

class pokemon_capture:
    def __init__(self, capture_distance, rate=100, move_speed=0.1, rotate_speed=0.1, dis_eps=0.025, pixel_eps=5,area_eps=0.1,ratio_threshold=0.7):
        self.cv_image = None
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/camera/rgb/image_raw", Image, self.img_callback)
        self.imgae_pub = rospy.Publisher(
            '/pokemon/image_raw', Image, queue_size=10)
        self.scan_sub = rospy.Subscriber(
            "/scan", LaserScan, self.scan_callback)
        self.cmd_vel = rospy.Publisher(
            '/cmd_vel', Twist, queue_size=10)
        self.capture_distance = capture_distance
        self.turn = 0
        self.vel = 0
        self.move_speed = move_speed
        self.rotate_speed = rotate_speed
        self.pokemon_cnt = 0
        self.state = state.IDLE
        self.start_capture_flag = False
        self.dis_eps = dis_eps
        self.pixel_eps = pixel_eps
        self.area_eps=area_eps
        self.ratio_threshold=ratio_threshold
        self.action_rate = rospy.Rate(rate)
        self.action_pub_thread = None
        # self.action_pub_thread.setDaemon(True)
        # self.action_pub_thread.start()

    # def close(self):
    #     self.imgae_pub.publish()
    #     self.cmd_vel.publish()

    def __del__(self):
        self.imgae_pub.publish()
        self.cmd_vel.publish()
        self.init_param()

    def img_callback(self, data):
        # rate=20
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        height, width, channels = img.shape
        # 1080,1920,3 BGR

        img,center=canny_detect(img,self.area_eps,self.ratio_threshold)

        if center is not None:
            # set turn angle to center the pokemon
            if self.start_capture_flag :
                center_x = center[0]
                decay_pixel=50
                if center_x >= width//2+self.pixel_eps:
                    self.turn = -min((self.rotate_speed/decay_pixel)*(center_x-width//2),self.rotate_speed)
                elif center_x <= width//2-self.pixel_eps:
                    self.turn = min((self.rotate_speed/decay_pixel)*(width//2-center_x),self.rotate_speed)
                else:
                    self.turn = 0
                    # self.state = state.TRANSLATE1

                print('center_x diff:%d' % abs(center_x-width//2))
                print('turn speed:%f'%self.turn)

        self.cv_image = img
        # cv2.imshow("cv_img",img)
        try:
            image_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        self.imgae_pub.publish(image_msg)

    def start_capture(self):
        rospy.loginfo("start pokemon capture")
        self.start_capture_flag = True
        self.state = state.TRANSLATE1
        self.action()

    def save_img(self):
        cv2.imwrite("Pokemon%d.jpg" % self.pokemon_cnt, self.cv_image)

    def action(self):
        while self.start_capture_flag:
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

    def scan_callback(self, data):
        if not self.start_capture_flag:
            return
        # rate=30
        ranges = data.ranges  # tuple
        angle_increment=data.angle_increment
        # range: (0,360) num: 720
        

        # f,l,b,r=(0,90,180,270)

        # center: (-5,5)
        angle1_idx=int(math.radians(-5)/angle_increment)
        angle2_idx=int(math.radians(5)/angle_increment)
        distance, count = 0, 0

        for dis in ranges[angle1_idx-1:]+ranges[:angle2_idx+2]:
            if not math.isnan(dis) and not math.isinf(dis):
                distance += dis
                count += 1

        if count==0:
            distance=-1
        else:
            distance = distance/count
        
        rospy.loginfo("%.3f meter away from pokemon." % (distance))

        if self.state == state.TRANSLATE1:
            if distance > self.capture_distance+self.dis_eps:
                self.vel=min(self.move_speed,self.vel+0.01)
            elif distance < self.capture_distance-self.dis_eps:
                self.vel=max(-self.move_speed,self.vel-0.01)
            

            eps=1e-7
            if self.vel<self.move_speed+eps and self.vel>self.move_speed-eps:
                self.state=state.TRANSLATE2
                print('trun to TRANSLATE2')
    
        elif self.state==state.TRANSLATE2:
            decay_dis=0.1
            if distance > self.capture_distance+self.dis_eps:
                self.vel = min((self.move_speed/decay_dis)*(distance-self.capture_distance),self.move_speed)
            elif distance < self.capture_distance-self.dis_eps:
                self.vel = -min((self.move_speed/decay_dis)*(self.capture_distance-distance),self.move_speed)
            else:
                self.vel=0
            
        
        print('speed:%f'%self.vel)
        if distance <= self.capture_distance+self.dis_eps and distance >= self.capture_distance-self.dis_eps and self.turn==0:
            self.init_param()
            self.pokemon_cnt += 1
            self.save_img()
            rospy.loginfo("I catch a pokemon. total:%d" % self.pokemon_cnt)
                  



def main(args):
    rospy.init_node('pokemon_capture', anonymous=True)

    pc = pokemon_capture(capture_distance=0.8)
    rospy.loginfo(
        '\npress any key to capture pokemon.\nenter "exit" to exit node.\n')
    try:
        # rospy.spin()
        while True:
            input_text = raw_input('open poke ball to capture pokemon?\n')
            # print(input_text)
            if input_text == 'exit':
                break
            else:
                pc.start_capture()

        # rospy.spin()
        # pc.close()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
