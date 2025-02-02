#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-
 
import roslib;
import rospy  
import actionlib  
from actionlib_msgs.msg import *  
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  
from random import sample  
from math import pow, sqrt  
from pokemon.srv import Location, LocationResponse
from tf.transformations import quaternion_from_euler
from pokemon_capture2 import pokemon_capture
from nav_msgs.msg import Odometry

class NavTest():  
    def __init__(self, id):  
        
        rospy.on_shutdown(self.shutdown)  

        self.id = id
        rospy.init_node('pokemon_capture_%d'%(self.id), anonymous=True)  
        self.pc = pokemon_capture(id=id, capture_distance=0.35, dis_eps=0.025,
                                  area_eps=0.5, move_speed=0.1, rotate_speed=0.1)
        # 在每个目标位置暂停的时间 (单位：s)
        self.rest_time = rospy.get_param("~rest_time", 2)  

        # 是否仿真？  
        self.fake_test = rospy.get_param("~fake_test", True)  

        # 到达目标的状态  
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED',   
                       'SUCCEEDED', 'ABORTED', 'REJECTED',  
                       'PREEMPTING', 'RECALLING', 'RECALLED',  
                       'LOST']  
 
        # 设置目标点的位置  
        # 在rviz中点击 2D Nav Goal 按键，然后单击地图中一点  
        # 在终端中就会看到该点的坐标信息  

        # 发布控制机器人的消息  
        self.cmd_vel_pub = rospy.Publisher('/tb3_%d/cmd_vel'%(self.id), Twist, queue_size=5)  

        # 订阅move_base服务器的消息  
        self.move_base = actionlib.SimpleActionClient("/tb3_%d/move_base"%(self.id), MoveBaseAction)  

        rospy.loginfo("Waiting for move_base action server...")  

        # 60s等待时间限制  
        self.move_base.wait_for_server(rospy.Duration(60))  
        rospy.loginfo("Connected to move base server")  
  
        # 保存机器人的在rviz中的初始位置  
        current_pose = rospy.wait_for_message('/tb3_%d/odom' % self.id, Odometry, timeout=5)
        last_location = current_pose.pose.pose

        # 保存成功率、运行时间、和距离的变量  
        n_goals = 0  
        n_successes = 0  
        i = 0  
        distance_traveled = 0  
        start_time = rospy.Time.now()  
        running_time = 0  

        self.catching_pokemon = -1
        self.goal_status = 0
        self.goal = MoveBaseGoal()  
        self.goal.target_pose.header.frame_id = 'map'  
        self.goal.target_pose.pose = current_pose.pose.pose
        
        self.goal.target_pose.header.stamp = rospy.Time.now()  
 
        # 确保有初始位置  
        while current_pose.header.stamp == "":  
            rospy.sleep(1)  

        rospy.loginfo("Starting navigation test")  

        # 开始主循环，随机导航  
        while not rospy.is_shutdown():  
            # 计数器加1  
            i += 1  
            n_goals += 1  

            current_pose = rospy.wait_for_message('/tb3_%d/odom' % self.id, Odometry, timeout=5).pose.pose
            
            # 跟踪行驶距离  
            # 使用更新的初始位置  
            distance = sqrt(pow(current_pose.position.x -   
                                last_location.position.x, 2) +  
                            pow(current_pose.position.y -   
                                last_location.position.y, 2))  
            # 存储上一次的位置，计算距离  
            last_location = current_pose
            # get next goal 
            pose = self.get_next_goal(current_pose, self.catching_pokemon, self.goal_status)
            if self.catching_pokemon == -1:
                rospy.loginfo("Finish.")
                break 

            # 设定下一个目标点  
            self.goal.target_pose.pose = pose
            self.goal.target_pose.header.stamp = rospy.Time.now()  

            # 向下一个位置进发  
            self.move_base.send_goal(self.goal)  

            # 五分钟时间限制  
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))

            # 查看是否成功到达  
            if not finished_within_time:  
                self.move_base.cancel_goal()  
                rospy.loginfo("Timed out achieving goal")  
                self.goal_status = 0
            else:  
                state = self.move_base.get_state()  
                if state == GoalStatus.SUCCEEDED:  
                    rospy.loginfo("Goal succeeded!")  
                    n_successes += 1  
                    distance_traveled += distance  
                    rospy.loginfo("State:" + str(state))  
                    self.start_capture()
                else:  
                  rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))  

                self.goal_status = state

            # 运行所用时间  
            running_time = rospy.Time.now() - start_time  
            running_time = running_time.secs / 60.0  

            # 输出本次导航的所有信息  
            rospy.loginfo("Success so far: " + str(n_successes) + "/" +   
                          str(n_goals) + " = " +   
                          str(100 * n_successes/n_goals) + "%")  

            rospy.loginfo("Running time: " + str(trunc(running_time, 1)) +   
                          " min Distance: " + str(trunc(distance_traveled, 1)) + " m")  

            rospy.sleep(self.rest_time) 

    def start_capture(self):
        try:
            self.pc.start_capture()
        except rospy.ROSInterruptException:
            pass
        except KeyboardInterrupt:
            print("Shutting down") 

    def get_next_goal(self, current_pose, catching_pokemon, goal_status):
        rospy.wait_for_service('manage')
        try:
            manage = rospy.ServiceProxy('manage', Location)
            x = current_pose.position.x
            y = current_pose.position.y
            next_goal = manage(self.id, catching_pokemon, goal_status, x, y)

            rospy.loginfo('Pokemon: %d  Location: (%.2f %.2f) Yaw: %.2f' % 
                         (next_goal.goal_id, next_goal.goal_x, next_goal.goal_y, next_goal.goal_yaw))
            q = quaternion_from_euler(0, 0, next_goal.goal_yaw)
            pose = Pose(Point(next_goal.goal_x, next_goal.goal_y, 0.000),  Quaternion(q[0], q[1], q[2], q[3]))
            self.catching_pokemon = next_goal.goal_id
            return pose 
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def update_initial_pose(self, initial_pose):  
        self.initial_pose = initial_pose  

    def shutdown(self):  
        rospy.loginfo("Stopping the robot...")  
        self.move_base.cancel_goal()  
        rospy.sleep(2)  
        self.cmd_vel_pub.publish(Twist())  
        rospy.sleep(1)  

def trunc(f, n):  
    slen = len('%.*f' % (n, f))  

    return float(str(f)[:slen])  

if __name__ == '__main__':  
    try:  
        NavTest(int(sys.argv[1]))  
        rospy.spin()  

    except rospy.ROSInterruptException:  
        rospy.loginfo("Exploring SLAM finished.")
