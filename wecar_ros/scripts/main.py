#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys,os
import rospy
import rospkg
import numpy as np
import cv2
import time
from nav_msgs.msg import Path,Odometry
from std_msgs.msg import Float64,Int16,Float32MultiArray
from geometry_msgs.msg import PoseStamped,Point,Point32
from morai_msgs.msg import EgoVehicleStatus,ObjectStatusList,CtrlCmd,GetTrafficLightStatus,SetTrafficLight
from lib.utils import pathReader,findLocalPath,cruiseControl,purePursuit,vaildObject,velocityPlanning,latticePlanner
import tf
import math
from math import cos,sin,sqrt,pow,atan,atan2,pi
from sensor_msgs.msg import Image,CompressedImage,LaserScan,PointCloud

class wecar_planner():
    def __init__(self):
        rospy.init_node('wecar_planner', anonymous=True)

        arg = rospy.myargv(argv=sys.argv)
        self.path_name = arg[1]

        #publisher
        global_path_pub= rospy.Publisher('/global_path',Path, queue_size=1) ## global_path publisher
        local_path_pub= rospy.Publisher('/local_path',Path, queue_size=1) ## local_path publisher
        self.motor_pub = rospy.Publisher('commands/motor/speed',Float64, queue_size=1)
        self.servo_pub = rospy.Publisher('commands/servo/position',Float64, queue_size=1)
        ########################  lattice  ########################
        for i in range(1,8):
            globals()['lattice_path_{}_pub'.format(i)]=rospy.Publisher('lattice_path_{}'.format(i),Path,queue_size=1)
        ########################  lattice  ########################

        #self.obj_pub = rospy.Publisher('lidar2D', LaserScan, queue_size=1)

        #subscriber
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.statusCB) ## Vehicl Status Subscriber
        rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus, self.traffic_callback)
        rospy.Subscriber("/lidar2D", LaserScan, self.laser_callback)

        self.is_status=False ## 차량 상태 점검
        #self.is_obj=False ## 장애물 상태 점검
        self.steering_angle_to_servo_offset=0.5304 ## servo moter offset
        self.rpm_gain = 4616 #4616
        self.motor_msg=Float64()
        self.servo_msg=Float64()

        self.object_num = 0
        self.no_white = 0
        self.traffic_value = 0
        self.cmd_speed = 0
        self.cmd_angle = 0.5
        self.heading = 0
        self.rotary_angle = 0
        self.final_angle = 0
        self.final_angle_1 = 0
        self.dynamic_object = 0
        self.static_object = 0
        self.flag = 0
        self.obstacle_count = 0
        self.distance = 0
        self.count = 0
        self.dynamic_motor_msg = 0
        self.obstacle_front = 0
        self.obstacle_back = 0

        #class
        path_reader=pathReader('wecar_ros') ## 경로 파일의 위치
        pure_pursuit=purePursuit() ## purePursuit import
        self.cc=cruiseControl(0.5,1) ## cruiseControl import (object_vel_gain, object_dis_gain)
        #self.vo=vaildObject() ## 장애물 유무 확인 ()
        #pid=pidController() ## pidController import

        #read path
        self.global_path=path_reader.read_txt(self.path_name+".txt") ## 출력할 경로의 이름
        vel_planner=velocityPlanning(10,0.15) ## 속도 계획
        vel_profile=vel_planner.curveBasedVelocity(self.global_path,30)

        #time var
        count = 0
        rate = rospy.Rate(30) # 30hz
        obstacle_flag = True
        MODE = 0
        mode = 0
        check = []
        location1 = 0
        location2 = 0
        weight = [7,5,3,0,2,4,6]
        angle = 0
        lattice_current_lane = 3
        r = 0
        x = -10
        parameter = 0

        while not rospy.is_shutdown():

            if self.is_status==True:
                local_path,self.current_waypoint=findLocalPath(self.global_path,self.status_msg)
                vehicle_status=[self.status_msg.position.x,self.status_msg.position.y,(self.status_msg.heading)/180*pi,self.status_msg.velocity.x/3.6]
                lattice_path,selected_lane=latticePlanner(local_path, vehicle_status,lattice_current_lane,weight)
                lattice_current_lane=selected_lane

                if selected_lane != -1:
                    local_path=lattice_path[selected_lane]

                if len(lattice_path)==7:
                    for i in range(1,8):
                        globals()['lattice_path_{}_pub'.format(i)].publish(lattice_path[i-1])

                pure_pursuit.getPath(local_path) # pure_pursuit 알고리즘에 Local path 적용
                pure_pursuit.getEgoStatus(self.status_msg) # pure_pursuit 알고리즘에 차량의 status 적용

                self.steering=pure_pursuit.steering_angle()

                self.cc_vel = 10
                self.servo_msg = self.steering*0.021 + self.steering_angle_to_servo_offset
                self.motor_msg = self.cc_vel *self.rpm_gain/3.6

                if not self.status_msg.position.x < 0:
                    if self.servo_msg < 0.42 or self.servo_msg > 0.58 :
                        self.motor_msg = 1700
                    else:
                        self.motor_msg = 5000
                # dynamic
                else:
                    if self.flag == 1:
                        self.motor_msg = 0
                        self.motor_pub.publish(self.motor_msg)
                        self.dynamic_motor_msg = self.motor_msg

                        if self.dynamic_motor_msg == 0:
                            self.count += 0.01

                        if self.count < 0.8:
                            pass
                        else:
                            for i in range(5000):
                                self.motor_msg = 5000
                                self.motor_pub.publish(self.motor_msg)
                            self.count = 0
                            self.flag = 0

                    else:
                        self.count = 0
                        self.motor_msg = 5000
                        self.motor_pub.publish(self.motor_msg)

                # start
                if (19.25 < self.status_msg.position.x) and (self.status_msg.position.x< 19.45) and (-2.30 > self.status_msg.position.y) and (self.status_msg.position.y> -2.50):
                    print('stop for 5 seconds')
                    for i in range(1000):
                        self.motor_msg = 0
                        self.motor_pub.publish(self.motor_msg)
                        time.sleep(0.0055)
                    self.motor_msg = 5000
                    self.motor_pub.publish(self.motor_msg)

                # end
                elif (4.2 < self.status_msg.position.x) and (self.status_msg.position.x < 4.6) and (-5.3 > self.status_msg.position.y) and (self.status_msg.position.y > -5.5):
                    print('----------finish----------')

                elif (4.21 < self.status_msg.position.x) and (self.status_msg.position.x < 4.75) and (0.60 > self.status_msg.position.y) and (self.status_msg.position.y > 0.42):
                    self.path_name = 'second'
                    print("second_path")
                    self.global_path=path_reader.read_txt(self.path_name+".txt")

                elif (-1.05 < self.status_msg.position.x) and (self.status_msg.position.x < -0.85) and (-0.9 > self.status_msg.position.y) and (self.status_msg.position.y > -1.3):
                    self.path_name = 'third'
                    print("third_path")
                    self.global_path=path_reader.read_txt(self.path_name+".txt")

                # Obstacle
                if self.status_msg.position.x >= -17.5 and self.status_msg.position.x <= -4.4 and self.status_msg.position.y >= -5.6 and self.status_msg.position.y <= -4.6:
                    if len(check) != 0:
                        if check[0] > -6.45:
                            parameter = 1
                        else:
                            parameter = 0
                    mode = 0
                    self.final_angle = 0
                    self.distance = 2.0
                    self.dynamic_object = 1
                elif self.status_msg.position.x > 0:
                    mode = 0
                    self.dynamic_object = 2

                else:
                    self.dynamic_object = 0

                if self.status_msg.position.x >= -15 and self.status_msg.position.x <= -3.75 and self.status_msg.position.y >= 4.7 and self.status_msg.position.y <= 5.7:
                    self.obstacle_count = 0
                    self.static_object = 1
                else:
                    self.static_object = 0

                if self.status_msg.position.x < 0:
                    if self.status_msg.position.y > 0:
                        if self.static_object == 1:
                            if parameter == 0:
                                if self.distance != 0 and self.distance < 1.46 and mode == 0 and (self.final_angle < 11 and self.final_angle > -50):
                                    self.servo_msg = 0.5 -(self.status_msg.position.y - 5.15)/1.35
                                    self.motor_msg = 1100
                                    self.motor_pub.publish(self.motor_msg)

                                    if self.final_angle > -43 and self.final_angle < -33:
                                        x = self.status_msg.position.x
                                        check.append(x)
                                        mode = 1

                                elif mode == 1:
                                    if self.final_angle > 22:
                                        mode = 2
                                    self.motor_msg = 1700
                                    self.motor_pub.publish(self.motor_msg)

                                elif mode == 2:
                                    if self.distance != 0 and self.distance < 1.46 and (self.final_angle < 11 and self.final_angle > -50):
                                        #self.servo_msg = 0.5 -(self.status_msg.position.y - 5.15)/1.35
                                        self.servo_msg = 0.5 -(self.status_msg.position.y - 5.03)/1.12
                                        self.motor_msg = 1100
                                        self.motor_pub.publish(self.motor_msg)

                                        if self.final_angle > -43 and self.final_angle < -33:
                                            x = self.status_msg.position.x
                                            check.append(x)
                                            mode = 0
                                    else:
                                        self.motor_msg = 1700
                                        self.motor_pub.publish(self.motor_msg)

                                else:
                                    self.motor_msg = 1955
                                    self.motor_pub.publish(self.motor_msg)
                            else:
                                if self.distance != 0 and self.distance < 1.46 and mode == 0:
                                    self.servo_msg = 0.5 -(self.status_msg.position.y - 5.03)/1.114
                                    self.motor_msg = 1100
                                    self.motor_pub.publish(self.motor_msg)

                                    if self.final_angle > -43 and self.final_angle < -33:
                                        mode = 1

                                if mode == 1:
                                    if self.final_angle > 16:
                                        mode = 3

                                    self.motor_msg = 1700
                                    self.motor_pub.publish(self.motor_msg)

                                elif mode == 3:
                                    if self.distance != 0 and self.distance < 1.46 and (self.final_angle < 6 and self.final_angle > -50):
                                        self.servo_msg = 0.5 -(self.status_msg.position.y - 5.023)
                                        self.motor_msg = 1100
                                        self.motor_pub.publish(self.motor_msg)

                                        if self.final_angle > -41 and self.final_angle < -31:
                                            mode = 0
                                    else:
                                        self.motor_msg = 1900
                                        self.motor_pub.publish(self.motor_msg)

                                else:
                                    self.motor_msg = 1700
                                    self.motor_pub.publish(self.motor_msg)
                        else:
                            self.motor_msg = 5000
                            self.motor_pub.publish(self.motor_msg)

                # Rotary

                # speed downgrade
                if ((15.15 <= self.status_msg.position.x) and (self.status_msg.position.x < 17.40) and (0.620 > self.status_msg.position.y) and (self.status_msg.position.y > 0.40)):
                    self.motor_msg = 2250
                    self.motor_pub.publish(self.motor_msg)

                if ((10.58 < self.status_msg.position.x) and (self.status_msg.position.x < 15.15) and (1.6 > self.status_msg.position.y) and (self.status_msg.position.y > -1.52)):
                    if MODE == 0:
                        for i in range(1000):
                            self.motor_msg = 0
                            self.motor_pub.publish(self.motor_msg)
                            time.sleep(0.0005)
                        MODE = 1
                    else:
                        if self.obstacle_front == 1 and self.obstacle_back == 0:
                            self.motor_msg = self.rotary_distance * 2000 - 2500
                            if self.motor_msg <= 0:
                                self.motor_msg = 0
                                self.motor_pub.publish(self.motor_msg)
                            elif self.motor_msg >= 1200:
                                self.motor_msg = 1200
                        elif self.obstacle_back == 1 and self.obstacle_front == 1:
                            self.motor_msg = 1000
                            self.motor_pub.publish(self.motor_msg)
                        else:
                            self.motor_msg = 1700
                            self.motor_pub.publish(self.motor_msg)

                else:
                    MODE = 0

                #Traffic Sign
                if self.traffic_value != 33:
                    if (6.8 < self.status_msg.position.x) and (self.status_msg.position.x< 7.4) and (-2.50 < self.status_msg.position.y) and (self.status_msg.position.y < -1.98):
                        self.motor_msg = 0
                        self.motor_pub.publish(self.motor_msg)
                    elif (6 < self.status_msg.position.x) and (self.status_msg.position.x< 6.7) and (1.9 < self.status_msg.position.y) and (self.status_msg.position.y < 2.45):
                        self.motor_msg = 0
                        self.motor_pub.publish(self.motor_msg)

                local_path_pub.publish(local_path) # Local Path 출력

                self.servo_pub.publish(self.servo_msg)
                self.motor_pub.publish(self.motor_msg)

            if count==300 : # global path 출력
                global_path_pub.publish(self.global_path)
                count=0
            count+=1

            rate.sleep()

    def laser_callback(self, msg):
        pcd=PointCloud()
        motor_msg=Float64()
        pcd.header.frame_id=msg.header.frame_id
        angle = 0
        angle_print = 0
        object_angle = []
        x_array = []
        y_array = []
        x_mean = 0
        y_mean = 0
        sum_object = 0
        average_a = 0
        average_b = 0
        self.Obstacle_Mode = 0
        self.obstacle_front = 0
        self.obstacle_back = 0
        filtered_angle = 0

        for r in msg.ranges:

            tmp_point=Point32()

            #rotary
            if self.dynamic_object == 2:
                tmp_point.x=r*cos(angle)
                tmp_point.y=r*sin(angle)

                if r < 1.5:
                    self.rotary_distance = r
                    if tmp_point.x == 0:
                        tmp_point.x = 0.0001

                    object_angle.append(atan(-tmp_point.y/abs(tmp_point.x)))
                    x_array.append(tmp_point.x)
                    y_array.append(tmp_point.y)
                    sum_object = 0
                    for i in range(len(object_angle)):
                        sum_object += object_angle[i]
                        x_mean += x_array[i]
                        y_mean += y_array[i]
                    self.rotary_angle = (sum_object/len(object_angle))*180/pi

                    if tmp_point.x <= 0 :
                        self.obstacle_front = 1
                    elif tmp_point.x > 0 and tmp_point.x < 1.5:
                        self.obstacle_back = 1

                angle = angle+(1.0/180*pi)
                angle_print = angle * 180 / pi

            #dynamic
            elif self.dynamic_object == 1:
                if angle >= pi/2 and angle <= 3*pi/2:
                    tmp_point.x=r*cos(angle)
                    tmp_point.y=r*sin(angle)
                if r < 1.4:
                    if tmp_point.x == 0:
                        tmp_point.x = 0.0001
                    if tmp_point.x == 0.0001 and tmp_point.y == 0.0:
                        pass
                    object_angle.append(atan(-tmp_point.y/abs(tmp_point.x)))
                    x_array.append(tmp_point.x)
                    y_array.append(tmp_point.y)
                    sum_object = 0
                    for i in range(len(object_angle)):
                        sum_object += object_angle[i]
                        x_mean += x_array[i]
                        y_mean += y_array[i]

                    #angle calculate
                    self.final_angle_1 = (sum_object/len(object_angle))*180/pi

                    average_a = sum_object / len(object_angle)
                    average_b = average_a*180/pi
                    x_mean = x_mean / len(object_angle)
                    y_mean = y_mean / len(object_angle)
                    obstacle_distance = sqrt(pow(x_mean,2) + pow(y_mean,2))

                    if x_mean < 0:
                        filtered_angle = atan(y_mean/x_mean) * 180 / pi
                    else:
                        filtered_angle = 0

                    #obstacle_count 1 : left, 2: right
                    if self.obstacle_count == 0 and (filtered_angle > 1 and filtered_angle < 57):
                        self.flag = 1
                        self.obstacle_count = 1

                    elif self.obstacle_count == 0 and (filtered_angle < -1 and filtered_angle > -47):
                        self.flag = 1
                        self.obstacle_count = 2

                    if self.obstacle_count == 1:
                        if (filtered_angle < 0 and r > 0.8):
                            if filtered_angle < -49:
                                self.flag = 0
                                self.count = 0

                                for i in range(1000):
                                    self.motor_msg = 5000
                                    self.motor_pub.publish(self.motor_msg)
                                    time.sleep(0.0006)

                                self.obstacle_count = 0
                            else:
                                self.flag = 1
                        else:
                            self.flag = 1

                    elif self.obstacle_count == 2:
                        if (filtered_angle > 0 and r > 0.8):
                            if filtered_angle > 58:
                                self.count = 0
                                self.flag = 0
                                for i in range(1000):
                                    self.motor_msg = 5000
                                    self.motor_pub.publish(self.motor_msg)
                                    time.sleep(0.0006)
                                self.obstacle_count = 0
                            else:
                                self.flag = 1
                        else:
                            self.flag = 1

                angle = angle + (1.0/180*pi)
                angle_print = angle * 180 / pi

            #static
            elif self.static_object == 1:
                angle = angle+(1.0/180*pi)
                angle_print = angle * 180 / pi
                if angle >= 13*pi/18 and angle <= 23*pi/18:
                    tmp_point.x=r*cos(angle)
                    tmp_point.y=r*sin(angle)

                if r < 1.46:
                    self.distance = r
                    if tmp_point.x == 0:
                        tmp_point.x = 0.0001

                    if tmp_point.x == 0.0001 and tmp_point.y == 0.0:
                        pass
                    else:
                        object_angle.append(atan(-tmp_point.y/abs(tmp_point.x)))
                        x_array.append(tmp_point.x)
                        y_array.append(tmp_point.y)
                        average_b = 0
                        sum_object = 0
                        for i in range(len(object_angle)):
                            sum_object += object_angle[i]
                            x_mean += x_array[i]
                            y_mean += y_array[i]

                        #angle calculate
                        self.final_angle = (sum_object/len(object_angle))*180/pi
                        if y_mean < 0:
                            sum_object = len(object_angle)*pi/2 - sum_object

                        average_a = sum_object / len(object_angle)
                        average_b = average_a*180/pi
                        x_mean = x_mean / len(object_angle)
                        y_mean = y_mean / len(object_angle)


    def statusCB(self, data): # Vehicle Status Subscriber
        self.status_msg=data
        br = tf.TransformBroadcaster()
        br.sendTransform((self.status_msg.position.x, self.status_msg.position.y, self.status_msg.position.z),
                        tf.transformations.quaternion_from_euler(0, 0, (self.status_msg.heading)/180*pi),
                        rospy.Time.now(),
                        "gps",
                        "map")
        self.is_status=True

    def traffic_callback(self, data):
        traffic = data
        self.traffic_value = traffic.trafficLightStatus

if __name__ == '__main__':
    try:
        kcity_pathtracking=wecar_planner()
    except rospy.ROSInterruptException:
        pass
