#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys,os
import rospy
import rospkg
import numpy as np
from nav_msgs.msg import Path,Odometry
from std_msgs.msg import Float64,Int16,Float32MultiArray
from geometry_msgs.msg import PoseStamped,Point
# from sensor_msg.msg import LaserScan
from morai_msgs.msg import EgoVehicleStatus,ObjectStatusList,CtrlCmd,GetTrafficLightStatus,SetTrafficLight
from lib.utils import pathReader, findLocalPath,purePursuit,cruiseControl,vaildObject,pidController,velocityPlanning,latticePlanner
import tf
from math import cos,sin,sqrt,pow,atan2,pi

def traffic_callback(data):
    traffic = data
    iam = traffic.trafficLightStatus
    print('---------------traffic--------------- : ', iam)
    
if __name__ == '__main__':
    while not rospy.is_shutdown():
        rospy.init_node('wecar_planner', anonymous=True)
        rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus, traffic_callback)
        rospy.spin()
