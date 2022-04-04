#! /usr/bin/env python

from numpy import angle
import rospy
import rospkg
import numpy as np
import sys
from std_msgs.msg import Float64
from morai_msgs.msg import EgoVehicleStatus

class EgoReceiver():
    
    def __init__(self):

        rospy.init_node('Ego_py_receiver', anonymous=False)
        self.subEgo = rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.callback)
        self.pubEgo_speed = rospy.Publisher("/commands/motor/speed", Float64, queue_size=10)
        self.pubEgo_angle = rospy.Publisher("/commands/servo/position", Float64, queue_size=10)

        rospack=rospkg.RosPack()
        pkg_path = rospack.get_path('wecar_ros')
        full_path = pkg_path+'/scripts/'+'Ego_data.txt'
        self.f = open(full_path, 'a')

        rospy.on_shutdown(self.Ego_shutdown)
        rospy.spin()

    def callback(self, data):
        global custom_Ego_speed
        Ego_header = data.header
        custom_Ego_speed = 1
        id=data.unique_id
        
        acc_x=data.acceleration.x
        acc_y=data.acceleration.y
        acc_z=data.acceleration.z
        
        pos_x=data.position.x
        pos_y=data.position.y
        pos_z=data.position.z

        vel_x=data.velocity.x
        vel_y=data.velocity.y
        vel_z=data.velocity.z
        
        heading=data.heading
        accel=data.accel
        brake=data.brake
        angle=data.wheel_angle

        custom_Ego = EgoVehicleStatus()
        custom_Ego.header=data.header
        custom_Ego_speed = Float64()
        custom_Ego_position = Float64()
        
        #print("data len : {}".format(len(data.ranges)))

        #self.f.write(str(lat)+'\n')
        goal_speed = self.change_speed(accel)
        goal_angle = self.change_position(angle)

        custom_Ego_speed.data = goal_speed
        custom_Ego_position.data = goal_angle
        self.pubEgo_speed.publish(custom_Ego_speed)
        self.pubEgo_angle.publish(custom_Ego_position)
        
        print(custom_Ego_speed)
    
    def Ego_shutdown(self):
        print("I'm dead!")
        self.f.close()
        custom_Ego_speed=0
        custom_Ego_position=0
        self.pubEgo_speed.publish(custom_Ego_speed)
        self.pubEgo_angle.publish(custom_Ego_position)
        
    def change_speed(self, accel):
        accel += 0.1
  
        #print(accel)
        return accel
    
    def change_position(self, angle):
        if angle < 100:
            angle += 1
        else :
            angle -= 1

        return angle

if __name__=="__main__":
    while not rospy.is_shutdown():
        custom_Ego_speed = 1
        ER = EgoReceiver()
