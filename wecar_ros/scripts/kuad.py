#! /usr/bin/env python

from numpy import angle
import rospy
import rospkg
import numpy as np
import sys
from std_msgs.msg import Float64
from morai_msgs.msg import EgoVehicleStatus
from vesc_msgs.msg import VescStateStamped

class EgoReceiver():
    
    def __init__(self):

        rospy.init_node('WeWannaSpeed', anonymous=False)
        self.subEgo_speed = rospy.Subscriber("/sensors/core", VescStateStamped, self.speed_callback)
        self.subEgo_angle = rospy.Subscriber("/sensors/servo_position_command", Float64, self.angle_callback)
        self.subEgo_topic = rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.Ego_callback)
        
        self.pubEgo_speed = rospy.Publisher("/commands/motor/speed", Float64, queue_size=10)
        self.pubEgo_angle = rospy.Publisher("/commands/servo/position", Float64, queue_size=10)
        
        self.cmd_speed = 0
        self.cmd_angle = 0.5

        #rospack=rospkg.RosPack()
        #pkg_path = rospack.get_path('wecar_ros')
        #full_path = pkg_path+'/scripts/'+'Ego_data.txt'
        #self.f = open(full_path, 'a')

        rospy.on_shutdown(self.Ego_shutdown)
        rospy.spin()

    def Ego_callback(self, data):
        Ego_header = data.header
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
        
        #print(angle)
        
        #print("data len : {}".format(len(data.ranges)))

        #self.f.write(str(lat)+'\n')
        goal_speed = self.change_speed(accel)
        goal_angle = self.change_position(angle)

        custom_Ego_speed.data = goal_speed
        custom_Ego_position.data = goal_angle
        
        #self.pubEgo_speed.publish(custom_Ego_speed)
        #self.pubEgo_angle.publish(custom_Ego_position)
        
        #print(custom_Ego_speed)
    def speed_callback(self, data):
        ego_speed = data.state.speed
        if ego_speed < 2000:
            cmd_speed = 10000
        else:
            cmd_speed = 10000
        Ego_speed_msg = Float64()
        Ego_speed_msg.data = cmd_speed
        
        #print(cmd_speed)
        
        self.pubEgo_speed.publish(Ego_speed_msg)
    
    def angle_callback(self, data):
        ego_angle = data.data
        now_angle = ((ego_angle*2)-1)*19.5
        '''if now_angle < 10:
            cmd_angle = 11
        else:
            cmd_angle = 9'''
        cmd_angle = round(((ego_angle/19.5)+1)/2.5)
        if cmd_angle >= 1:
            cmd_angle = 1
        elif cmd_angle <= -1:
            cmd_angle = -1 
        
        Ego_angle_msg = Float64()
        Ego_angle_msg.data = cmd_angle
        
        #print(cmd_angle)
        
        self.pubEgo_angle.publish(Ego_angle_msg)
       
           
    def Ego_shutdown(self):
        print("I'm dead!")
        #self.f.close()
        custom_Ego_speed=0
        custom_Ego_position=0.5
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

    ER = EgoReceiver()
