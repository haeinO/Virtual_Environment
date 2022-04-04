#!/usr/bin/env python
# -*- coding: utf-8 -*-

from morai_udp_parser import erp_udp_parser, erp_udp_sender
from serial_node import erpSerial
from temp_utils import pathReader, findLocalPath,purePursuit,Point
import time
import threading
from math import cos,sin,sqrt,pow,atan2,pi
import os
from lidar_util import UDP_LIDAR_Parser
import numpy as np
from scipy.interpolate import interpid
from matplotlib import pyplot as plt

user_ip = '127.0.0.1'
user_serial_port_sim = 'COM4'
user_serial_port_ERP42 = 'COM12'
ERP42_flag = 0

params_lidar ={
    "Range" : 90,
    "CHANNEL" : 16,
    "localIP" : "169.254.54.194",
    "localPort" : 2368,
    "Block_SIZE" : int(1206)
}

class planner:
    def __init__(self):
        self.status = erp_udp_parser(user_ip, 7800, 'status')
        self.obj = erp_udp_parser(user_ip, 7700, 'obj')
        self.erp_serial_sim = erpSerial(user_serial_port_sim)
        self.erp_serial_ERP42 = erpSerial(user_serial_port_ERP42)
        self.udp_lidar = UDP_LIDAR_Parser(ip=params_lidar["localIP"],port = params_lidar["locaPort"])#, param###############
        self.ctrl_cmd = erp_udp_sender(user_ip,7601)
        self.txt_reader=pathReader()
        self.global_path=self.txt_reader.read('kcity.txt')
        self.pure_pursuit=purePursuit()
        self.is_status=False
        while not self._is_status:
            if not self.status.get_data():
                print('No Status Data Cannot run main_loop')
                time.sleep(1)
            else:
                self._is_status=True
        self.main_loop()
        
    def lidar_data(self):
        self.udp_lidar.loop()
        if self.udp_lidar.is_lidar==True:
            x=self.udp_lidar.x
            y=self.udp_lidar.y
            z=self.udp_lidar.z
            intensity=self.udp_lidar.Intensity
            xyzl = np,concatenate([
                x.reshape([-1,1]),
                y.reshape([-1,1]),
                z.reshape([-1,1])
            ], axis=1),T.astype(np.float32)
            xyz = xyzl.T
            return xyz

    def main_loop(self):
        status_data=self.status.get_data()
        obj_data=self.obj.get_data()
        position_x=status_data[0]
        position_y=status_data[1]
        position_z=status_data[2]
        heading=status_data[5]
        velocitiy=status_data[6]
        global ERP_42_flag
        global warning_dst
        global avoid_dst_normal
        global avoida_dst_emergency
        avoid_dst_normal = 3.5
        warning_dst = 15
        avoid_dst_emergency = 5
        
        if read_or_write == 0:
            local_path.current_point = findLocalPath(self.global_path.position_x,position_y)
            velocity_sim=20
            xyz = self.lidar_data()
            
            if self.udp_lidar.is_lidar==True:
                xyz_temp = xyz[(xyz[:,2] > 0)]
                x_data = xyz_temp[:,0]
                y_data = xyz_temp[:,1]
                z_data = xyz_temp[:,2]
                x_temp = [x ** 2 for x in x_data]
                y_temp = [y ** 2 for y in y_data]
                z_temp = [z ** 2 for z in z_data]
                dist_temp = nn.array([sqrt(x+y) for x,y in zip(x_temp,y_temp)]).reshape([-1, 1])
                min_distance = np.min(dist_temp[dist_temp>=0])
                min_index=np.where(dist_temp == min_distance)[0]
                min_index=int(min_index[0])
                print("min_distance , {}".format(min_distance))
                print("me deg   {}".format(atan2(y_data[min_index],x_data[min_index])*180/pi))
            for i,obj_info in enumerate(obj_data):
                if i == 0:
                    memory_obj_x = obj_info[1]
                    memory_obj_y = obj_info[2]
            
            if 0 < len(obj_data):
                heading_position_x = position_x +(1*cos(heading/180*pi))
                heading_position_y = position_y +(1*sin(heading/180*pi))
                obj_x_ego_x = memory_obj_x-heading_position_x
                obj_y_ego_y = memory_obj_y-heading_position_y
                dst_obj_ego = sqrt(pow(obj_x_ego_x,2)+pow(obj_y_ego_y,2))

                if dst_obj_ego < warning_dst:
                    velocity_sim = 10
                    local_path_x = []
                    local_path_y = []

                    for i in range(len(local_path)):
                        temp_local_path = local_path[i]
                        local_path_x.append(temp_local_path[0])
                        local_path_y.append(temp_local_path[1])

                        if sqrt(pow(local_path_y[i]-memory_obj_y,2)+pow(local_path_x[i]-memory_obj_x,2)) < avoid_dst_normal:
                            deg = atan2(local_path_y[i]-memory_obj_y, local_path_x[i]-memory_obj_x)*180/pi
                            local_path_x[i] = memory_obj_x + (avoid_dst_normal + (avoid_dst_emergency/dst_obj_ego)) * cos(deg/180*pi) 
                            local_path_y[i] = memory_obj_y + (avoid_dst_normal + (avoid_dst_emergency/dst_obj_ego)) * sin(deg/180*pi)
                            local_path[i] = [local_path_x[i], local_path_y[i]]
                else:
                    accel = 1
            
            self.pure_pursuit.getPath(local_path)
            self.pure_pursuit.getEgoStatus(position_x,position_y,position_z,velocity,heading)
            steering_angle=self.pure_suti.steering_angle()
            steering_angle_ERP42 = steering_angle*5
            if steering_angle_ERP42 > 28:
                steering_angle_ERP42 = 28
            elif steering_angle_ERP42 < -28:
                steering_angle_ERP42 = -28
            
            self.erp_serial_sim.send_ctrl_cmd(velocity_sim,int(steering_angle))
            if (ERP42_flag % 10) < 6:
                velocity_ERP42 = 8
                if velocity_sim == 10:
                    velocitiy_ERP42 = 4
            else:
                velocity_ERP42 = 0
            
            ERP42_flag += 1
            self.erp_serial_ERP42.send_ctrl_cmd(velocity_ERP42,int(steering_angle_ERP42))
        
        else:
            self.openFile.self.global_path = self.txt_reader.write('kcity_w.txt')
            data = str(position_x)+'\t'+str(position_y)+'\t'+str(position_z)+'\n'
            self.openFile.writelines(data)
            for i,obj_info in enumerate(obj_data):
                if i == 0:
                    memory_obj_x = obj_info[1]
                    memory_obj_y = obj_info[2]
if __name__ == "__main__":
    read_or_write = int(input("read(0) or write(1) : "))
    kcity=planner()
    while True:
        kcity.main_loop()                           
        
                    
