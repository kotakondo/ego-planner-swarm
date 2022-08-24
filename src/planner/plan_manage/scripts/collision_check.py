#!/usr/bin/env python  
# Kota Kondo

#  Commands: Open three terminals and execute this:
#  python collision_check.py

#Whenever you want, you can ctrl+C the second command and you will get the absolute and relative velocities

import bagpy
from bagpy import bagreader
import pandas as pd

import rosbag
import rospy
import math
import tf2_ros
import geometry_msgs.msg
import tf
import numpy as np
import matplotlib.pyplot as plt
import os
import glob

if __name__ == '__main__':

    cd_list = [0, 50, 100, 200, 300]

    for cd in cd_list:

        collision_cnt = 0

        # home_dir = "/media/kota/T7/data/ego_sw/arm_data"
        home_dir = "/home/kota/ego_swarm_data"
        source_dir = home_dir+"/bags/cd"+str(cd)+"ms" # change the source dir accordingly #10 agents
        
        source_len = len(source_dir)
        source_bags = source_dir + "/*.bag" # change the source dir accordingly

        rosbag_list = glob.glob(source_bags)
        rosbag_list.sort() #alphabetically order
        rosbag = []

        for bag in rosbag_list:
            rosbag.append(bag)

        for i in range(len(rosbag)):

            try:
                b = bagreader(rosbag[i], verbose=False)
                sim_id = rosbag[i][source_len+15:source_len+17]
                log_data = b.message_by_topic("/is_collided")
                if (log_data == None):
                    print("sim " + sim_id + ": no collision" )
                    os.system('echo "simulation '+sim_id+': no collision" >> '+source_dir+'/collision_status.txt')
                else:
                    collision_cnt = collision_cnt + 1
                    print("sim " + sim_id + ": ******collision******" )
                    os.system('echo "simulation '+sim_id+': ***collision***" >> '+source_dir+'/collision_status.txt')
            except:
                pass

        collision_per = 100 - collision_cnt / len(rosbag) * 100
        os.system('paste '+source_dir+'/collision_status.txt '+source_dir+'/status.txt >> '+source_dir+'/complete_status.txt')
        os.system('echo "'+source_dir+'" >> '+home_dir+'/collision_count.txt')
        os.system('echo "'+str(collision_cnt)+'/'+str(len(rosbag))+' - '+str(round(collision_per,2))+'%" >> '+home_dir+'/collision_count.txt')
        os.system('echo "------------------------------------------------------------" >> '+home_dir+'/collision_count.txt')
