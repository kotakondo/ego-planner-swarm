#!/usr/bin/env python  
# Kota Kondo

#  Commands: Open three terminals and execute this:
#  python missed_msgs_count.py

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

    n_agents = 10
    cd_list = [0, 50, 100, 200, 300]

    for cd in cd_list:

        home_dir = "/media/kota/T7/data/ego_swarm_data"
        source_dir = home_dir+"/bags/cd"+str(cd)+"ms" # change the source dir accordingly #10 agents
            
        source_len = len(source_dir)
        source_bags = source_dir + "/*.bag" # change the source dir accordingly

        rosbag_list = glob.glob(source_bags)
        rosbag_list.sort() #alphabetically order
        rosbag = []

        for bag in rosbag_list:
            rosbag.append(bag)

        # going through the rosbags
        ave_list = [] # size will be num of sims
        for i in range(len(rosbag)):
            b = bagreader(rosbag[i], verbose=False)
            sim_id = rosbag[i][source_len+5:source_len+7]

            # going through the agents
            missed_msgs_list_per_sim = [] # size will be num_of_agents
            msgs_list_per_sim = [] # size will be num_of_agents
            
            msgs_cnt = 0
            missed_msgs_cnt = 0
            ave = 0

            for i in range(n_agents):
                log_data = b.message_by_topic("/drone_"+str(i)+"_ego_planner_node/comm_delay")
                # print(log_data)
                try:
                    log = pd.read_csv(log_data)

                    for j in range(len(log.comm_delay)):
                        msgs_cnt = msgs_cnt + 1
                        # print(log.comm_delay[j])
                        if log.comm_delay[j] > dc_in_s:
                            missed_msgs_cnt = missed_msgs_cnt + 1
                except:
                    pass

            try:
                ave = missed_msgs_cnt/msgs_cnt
                ave_list.append(ave)
                # os.system('echo "simulation '+sim_id+': missed_msgs_cnt'+ave_missed_msgs_cnt+'" >> '+source_dir+'/missed_msgs_cnt.txt')
            except:
                pass

        try:      
            ave_missed_per_dc = sum(ave_list)/len(ave_list)
            os.system('echo "'+source_dir+'" >> '+home_dir+'/missed_msgs_cnt.txt')
            os.system('echo " missed/total '+str(round(ave_missed_per_dc*100,2))+'%" >>  '+home_dir+'/missed_msgs_cnt.txt')
            os.system('echo "------------------------------------------------------------" >>  '+home_dir+'/missed_msgs_cnt.txt')
        except:
            pass
