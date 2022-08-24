#!/usr/bin/env python  
# Kota Kondo

#  python collision_check.py

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
import statistics

if __name__ == '__main__':

    num_of_agents = 10

    # stop count torelance
    stop_cnt_tol = 1e-3

    cd_list = [0, 50, 100, 200, 300]

    for cd in cd_list:

            home_dir = "/media/kota/T7/data/ego_swarm_data"

            # source directory
            source_dir = home_dir+"/bags/cd"+str(cd)+"ms" # change the source dir accordingly #10 agents
            
            source_len = len(source_dir)
            source_bags = source_dir + "/*.bag" # change the source dir accordingly
            rosbag_list = glob.glob(source_bags)
            rosbag_list.sort() #alphabetically order
            rosbag = []

            for bag in rosbag_list:
                rosbag.append(bag)

            # read ros bags
            total_dist_list = []
            stop_cnt_list = []
            for i in range(len(rosbag)):

                is_skip_bag = False

                print('rosbag ' + str(rosbag[i]))
                b = bagreader(rosbag[i], verbose=False)
                sim_id = rosbag[i][source_len+5:source_len+7]
                
                # introduced goal_reached topic so no need to check actual_traj

                dist = 0
                stop_cnt = 0
                for i in range(num_of_agents):
                    
                    stopped = True # in the beginning it is stopped
                    log_data = b.message_by_topic("/drone_" + str(i) + "_visual_slam/odom")

                    # print(log_data)
                
                    log = pd.read_csv(log_data, usecols=["pose.pose.position.x", "pose.pose.position.y", "pose.pose.position.z", "twist.twist.linear.x", "twist.twist.linear.y", "twist.twist.linear.z"])
                    
                    # print(log.columns)

                    log.columns = ["px", "py", "pz", "vx", "vy", "vz"]
                    
                    # print(log)
                    
                    ###### difference from the previous pos to the current pos
                    # print(log.diff().to_numpy())
                    diff_matrix = log.diff().to_numpy()

                    # since the first row is NaN, starts 
                    for i in range(1, len(log.diff())):
                        dist += np.linalg.norm(log.diff().to_numpy()[i,0:2])

                    ###### stop count
                    for i in range(len(log.to_numpy())):
                        if np.linalg.norm(log.to_numpy()[i,3:5]) > stop_cnt_tol:
                            stopped = False
                        elif np.linalg.norm(log.to_numpy()[i,3:5]) < stop_cnt_tol and not stopped:
                            stop_cnt = stop_cnt + 1
                            stopped = True

                    stop_cnt = stop_cnt - 1 # for the last stop

                    # in case of collision, stop_cnt won't work, so need to skip the bag
                    if (stop_cnt < 0):
                        is_skip_bag = True
                        print("skip the bag")
                        break

                if not is_skip_bag:
                    dist /= num_of_agents
                    stop_cnt /= num_of_agents
                    total_dist_list.append(dist)
                    stop_cnt_list.append(stop_cnt)

                is_skip_bag = False

            ave_total_dist = sum(total_dist_list)/len(total_dist_list)
            ave_stop_cnt = sum(stop_cnt_list)/len(stop_cnt_list)
                            
            os.system('echo "'+source_dir+'" >> '+home_dir+'/total_dist.txt')
            os.system('echo "ave travel dist '+str(round(ave_total_dist,3))+'m" >> '+home_dir+'/total_dist.txt')
            os.system('echo "------------------------------------------------------------" >> '+home_dir+'/total_dist.txt')

            os.system('echo "'+source_dir+'" >> '+home_dir+'/stop_cnt.txt')
            os.system('echo "ave stop count '+str(round(ave_stop_cnt,3))+'" >> '+home_dir+'/stop_cnt.txt')
            os.system('echo "------------------------------------------------------------" >> '+home_dir+'/stop_cnt.txt')