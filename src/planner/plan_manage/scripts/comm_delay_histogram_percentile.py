#!/usr/bin/env python  
# Kota Kondo

#  Commands: Open three terminals and execute this:
#  python comm_delay_histogram_percentile.py
#  Ex. python comm_delay_histogram_percentile.py

# change cd and dc

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
import sys
import scipy
import numpy

if __name__ == '__main__':

    num_of_agents = 10

    cd_list = [0, 50, 100, 200, 300]

    for cd in cd_list:

        figname = 'cd'+str(cd)+'_comm_delay_histogram.png'
        source_dir = "/home/kota/ego_swarm_data/bags" # change the source dir accordingly #10 agents 
        source_bags = source_dir + "/cd"+str(cd)+"ms/*.bag" # change the source dir accordingly #10 agents

        rosbag_list = glob.glob(source_bags)
        rosbag_list.sort() #alphabetically order
        rosbag = []
        comm_delay = []

        for bag in rosbag_list:
            rosbag.append(bag)

        # print(rosbag)

        for i in range(len(rosbag)):
        # for i in range(10):

            b = bagreader(rosbag[i], verbose=False);
            
            for i in range(num_of_agents):
                log_data = b.message_by_topic("/drone_"+str(i)+"_ego_planner_node/comm_delay")
                try:
                    log = pd.read_csv(log_data)

                    for j in range(len(log.comm_delay)):
                        comm_delay.append(log.comm_delay[j])
                except:
                    pass

        # print percentile

        comm_delay_arr = numpy.array(comm_delay)
        os.system('echo "----------------------------------------------------------------------------------" >> /home/kota/ego_swarm_data/comm_delay_percentile.txt')
        os.system('echo "'+source_bags+'" >> /home/kota/ego_swarm_data/comm_delay_percentile.txt')

        # print(comm_delay)

        
        max_comm_delay = max(comm_delay)

        fig = plt.figure()
        ax = fig.add_subplot()
        n, bins, patches = plt.hist(x=comm_delay, color="blue", edgecolor = 'black')
        # plt.axvline(x=dc/1000, color="red")
        if cd == 50:
            ax.set_xticks(np.arange(0,0.125,0.025))
            ax.set_xticklabels(np.arange(0,125,25))
        elif cd == 100:
            ax.set_xticks(np.arange(0,0.175,0.025))
            ax.set_xticklabels(np.arange(0,175,25))
        elif cd == 200:
            ax.set_xticks(np.arange(0,0.250,0.025))
            ax.set_xticklabels(np.arange(0,250,25))
        elif cd == 500:
            ax.set_xticks(np.arange(0,0.375,0.025))
            ax.set_xticklabels(np.arange(0,375,25))
        # plt.rcParams["font.family"] = "Times New Roman"
        plt.grid(axis='y', color='black', alpha=0.2)
        plt.title('Comm delay histogram \n max comm_delay is '+str(round(max_comm_delay*1000))+' [ms]')
        plt.xlabel("comm delay [ms]")
        plt.ylabel("count")
        plt.savefig('/home/kota/ego_swarm_data/'+figname)
        plt.close('all')
        # plt.show()
        # except:
            # pass

        # in case you wanna calculate the value of q-th percentile
        # print("----------------------------------------------------------------------------------")
        for q in range(100,0,-25):
            try:
                os.system('echo "'+str(q)+'-th : '+ str(round(numpy.percentile(comm_delay_arr, q)*1000,2)) + 'ms" >> /home/kota/ego_swarm_data/comm_delay_percentile.txt')
            except:
                pass