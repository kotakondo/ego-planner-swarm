#!/usr/bin/env python
# coding=utf-8

# /* ----------------------------------------------------------------------------
#  * Copyright 2022, Kota Kondo, Aerospace Controls Laboratory
#  * Massachusetts Institute of Technology
#  * All Rights Reserved
#  * Authors: Kota Kondo, et al.
#  * See LICENSE file for the license information
#  * -------------------------------------------------------------------------- */

import math
import os
import sys
import time
import rospy
# from snapstack_msgs.msg import State
import subprocess
import numpy as np
import time
from numpy import linalg as LA
import struct
from traj_utils.msg import GoalReached

def checkGoalReached(num_of_agents):
    try:
        is_goal_reached = subprocess.check_output(['rostopic', 'echo', '/goal_reached', '-n', '1'], timeout=2).decode()
        print("True")
        return True 
    except:
        print("False")
        return False        

def myhook():
  print("shutdown time!")

if __name__ == '__main__':

    # parameters
    num_of_sims=1
    num_of_agents=10
    how_long_to_wait = 60 #[s]
    cd_list = [0, 50, 100, 200, 300]
        
    # folder initialization
    folder_bags_list = []
    folder_txts_list = []

    for cd in cd_list:
        cd_in_s = cd/1000;
        folder_bags="/home/data/ego_swarm/bags/cd"+str(cd)+"ms"
        # folder_csv="/home/data/ego_swarm_data/csv/cd"+str(cd)+"ms"

        # create directy if not exists
        if (not os.path.exists(folder_bags)):
            os.makedirs(folder_bags)

         # create directy if not exists
        # if (not os.path.exists(folder_csv)):
        #     os.makedirs(folder_csv)        

        # name_node_record="bag_recorder"
        kill_all="tmux kill-server & killall -9 gazebo & killall -9 gzserver  & killall -9 gzclient & killall -9 roscore & killall -9 rosmaster & pkill mader_node & pkill -f dynamic_obstacles & pkill -f rosout & pkill -f behavior_selector_node & pkill -f rviz & pkill -f rqt_gui & pkill -f perfect_tracker & pkill -f mader_commands"

        #make sure ROS (and related stuff) is not running
        os.system(kill_all)

        for k in range(num_of_sims):

            if k <= 9:
                sim_id = "0"+str(k)
            else:
                sim_id = str(k)

            commands = []
            name_node_record="bag_recorder"
            commands.append("roscore")

            for num in range(num_of_agents):
                agent_id = str(num)
                # to set the commdelay param, i commented out <param name="fsm/commdelay" value="0.0" type="double"/> in advanced_param.xml
                commands.append("sleep 2.0 && rosparam set /drone_"+agent_id+"_ego_planner_node/fsm/commdelay "+str(cd_in_s))

            commands.append("sleep 2.0 && cd "+folder_bags+" && rosbag record -a -o ego_swarm_sim_" + sim_id + " __name:="+name_node_record)
            commands.append("sleep 2.0 && roslaunch ego_planner collision_detector.launch num_of_agents:=" + str(num_of_agents))
            # commands.append("sleep 2.0 && roslaunch ego_planner ave_distance.launch num_of_agents:="+str(num_of_agents)+" folder_loc:="+folder_csv+" sim:="+sim_id)
            commands.append("sleep 2.0 && roslaunch ego_planner goal_reached.launch") #we are calculating completion time here so sleep time needs to be the same as send_goal

            commands.append("sleep 5.0 && roslaunch ego_planner multi_run.launch")
            #publishing the goal should be the last command
            commands.append("sleep 5.0 && tmux detach")

            # print("len(commands)= " , len(commands))
            session_name="run_many_sims_multi_agent_session"
            os.system("tmux kill-session -t" + session_name)
            os.system("tmux new-session -d -s "+str(session_name)+" -x 300 -y 300")

            # tmux splitting
            for i in range(len(commands)):
                # print('splitting ',i)
                os.system('tmux new-window -t ' + str(session_name))
           
            time.sleep(3.0)

            for i in range(len(commands)):
                os.system('tmux send-keys -t '+str(session_name)+':'+str(i) +'.0 "'+ commands[i]+'" '+' C-m')

            os.system("tmux attach")
            print("Commands sent")

            # rospy.init_node('goalReachedCheck_in_sims', anonymous=True)
            # c = GoalReachedCheck_sim()
            # rospy.Subscriber("goal_reached", GoalReached, c.goal_reachedCB)
            # rospy.on_shutdown(myhook)

            # check if all the agents reached the goal
            is_goal_reached = False
            tic = time.perf_counter()
            toc = time.perf_counter()
            os.system('tmux new-window -t ' + str(session_name))

            while (toc - tic < how_long_to_wait and not is_goal_reached):
                toc = time.perf_counter()
                if(checkGoalReached(num_of_agents)):
                    print('all the agents reached the goal')
                    time.sleep(2) # gives us time to write csv file for ave distance
                    is_goal_reached = True
                time.sleep(0.1)

            if (not is_goal_reached):
                os.system('echo "simulation '+sim_id+': not goal reached" >> '+folder_bags+'/status.txt')
            else:
                os.system('echo "simulation '+sim_id+': goal reached" >> '+folder_bags+'/status.txt')

            os.system("rosnode kill "+name_node_record);
            # os.system("rosnode kill goalReachedCheck_in_sims")
            # os.system("rosnode kill -a")
            time.sleep(1.0)
            os.system(kill_all)
            time.sleep(1.0)

        time.sleep(5.0)


# After the simulations
session_name="data"
os.system("tmux kill-session -t" + session_name)
os.system("tmux new-session -d -s "+str(session_name)+" -x 300 -y 300")

commands = []
commands.append("sleep 3.0 && roscd ego_planner && cd scripts && python collision_check.py")
commands.append("sleep 3.0 && roscd ego_planner && cd scripts && python completion_time.py")
commands.append("sleep 3.0 && roscd ego_planner && cd scripts && python comm_delay_histogram_percentile.py")
# commands.append("sleep 3.0 && roscd ego_planner && cd scripts && python ave_distance_csv2txt.py")
# commands.append("sleep 3.0 && roscd ego_planner && cd scripts && python total_dist_and_stoppage_ego_swarm.py")
# commands.append("sleep 3.0 && roscd ego_planner && cd scripts && python detect_who_died_ego_swarm.py")

# tmux splitting
for i in range(len(commands)):
    # print('splitting ',i)
    os.system('tmux new-window -t ' + str(session_name))

time.sleep(3.0)

for i in range(len(commands)):
    os.system('tmux send-keys -t '+str(session_name)+':'+str(i) +'.0 "'+ commands[i]+'" '+' C-m')

print("Commands sent")
