#!/usr/bin/env python

#!/usr/bin/env python  
#Author: Kota Kondo
#Date: July 6 2022

import math
import os
import sys
import time
import rospy
import rosgraph
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from snapstack_msgs.msg import State
from traj_utils.msg import GoalReached
import numpy as np
from random import *
import tf2_ros
from numpy import linalg as LA

class GoalReachedCheck:

    def __init__(self):


        rospy.sleep(3)

        # goal radius
        self.goal_radius = 0.2

        # number of agents
        self.num_of_agents = 10

        # is initialized?
        self.initialized = False

        # starting time to calculate completion time
        now = rospy.get_rostime()
        self.starting_time = now.secs + now.nsecs / 1e9

        # state and term_goal
        self.state_pos = np.empty([self.num_of_agents,3])
        self.term_goal_pos = np.array([
            [12.1, 8.8, 1.0],
            [4.6, 14.3, 1.0],
            [-4.6, 14.3, 1.0],
            [-12.1, 8.8, 1.0],
            [-15.0, 0.0, 1.0],
            [-12.1, -8.8, 1.0],
            [-4.6, -14.3, 1.0],
            [4.6, -14.3, 1.0],
            [12.1, -8.8, 1.0],
            [15.0, 0.0, 1.0],
            ])

        # publisher init
        self.goal_reached = GoalReached()
        self.pubIsGoalReached = rospy.Publisher('goal_reached', GoalReached, queue_size=1, latch=True)

        # is goal reached
        self.is_goal_reached = False

        # keep track of which drone has already got to the goal
        self.is_goal_reached_cnt = 0
        self.is_goal_reached_mat = [False for i in range(self.num_of_agents)]
        # rospy.sleep(10) #term_goal is published after 10 seconds

    # collision detection
    def goalReachedCheck(self, timer):
        if not self.is_goal_reached and self.initialized:
            for i in range(self.num_of_agents):
                if self.is_goal_reached_mat[i] == False:
                    if (LA.norm(self.state_pos[i,:] - self.term_goal_pos[i,:]) > self.goal_radius):
                        # print(i)
                        # print(self.state_pos[i,:])
                        # print(self.term_goal_pos[i,:])
                        print(LA.norm(self.state_pos[i,:] - self.term_goal_pos[i,:]))
                        return
                    else:
                        self.is_goal_reached_mat[i] = True
                        self.is_goal_reached_cnt = self.is_goal_reached_cnt + 1
                        # print(self.is_goal_reached_cnt)
                        # print(i)
                        # print(self.state_pos[i,:])
                        # print(self.term_goal_pos[i,:])

                        if self.is_goal_reached_cnt == self.num_of_agents:
                            self.is_goal_reached = True
                            now = rospy.get_rostime()
                            self.goal_reached.completion_time = now.secs + now.nsecs / 1e9 - self.starting_time
                            self.goal_reached.is_goal_reached = True
                            self.pubIsGoalReached.publish(self.goal_reached)

    def SQ00stateCB(self, data):
        self.state_pos[0,0:3] = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
        self.initialized = True
    def SQ01stateCB(self, data):
        self.state_pos[1,0:3] = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
    def SQ02stateCB(self, data):
        self.state_pos[2,0:3] = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
    def SQ03stateCB(self, data):
        self.state_pos[3,0:3] = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
    def SQ04stateCB(self, data):
        self.state_pos[4,0:3] = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
    def SQ05stateCB(self, data):
        self.state_pos[5,0:3] = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
    def SQ06stateCB(self, data):
        self.state_pos[6,0:3] = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
    def SQ07stateCB(self, data):
        self.state_pos[7,0:3] = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
    def SQ08stateCB(self, data):
        self.state_pos[8,0:3] = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
    def SQ09stateCB(self, data):
        self.state_pos[9,0:3] = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])

def startNode():
    c = GoalReachedCheck()
    rospy.Subscriber("/drone_0_visual_slam/odom", Odometry, c.SQ00stateCB)
    rospy.Subscriber("/drone_1_visual_slam/odom", Odometry, c.SQ01stateCB)
    rospy.Subscriber("/drone_2_visual_slam/odom", Odometry, c.SQ02stateCB)
    rospy.Subscriber("/drone_3_visual_slam/odom", Odometry, c.SQ03stateCB)
    rospy.Subscriber("/drone_4_visual_slam/odom", Odometry, c.SQ04stateCB)
    rospy.Subscriber("/drone_5_visual_slam/odom", Odometry, c.SQ05stateCB)
    rospy.Subscriber("/drone_6_visual_slam/odom", Odometry, c.SQ06stateCB)
    rospy.Subscriber("/drone_7_visual_slam/odom", Odometry, c.SQ07stateCB)
    rospy.Subscriber("/drone_8_visual_slam/odom", Odometry, c.SQ08stateCB)
    rospy.Subscriber("/drone_9_visual_slam/odom", Odometry, c.SQ09stateCB)
    rospy.Timer(rospy.Duration(0.01), c.goalReachedCheck)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('goalReachedCheck')
    startNode()