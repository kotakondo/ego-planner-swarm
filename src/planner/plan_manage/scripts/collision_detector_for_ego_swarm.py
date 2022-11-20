#!/usr/bin/env python

#Author: Kota Kondo
#Date: August 13, 2022

#you can get transformation btwn two agents and if they are too close (violating bbox) then it prints out warning
#the reason why we use tf instead of snapstack_msgs/State is two agents publish their states asynchrnonously and therefore comparing
#these states (with slightly different timestamp) is not accurate position comparison. Whereas tf always compares two states with the same time stamp

import math
import os
import sys
import time
import rospy
import rosgraph

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
# from snapstack_msgs.msg import State
from traj_utils.msg import Collision

import numpy as np
from random import *
import tf2_ros
from numpy import linalg as LA

class CollisionDetector:

    def __init__(self):

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        rospy.sleep(3) #Important, if not it won't work

        # numerical tolerance
        # state is not synchronized and time difference could be up to 0.01[s] so we need tolerance
        # if max vel is 2.0m/s -> there should be 0.04m tolerance
        self.tol = 0.02 #[m] 

        # bbox size
        self.bbox_x = rospy.get_param('~bbox_x', 0.25) - self.tol #default value is 0.15
        self.bbox_y = rospy.get_param('~bbox_y', 0.25) - self.tol #default value is 0.15
        self.bbox_z = rospy.get_param('~bbox_z', 0.25) - self.tol #default value is 0.15
        self.num_of_agents = rospy.get_param('~num_of_agents', 10)

        self.initialized = False
        self.initialized_mat = [False for i in range(self.num_of_agents)]

        self.state_pos = np.empty([self.num_of_agents,3])

        # publisher init
        self.collision=Collision()
        self.pubIsCollided = rospy.Publisher('is_collided', Collision, queue_size=1, latch=True)

        # 

    # collision detection
    def collisionDetect(self, timer):
        
        if not self.initialized:
            initialized = True
            for k in range(self.num_of_agents):
                if not self.initialized_mat[k]:
                    initialized = False
                    break
            self.initialized = initialized

        if self.initialized:
            for i in range(1,self.num_of_agents):
                for j in range(i+1,self.num_of_agents+1):

                    if i<=9:
                        agent1 = "SQ0" + str(i) + "s" 
                    else:
                        agent1 = "SQ" + str(i) + "s" 

                    if j<=9:
                        agent2 = "SQ0" + str(j) + "s" 
                    else:
                        agent2 = "SQ" + str(j) + "s" 
                    
                    # trans = self.get_transformation(agent1, agent2)
                    # if trans is not None:

                    # print("here")

                    if (abs(self.state_pos[i-1,0] - self.state_pos[j-1,0]) < self.bbox_x 
                        and abs(self.state_pos[i-1,1] - self.state_pos[j-1,1]) < self.bbox_y 
                        and abs(self.state_pos[i-1,2] - self.state_pos[j-1,2]) < self.bbox_z):

                    # if (abs(trans.transform.translation.x) < self.bbox_x
                    #     and abs(trans.transform.translation.y) < self.bbox_y
                    #     and abs(trans.transform.translation.z) < self.bbox_z):
                        
                        print("collision btwn " + agent1 + " and " + agent2)
                        # print("agent" + str(i+1) + " and " + str(j+1) + " collide")

                        x_diff = abs(self.state_pos[i-1,0] - self.state_pos[j-1,0])
                        y_diff = abs(self.state_pos[i-1,1] - self.state_pos[j-1,1])
                        z_diff = abs(self.state_pos[i-1,2] - self.state_pos[j-1,2])

                        # print("difference in x is " + str(x_diff))
                        # print("difference in y is " + str(y_diff))
                        # print("difference in z is " + str(z_diff))

                        # max_dist = max(abs(trans.transform.translation.x), abs(trans.transform.translation.y), abs(trans.transform.translation.z))
                        max_dist = max(x_diff, y_diff, z_diff)

                        self.collision.is_collided = True
                        self.collision.agent1 = agent1
                        self.collision.agent2 = agent2

                        print("violation dist is " + str(max_dist))
                        print("\n")

                        self.collision.dist = max_dist 
                        self.pubIsCollided.publish(self.collision)

                    #     # print(str(agent1) + " and " + str(agent2) + ": " + str(trans.transform.translation.x))
                    
                    #     if (abs(trans.transform.translation.x) < self.bbox_x
                    #         and abs(trans.transform.translation.y) < self.bbox_y
                    #         and abs(trans.transform.translation.z) < self.bbox_z):
                            
                    #         self.collision.is_collided = True
                    #         self.collision.agent1 = trans.header.frame_id
                    #         self.collision.agent2 = trans.child_frame_id

                    #         print("collision btwn " + trans.header.frame_id + " and " + trans.child_frame_id)

                    #         max_dist = max(abs(trans.transform.translation.x), abs(trans.transform.translation.y), abs(trans.transform.translation.z))

                    #         print("violation dist is " + str(max_dist))

                    #         self.collision.dist = max_dist 
                    #         self.pubIsCollided.publish(self.collision)


    def SQ00stateCB(self, data):
        self.state_pos[0,0:3] = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
        # print(LA.norm(self.state_pos))
        # print(self.state_pos[0,:])
        if self.initialized_mat[0] == False and LA.norm(self.state_pos[0,0:3]) > 0.1: # make sure first [0, 0, 0] state info will not be used
            self.initialized_mat[0] = True
    def SQ01stateCB(self, data):
        self.state_pos[1,0:3] = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
        if self.initialized_mat[1] == False and LA.norm(self.state_pos[1,0:3]) > 0.1: # make sure first [0, 0, 0] state info will not be used
            self.initialized_mat[1] = True
    def SQ02stateCB(self, data):
        self.state_pos[2,0:3] = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
        if self.initialized_mat[2] == False and LA.norm(self.state_pos[2,0:3]) > 0.1: # make sure first [0, 0, 0] state info will not be used
            self.initialized_mat[2] = True
    def SQ03stateCB(self, data):
        self.state_pos[3,0:3] = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
        if self.initialized_mat[3] == False and LA.norm(self.state_pos[3,0:3]) > 0.1: # make sure first [0, 0, 0] state info will not be used
            self.initialized_mat[3] = True
    def SQ04stateCB(self, data):
        self.state_pos[4,0:3] = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
        if self.initialized_mat[4] == False and LA.norm(self.state_pos[4,0:3]) > 0.1: # make sure first [0, 0, 0] state info will not be used
            self.initialized_mat[4] = True
    def SQ05stateCB(self, data):
        self.state_pos[5,0:3] = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
        if self.initialized_mat[5] == False and LA.norm(self.state_pos[5,0:3]) > 0.1: # make sure first [0, 0, 0] state info will not be used
            self.initialized_mat[5] = True
    def SQ06stateCB(self, data):
        self.state_pos[6,0:3] = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
        if self.initialized_mat[6] == False and LA.norm(self.state_pos[6,0:3]) > 0.1: # make sure first [0, 0, 0] state info will not be used
            self.initialized_mat[6] = True
    def SQ07stateCB(self, data):
        self.state_pos[7,0:3] = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
        if self.initialized_mat[7] == False and LA.norm(self.state_pos[7,0:3]) > 0.1: # make sure first [0, 0, 0] state info will not be used
            self.initialized_mat[7] = True
    def SQ08stateCB(self, data):
        self.state_pos[8,0:3] = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
        if self.initialized_mat[8] == False and LA.norm(self.state_pos[8,0:3]) > 0.1: # make sure first [0, 0, 0] state info will not be used
            self.initialized_mat[8] = True
    def SQ09stateCB(self, data):
        self.state_pos[9,0:3] = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
        if self.initialized_mat[9] == False and LA.norm(self.state_pos[9,0:3]) > 0.1: # make sure first [0, 0, 0] state info will not be used
            self.initialized_mat[9] = True
    # def SQ10stateCB(self, data):
    #     self.state_pos[10,0:3] = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
    #     if self.initialized_mat[10] == False and LA.norm(self.state_pos[10,0:3]) > 0.1: # make sure first [0, 0, 0] state info will not be used
    #         self.initialized_mat[10] = True
    # def SQ11stateCB(self, data):
    #     self.state_pos[11,0:3] = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
    #     if self.initialized_mat[11] == False and LA.norm(self.state_pos[11,0:3]) > 0.1: # make sure first [0, 0, 0] state info will not be used
    #         self.initialized_mat[11] = True
    # def SQ12stateCB(self, data):
    #     self.state_pos[12,0:3] = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
    #     if self.initialized_mat[12] == False and LA.norm(self.state_pos[12,0:3]) > 0.1: # make sure first [0, 0, 0] state info will not be used
    #         self.initialized_mat[12] = True
    # def SQ13stateCB(self, data):
    #     self.state_pos[13,0:3] = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
    #     if self.initialized_mat[13] == False and LA.norm(self.state_pos[13,0:3]) > 0.1: # make sure first [0, 0, 0] state info will not be used
    #         self.initialized_mat[13] = True
    # def SQ14stateCB(self, data):
    #     self.state_pos[14,0:3] = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
    #     if self.initialized_mat[14] == False and LA.norm(self.state_pos[14,0:3]) > 0.1: # make sure first [0, 0, 0] state info will not be used
    #         self.initialized_mat[14] = True
    # def SQ15stateCB(self, data):
    #     self.state_pos[15,0:3] = np.array([data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z])
    #     if self.initialized_mat[15] == False and LA.norm(self.state_pos[15,0:3]) > 0.1: # make sure first [0, 0, 0] state info will not be used
    #         self.initialized_mat[15] = True

    def get_transformation(self, source_frame, target_frame):

        # get the tf at first available time
        try:
            transformation = self.tfBuffer.lookup_transform(source_frame, target_frame, rospy.Time(0), rospy.Duration(0.01))
            return transformation
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
            # rospy.logerr("Unable to find the transformation")

def startNode():
    c = CollisionDetector()
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
    # rospy.Subscriber("/drone_10_visual_slam/odom", Odometry, c.SQ10stateCB)
    # rospy.Subscriber("/drone_11_visual_slam/odom", Odometry, c.SQ11stateCB)
    # rospy.Subscriber("/drone_12_visual_slam/odom", Odometry, c.SQ12stateCB)
    # rospy.Subscriber("/drone_13_visual_slam/odom", Odometry, c.SQ13stateCB)
    # rospy.Subscriber("/drone_14_visual_slam/odom", Odometry, c.SQ14stateCB)
    # rospy.Subscriber("/drone_15_visual_slam/odom", Odometry, c.SQ15stateCB)

    rospy.Timer(rospy.Duration(0.001), c.collisionDetect)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('CollisionDetector')
    startNode()