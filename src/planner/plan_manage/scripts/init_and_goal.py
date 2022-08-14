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
from random import *
# import numpy as np
# from pyquaternion import Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion


if __name__ == '__main__':

    # formation="sphere", "square" "circle"
    formation="circle"

    # number of agents
    num_of_agents=10; 

    # radius of the circle
    radius=15;

    if(formation=="sphere"):
        # num_mer=int(math.sqrt(num_of_agents)); #Num of meridians
        # num_of_agents_per_mer=int(math.sqrt(num_of_agents));    #Num of agents per meridian
        if(num_of_agents%3==0):
            num_mer=max(int(num_of_agents/4.0),3); #Num of meridians
        else: #either divisible by 4 or will force it by changing num of agents
            num_mer=max(int(num_of_agents/4.0),4); #Num of meridians
        num_of_agents_per_mer=int(num_of_agents/num_mer);    #Num of agents per meridian

    if(formation=="circle" or formation=="square"):
        num_mer=num_of_agents
        num_of_agents_per_mer=1

    id_number=0;
    shift_z=radius;
    shift_z=1.0 # should always above 0. look at terminal goal CB in mader_ros.cpp

    #TODO: Implement the square as well for other number_of_agents
    square_starts=[[4.0, 0.0, 1.0], 
                    [4.0, 4.0, 1.0], 
                    [0.0, 4.0, 1.0], 
                    [-4.0, 4.0, 1.0],
                    [-4.0, 0.0, 1.0],
                    [-4.0, -4.0, 1.0],
                    [0.0, -4.0, 1.0],
                    [4.0, -4.0, 1.0] ]

    square_goals=  [[-4.0, 0.0, 1.0],
                    [-4.0, -4.0, 1.0],
                    [0.0, -4.0, 1.0],
                    [4.0, -4.0, 1.0],
                    [4.0, 0.0, 1.0],
                    [4.0, 4.0, 1.0],
                    [0.0, 4.0, 1.0],
                    [-4.0, 4.0, 1.0]];

    square_yaws_deg=  [-180.0, -135.0, -90.0, -45.0, 0.0, 45.0, 90.0, 135.0];

    for i in range(1, num_mer+1):
        theta=0.0+i*(2*math.pi/num_mer);
        for j in range(1, num_of_agents_per_mer+1):

            phi=(-math.pi +j*(math.pi/(num_of_agents_per_mer+1)))
            x=radius*math.cos(theta)*math.sin(phi)
            y=radius*math.sin(theta)*math.sin(phi)
            z=shift_z + radius*math.cos(phi)
            x=round(x,1)
            y=round(y,1)
            z=round(z,1)

            pitch=0.0;
            roll=0.0;
            yaw= theta#+math.pi  

            goal_x=radius*math.cos(theta+2*math.pi)*math.sin(phi+math.pi)
            goal_y=radius*math.sin(theta+2*math.pi)*math.sin(phi+math.pi)
            goal_z=shift_z + radius*math.cos(phi+math.pi)
            goal_x=round(goal_x,1)
            goal_y=round(goal_y,1)
            goal_z=round(goal_z,1)

            # quad="SQ0" + str(id_number) + "s";
            veh="SQ";
            if i <= 9:
                num="0" + str(id_number)
            else:
                num=str(id_number)
            quad=veh+num

            print(str(quad)+'\n x: '+str(x)+'\n y: '+str(y)+'\n z: '+str(z)+'\n goal_x: '+str(goal_x)+'\n goal_y: '+str(goal_y)+'\n goal_z: '+str(goal_z))

            id_number=id_number+1

            if(formation=="square"):
                x=square_starts[i-1][0];
                y=square_starts[i-1][1];
                z=square_starts[i-1][2];

                goal_x=square_goals[i-1][0];
                goal_y=square_goals[i-1][1];
                goal_z=square_goals[i-1][2];

                yaw=square_yaws_deg[i-1]*math.pi/180;
                print("yaw= ", square_yaws_deg[i-1])

