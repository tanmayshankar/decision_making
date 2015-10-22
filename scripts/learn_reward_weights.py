#!/usr/bin/env python

import numpy as npy
from scipy.stats import truncnorm
import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import String
import roslib
from nav_msgs.msg import Odometry
import sys
# from mpl_toolkits.mplot3d import Axes3D
# import matplotlib.pyplot as plt

length=100
width=100

value_function = npy.zeros(shape=(length,width))
cost_map = npy.zeros(shape=(length,width))
reward = npy.zeros(shape=(length,width))

lamda = 0.1

def learn_weight(state_from, action, state_to):
	# State must be a collection of x and y indices. Action must be a choice from the possible discretizations. 
	reward[state_from[0]][state_from[1]] = 

