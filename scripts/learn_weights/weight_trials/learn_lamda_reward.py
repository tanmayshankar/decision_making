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
#Values of lamda. 

lamda_space_size=10
lamda_space = npy.linspace(0,1,lamda_space_size)

#Define learning rate alpha. 
alpha =0.3

# def learn_weight(state_from, action, state_to):
	# State must be a collection of x and y indices. Action must be a choice from the possible discretizations. 
	# reward[state_from[0]][state_from[1]]

## LEARNING WEIGHTS FROM DEMONSTRATION / EXPERT ACTIONS: 
def learn_weight(state_from, state_to, action):
	max_reward_value = 0
	lamda_mod =0
	for lamda_try in lamda_space:
		# reward[state_from[0]][state_from[1]] = lamda*value_function[state_from[0]][state_from[1]] + (1-lamda)*cost_map[state_from[0]][state_from[1]]
		reward_value = lamda*value_function[state_from[0]][state_from[1]] + (1-lamda)*cost_map[state_from[0]][state_from[1]]
		if reward_value > max_reward_value:	
			lamda_mod = lamda_try

	lamda = (1-alpha)*lamda + alpha*lamda_mod



