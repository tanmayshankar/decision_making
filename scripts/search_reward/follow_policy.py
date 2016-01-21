#!/usr/bin/env python
import numpy as npy
from scipy.stats import truncnorm
import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import String
import roslib
from nav_msgs.msg import Odometry
import sys
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import random
import copy
from scipy.stats import rankdata
from matplotlib.pyplot import *

max_dist = 5

discrete_space_x = 50
discrete_space_y = 50
discrete_size = 50

path_plot = npy.zeros(shape=(discrete_space_x,discrete_space_y))

max_path_length=30
current_pose = [0,0]
# ax.plot_surface(X,Y,path_plot,cmap=plt.cm.jet,cstride=1,rstride=1)
max_number_demos = 50

trajectory_lengths = npy.zeros(max_number_demos)

state_counter = 0
number_demos = 0

# trajectories = [[[0,0],[1,2],[3,4]]]

basis_size=3
basis_functions = npy.loadtxt(str(sys.argv[1]))
basis_functions = basis_functions.reshape((basis_size,discrete_size,discrete_size))

reward_weights = npy.loadtxt(str(sys.argv[2]))

optimal_policy = npy.loadtxt(str(sys.argv[3]))
optimal_policy = optimal_policy.astype(int)
reward_function = reward_weights[0]*basis_functions[0]+reward_weights[1]*basis_functions[1]+reward_weights[2]*basis_functions[2]
path_plot = copy.deepcopy(reward_function)
max_val = npy.amax(path_plot)

action_space = [[0,1],[1,0],[0,-1],[-1,0],[1,1],[1,-1],[-1,1],[-1,-1]]

def follow_policy():

	counter=0	
	
	ax = random.randrange(0,discrete_space_x)
	ay = random.randrange(0,discrete_space_y)

	current_pose[0] = ax
	current_pose[1] = ay
	next_pose=copy.deepcopy(current_pose)
	dummy='y'
	while (counter<max_path_length)and(dummy=='y'):

		path_plot[current_pose[0]][current_pose[1]]=-max_val

		next_pose[0] = current_pose[0] + action_space[optimal_policy[current_pose[0],current_pose[1]]][0]
		next_pose[1] = current_pose[1] + action_space[optimal_policy[current_pose[0],current_pose[1]]][1]

		imshow(path_plot, interpolation='nearest', origin='lower', extent=[0,10,0,10], aspect='auto')
		plt.show(block=False)
		colorbar()
		draw()
		show() 
		current_pose[0] = next_pose[0]		
		current_pose[1] = next_pose[1]		
		counter+=1

		# dummy = raw_input("Continue? ")
follow_policy()