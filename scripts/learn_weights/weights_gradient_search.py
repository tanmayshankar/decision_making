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

number_objects=41
number_value_functions = number_objects
number_costmaps=3
basis_size= number_value_functions+number_costmaps

#Defining basis functions for the reward. Each of these basis functions is defined over a 2D space. 
#Two types of basis functions - the value function for a particular object, and the costmaps at different heights. 


# value_functions = npy.zeros(shape=(number_value_functions,length,width))

value_functions = npy.loadtxt(str(sys.argv[1]))
# Note that this returned a 2D array!
# print pairwise_value_function.shape
value_functions = value_functions.reshape((number_objects,number_objects,discrete_size))

cost_maps = npy.zeros(shape=(number_costmaps,length,width))

#Defining collective basis functions. 
basis_functions = npy.zeros(shape=(basis_size, length, width))
for i in range(0,number_value_functions):
	basis_functions[i][:][:] = value_functions[i][:][:]
for i in range(0,number_costmaps):
	basis_functions[i+number_value_functions][:][:] = cost_maps[i][:][:]

# #Defining the weights of these value functions. 
# value_weights = npy.zeros(number_value_functions)
# costmap_weights = npy.zeros(number_costmaps)

#Defining collective weights. 
reward_weights = npy.zeros(basis_size)

weight_space_size=10
weight_space = npy.linspace(0,1,weight_space_size)
# lamda = 0.1
#Values of lamda. 

# lamda_space_size=10


# Define learning rate alpha. 
alpha =0.3

#Define number of trajectories. 
number_trajectories = 1 

#Define length of each trajectory. 
trajectory_length = 10

#Defining set of demo trajectories. 
from_states = npy.zeros(shape=(number_trajectories,trajectory_length,2))
to_states = npy.zeros(shape=(number_trajectories,trajectory_length,2))
action_list = npy.zeros(shape=(number_trajectories,trajectory_length))

# LEARNING WEIGHTS FROM DEMONSTRATION / EXPERT ACTIONS: 
weight_h = 0.05

upper_w_reward = 0.0
lower_w_reward = 0.0
# mid_w_reward = 0.0 
reward_derivative = 0.0

prev_reward_value = 0.0
cur_reward_value =0.0 

epsilon = 0.005

buffer_size = 10
convergence_test = npy.zeros(buffer_size)



def calculate_expected_reward_increase(trajectory_index,trajectory_length,calc_weights):
	expected_increase = 0	
	for t in range(0,trajectory_length):
		state_from = from_states[trajectory_index][t]
		state_to = to_states[trajectory_index][t]

		from_reward_value = npy.dot(calc_weights, basis_functions[:][state_from[0]][state_from[1]])
		to_reward_value = npy.dot(calc_weights, basis_functions[:][state_to[0]][state_to[1]])
		reward_increase = to_reward_value - from_reward_value
		expected_increase += reward_increase
	return expected_increase


def update_weights(trajectory_index,trajectory_length):
	
	#Initializing the weights for search.
	temp_weights = reward_weights
	mod_weights = reward_weights
	alpha_1 = 0.2

	while convergence_test.prod()=0:		

		for j in range(0,basis_size):
			mod_weights[j] = temp_weights[j] + weight_h
			upper_w_reward = calculate_expected_reward_increase(trajectory_index,trajectory_length,mod_weights)
			mod_weights[j] = temp_weights[j] - weight_h
			lower_w_reward = calculate_expected_reward_increase(trajectory_index,trajectory_length,mod_weights)

			reward_derivative = upper_w_reward - lower_w_reward
			reward_derivative /= weight_h

			temp_weights[j] = temp_weights[j] - alpha_1 * reward_derivative

		temp_weights = temp_weights[:]/temp_weights.sum()

		prev_reward_value = cur_reward_value
		cur_reward_value = calculate_expected_reward_increase(trajectory_index,trajectory_length,mod_weights)

		if ((cur_reward_value - prev_reward_value)<epsilon):
			convergence_test = npy.roll(convergence_test,-1)
			convergence_test[buffer_size-1]

	return temp_weights


def learn_weights(state_from, state_to, action):
	
	reward_weights = npy.ones(basis_size)/basis_size
	up_mod_weights = reward_weights
	for trajectory_index in range(0,number_trajectories):
		up_mod_weights = update_weights(trajectory_index,trajectory_lengths[trajectory_index],reward_weights)	
		reward_weights[:] = reward_weights[:] + alpha*up_mod_weights[:]
		# alpha = ((number_trajectories-1)/number_trajectories)*alpha
		alpha = 0.9 * alpha








