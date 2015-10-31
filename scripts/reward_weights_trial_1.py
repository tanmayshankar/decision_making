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
value_functions = npy.zeros(shape=(number_value_functions,length,width))
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



######THIS IS ONLY VALID FOR SINGLE STEP LOOKAHEAD 


def learn_lamda(state_from, state_to, action):
	# State must be a collection of x and y indices. Action must be a choice from the possible discretizations. 
	max_reward_value = 0
	reward_value=0
	lamda_mod =0
	for lamda_try in lamda_space:
		# reward[state_from[0]][state_from[1]] = lamda*value_function[state_from[0]][state_from[1]] + (1-lamda)*cost_map[state_from[0]][state_from[1]]
		reward_value = lamda*value_function[state_from[0]][state_from[1]] + (1-lamda)*cost_map[state_from[0]][state_from[1]]
		if reward_value > max_reward_value:	
			max_reward_value=reward_value	
			lamda_mod = lamda_try


	lamda = (1-alpha)*lamda + alpha*lamda_mod

def update_weights(state_from, state_to, action):
	
	mod_weights=npy.zeros(basis_size)
	temp_weights=npy.zeros(basis_size)

	#Iterate over all weights. 
	for i in range(0,basis_size):
		max_reward_value=0
		reward_value=0	

		#Iterate over all possible values of the weights (indepedent maximization)
		for var_weight in weight_space:
		# for j in range(0,weight_space_size):
			temp_weights=reward_weights
			temp_weights[i]=var_weight
			reward_value = npy.dot(reward_weights, basis_functions[:][state_from[0]][state_from[1]])
			if reward_value>max_reward_value:
				max_reward_value=reward_value
				mod_weights[i]=var_weight

	for i in range(0,basis_size):
		reward_weights[i] = reward_weights[i]*(1-alpha) + mod_weights[i]*alpha

def learn_weights(state_from, state_to, action):

	for i in range(0,number_trajectories):
		for j in range(0,trajectory_length):
			update_weights(from_states[i][j][:],to_states[i][j][:],action_list[i][j])













