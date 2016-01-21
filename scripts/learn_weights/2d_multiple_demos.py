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
from scipy.stats import rankdata
from matplotlib.pyplot import *
# from mpl_toolkits.mplot3d import Axes3D
# import matplotlib.pyplot as plt

basis_size = 5  
discrete_size = 100

basis_functions = npy.zeros(shape=(basis_size,discrete_size,discrete_size))
weights = npy.ones(basis_size)/basis_size

max_val=0
for i in range(0,discrete_size):
	for j in range(0,discrete_size):
		basis_functions[0][i][j] = i-j
		basis_functions[1][i][j] = -j
		basis_functions[2][i][j] = i+j
		basis_functions[3][i][j] = -i-j
		basis_functions[4][i][j] = j


for i in range(0,discrete_size):
	for j in range(0,discrete_size):
		for k in range(0,basis_size):
			if (basis_functions[k][i][j]>max_val):
				max_val=basis_functions[k][i][j]

for i in range(0,discrete_size):
	for j in range(0,discrete_size):
		for k in range(0,basis_size):
			basis_functions[k][i][j] /= max_val
		# basis_functions[2][i][j] += 1
		# basis_functions[3][i][j] += 1

weight_space_size=10
weight_space = npy.linspace(0,1,weight_space_size)

# Define learning rate alpha. 
alpha =0.2

#Define number of trajectories. 
# number_trajectories = 1 

#Define length of each trajectory. 
# trajectory_length = 10
number_trajectories = 1
max_trajectory_length = 20

#Defining set of demo trajectories. 
# from_states = npy.zeros(shape=(number_trajectories,max_trajectory_length,2))
# to_states = npy.zeros(shape=(number_trajectories,max_trajectory_length,2))
trajectory_lengths = npy.zeros(number_trajectories)
trajectory_lengths = trajectory_lengths.astype(int)
trajectories = npy.zeros(shape=(number_trajectories,max_trajectory_length,2))
trajectories = trajectories.astype(int)
# action_list = npy.zeros(shape=(number_trajectories,trajectory_length))

# LEARNING WEIGHTS FROM DEMONSTRATION / EXPERT ACTIONS: 
weight_h = 0.01

upper_w_reward = 0.0
lower_w_reward = 0.0
# mid_w_reward = 0.0 
reward_derivative = 0.0

buffer_size = 10
# trajectory_length = 20

trajectories = npy.loadtxt(str(sys.argv[1]))
print trajectories

def define_trajectory(trajectory_index):
	
	trajectory_lengths[trajectory_index] = 20
	trajectories[trajectory_index,0,0] = 26
	trajectories[trajectory_index,0,1] = 28
	for t in range(1,trajectory_lengths[trajectory_index]):
		trajectories[trajectory_index,t,0] = trajectories[trajectory_index,t-1,0] - (t%2)
		trajectories[trajectory_index,t,1] = trajectories[trajectory_index,t-1,1] - 1
	print trajectories

def calculate_expected_reward_increase(trajectory_index,calc_weights):
# def calculate_expected_reward_increase(calc_weights):
 	
 	expected_increase = 0	 	
 	reward_increase =0 
 	gamma = 0.95
	# from_state = from_states[trajectory_index,0,:]
	# to_state = _states[trajectory_index,0,:]
	from_state = trajectories[trajectory_index,0,:]
	to_state = trajectories[trajectory_index,1,:]

 	for t in range(1,trajectory_lengths[trajectory_index]-1):

		from_reward_value = npy.dot(calc_weights,basis_functions[:,from_state[0],from_state[1]])
		to_reward_value = npy.dot(calc_weights,basis_functions[:,to_state[0],to_state[1]])
		reward_increase = to_reward_value - from_reward_value
 	 	expected_increase += (gamma**t)*reward_increase

 	 	from_state = to_state
 	 	to_state = trajectories[trajectory_index,t+1]

 	return expected_increase

def update_weights(trajectory_index, weights):
# def update_weights():	
	
	#Initializing the weights for search.
	convergence_test = npy.zeros(buffer_size)
	temp_weights = weights
	mod_weights = weights
	trial_weights = weights
	alpha_1 = 0.01
	number_iterations=0
	epsilon = 0.00001
	prev_reward_value = 0.0
	cur_reward_value =0.0
	diff = 0. 
	max_iter=4000

	while ((convergence_test.prod()==0)and(number_iterations<max_iter)):		
		
		for j in range(0,basis_size):
			diff = 0.
			if (trial_weights[j]<(1.0-weight_h)):
				mod_weights[j] = trial_weights[j] + weight_h
				diff+=weight_h
			upper_w_reward = calculate_expected_reward_increase(trajectory_index,mod_weights)
			# upper_w_reward = calculate_expected_reward_increase(mod_weights)

			if (trial_weights[j]>weight_h):
				mod_weights[j] = trial_weights[j] - weight_h
				diff+=weight_h
			lower_w_reward = calculate_expected_reward_increase(trajectory_index,mod_weights)
			# lower_w_reward = calculate_expected_reward_increase(mod_weights)

			reward_derivative = upper_w_reward - lower_w_reward			
			reward_derivative /= diff

			temp_weights[j] = temp_weights[j] + alpha_1 * reward_derivative #Should it be + alpha....

		temp_weights = temp_weights[:]/temp_weights.sum()	
		trial_weights=temp_weights

		prev_reward_value = cur_reward_value
		cur_reward_value = calculate_expected_reward_increase(trajectory_index,temp_weights)

		# print prev_reward_value, cur_reward_value, ((cur_reward_value-prev_reward_value)/cur_reward_value)*100

		if (((cur_reward_value - prev_reward_value)/cur_reward_value)<epsilon):
			convergence_test = npy.roll(convergence_test,-1)
			convergence_test[buffer_size-1]=1

		number_iterations+=1
		# print number_iterations
		
	return temp_weights

# print basis_functions	
define_trajectory(0)
# print update_weights(0, weights)

reward_weights = npy.zeros(basis_size)
dummy_weights = npy.zeros(basis_size)

# for i in range(0,5):
# 	for j in range(0,basis_size):		
# 		weights[j] = random.random()	
# 	weights = weights[:]/weights.sum()
# 	print "Initial weights: ",weights
# 	# for j in range(0,basis_size):
# 	dummy_weights = update_weights(0,weights)
# 	reward_weights += dummy_weights
# 	print "Final weights: ",dummy_weights,"\n"

reward_weights = reward_weights[:]/reward_weights.sum()

print reward_weights

