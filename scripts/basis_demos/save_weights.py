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

basis_size = 3
discrete_size = 50

#####INPUTS: basis functions, weights, trajectory lengths, trajectories.

basis_functions = npy.loadtxt(str(sys.argv[1]))
basis_functions = basis_functions.reshape((basis_size,discrete_size,discrete_size))
# print basis_functions

init_weights = npy.loadtxt(str(sys.argv[2]))
print "Initial basis weights. ",init_weights

trajectory_lengths = npy.loadtxt(str(sys.argv[3]))
trajectory_lengths=trajectory_lengths.astype(int)
# print trajectory_lengths

raw_trajectories = npy.loadtxt(str(sys.argv[4]))
# print raw_trajectories
trajectories = [[[0,0],[1,2],[3,4]]]
asum=0

def store_demonstrations():
	asum=0
	for i in range(0,len(trajectory_lengths)):
		current_demo = [[0,0]]		
		for j in range(0,trajectory_lengths[i]):			
			current_demo.append([raw_trajectories[asum+j,0],raw_trajectories[asum+j,1]])
		current_demo.remove(current_demo[0])		
		trajectories.append(current_demo)	
		asum += trajectory_lengths[i]
	trajectories.remove(trajectories[0])

store_demonstrations()
# for i in range(len(trajectory_lengths)):
# 	print trajectories[i],'\n'

max_val=0
weight_space_size=10
weight_space = npy.linspace(0,1,weight_space_size)

# Define learning rate alpha. 
alpha =0.2

# LEARNING WEIGHTS FROM DEMONSTRATION / EXPERT ACTIONS: 
weight_h = 0.01

upper_w_reward = 0.0
lower_w_reward = 0.0
# mid_w_reward = 0.0 
reward_derivative = 0.0

buffer_size = 10

def calculate_expected_reward_increase(trajectory_index,calc_weights):
# def calculate_expected_reward_increase(calc_weights):
 	
 	expected_increase = 0	 	
 	reward_increase =0 
 	gamma = 0.95
	# from_state = from_states[trajectory_index,0,:]
	# to_state = _states[trajectory_index,0,:]
	from_state = trajectories[trajectory_index][0][:]
	to_state = trajectories[trajectory_index][1][:]

 	for t in range(1,trajectory_lengths[trajectory_index]-1):

		from_reward_value = npy.dot(calc_weights,basis_functions[:,from_state[0],from_state[1]])
		to_reward_value = npy.dot(calc_weights,basis_functions[:,to_state[0],to_state[1]])
		reward_increase = to_reward_value - from_reward_value
 	 	expected_increase += (gamma**t)*reward_increase

 	 	from_state = to_state
 	 	to_state = trajectories[trajectory_index][t+1][:]

 	return expected_increase

def update_weights(trajectory_index, weights):
# def update_weights():	
	
	#Initializing the weights for search.
	convergence_test = npy.zeros(buffer_size)
	temp_weights = weights
	mod_weights = weights
	trial_weights = weights
	alpha_1 = 0.1
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

		# if (((cur_reward_value - prev_reward_value)/cur_reward_value)<epsilon):
		# 	convergence_test = npy.roll(convergence_test,-1)
		# 	convergence_test[buffer_size-1]=1

		number_iterations+=1
		# print number_iterations
		
	return temp_weights

reward_weights = npy.zeros(basis_size)
dummy_weights = npy.zeros(basis_size)
fake_weights = npy.zeros(basis_size)
repeat_weights = npy.zeros(basis_size)

#Repeating with random initializations of weights for same demo. 
number_repeats=5

for i in range(0,len(trajectory_lengths)):
	for k in range(0,number_repeats):
		for j in range(0,basis_size):
			fake_weights[j] = random.random()
			fake_weights = fake_weights[:]/fake_weights.sum()
		print "Starting weights on demonstration:",i," iteration: ",k,":",fake_weights
		dummy_weights=update_weights(i,fake_weights)
		print "Ending weights on demonstration:",i," iteration: ",k,":",dummy_weights
		repeat_weights+=dummy_weights
	repeat_weights=repeat_weights[:]/repeat_weights.sum()
	print "Weights for demonstration:",i,":",repeat_weights
	reward_weights+=repeat_weights
reward_weights = reward_weights[:]/reward_weights.sum()

print "Final reward weights:",reward_weights

with file('reward_weights.txt','w') as outfile: 
	outfile.write('#Reward weights.\n')
	npy.savetxt(outfile,reward_weights,fmt='%-7.2f')
