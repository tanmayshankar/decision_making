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

x=npy.linspace(0,1,5)
f1=npy.linspace(0,1,5)
f2=npy.linspace(0.8,0.3,5)
f3=npy.ones(5)/3
f=npy.transpose([f1,f2,f3])
number_val=3

print f

def frange(start, stop, step):
	i = start
	while i < stop:
		yield i
		i += step

def weights():
	max_val=-100000
	# w_star=[0,0,0]
	for w1 in npy.linspace(0,1,11):
		for w2 in frange(0,1.01-w1,0.1):
			for w3 in frange(0,1.01-w1-w2,0.1):
				w=[w1,w2,w3]				
				sum_val=0
				for t in range(1,x.size):
					sum_val+=(0.95**(t-1))*(npy.dot(w,f[:][t])-npy.dot(w,f[:][t-1]))
				if (sum_val>max_val):
					max_val=sum_val
					print w

weight_h = 0.05

upper_w_reward = 0.0
lower_w_reward = 0.0
# mid_w_reward = 0.0 
reward_derivative = 0.0



epsilon = 0.005

buffer_size = 10
convergence_test = npy.zeros(buffer_size)

# def calculate_expected_reward_increase(trajectory_index,trajectory_length,calc_weights):
def calculate_expected_reward_increase(w):
	expected_increase = 0	
	for t in range(0,x.size):
		reward_increase = (0.95**(t-1))*(npy.dot(w,f[:][t])-npy.dot(w,f[:][t-1]))
		expected_increase += reward_increase
	return expected_increase

# def update_weights(trajectory_index,trajectory_length):
def update_weights(reward_weights):
	#Initializing the weights for search.

	prev_reward_value = 0.0
	cur_reward_value =0.0 

	temp_weights = reward_weights
	mod_weights = reward_weights
	alpha_1 = 0.2
	convergence_test = npy.zeros(buffer_size)
	# while convergence_test.prod()=0:	
	iterate=0

	while ((npy.prod(convergence_test)==0)and(iterate<4000)):
		print iterate
		iterate+=1
		for j in range(0,number_val):
			mod_weights[j] = temp_weights[j] + weight_h
			upper_w_reward = calculate_expected_reward_increase(mod_weights)
			# upper_w_reward = calculate_expected_reward_increase(trajectory_index,trajectory_length,mod_weights)
			mod_weights[j] = temp_weights[j] - weight_h
			# lower_w_reward = calculate_expected_reward_increase(trajectory_index,trajectory_length,mod_weights)
			lower_w_reward = calculate_expected_reward_increase(mod_weights)

			reward_derivative = upper_w_reward - lower_w_reward
			reward_derivative /= weight_h

			temp_weights[j] = temp_weights[j] - alpha_1 * reward_derivative

		temp_weights = temp_weights[:]/temp_weights.sum()

		prev_reward_value = cur_reward_value
		cur_reward_value = calculate_expected_reward_increase(mod_weights)

		if ((cur_reward_value - prev_reward_value)<epsilon):
			convergence_test = npy.roll(convergence_test,-1)
			convergence_test[buffer_size-1]

	return temp_weights


# def learn_weights(state_from, state_to, action):
def learn_weights():
	basis_size=3
	reward_weights = npy.ones(basis_size)/basis_size
	# up_mod_weights = reward_weights
	# for trajectory_index in range(0,number_trajectories):
	# 	# up_mod_weights = update_weights(trajectory_index,trajectory_lengths[trajectory_index],reward_weights)	
	# 	up_mod_weights = update_weights(reward_weights)
	# 	reward_weights[:] = reward_weights[:] + alpha*up_mod_weights[:]
	# 	# alpha = ((number_trajectories-1)/number_trajectories)*alpha
	# 	alpha = 0.9 * alpha

	up_mod_weights = update_weights(reward_weights)

print learn_weights()

# 
# weights()