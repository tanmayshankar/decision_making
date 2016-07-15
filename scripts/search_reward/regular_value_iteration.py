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

basis_size = 3
discrete_size = 250
action_size = 8
action_space = [[0,1],[1,0],[0,-1],[-1,0],[1,1],[1,-1],[-1,1],[-1,-1]]

transition_space = 7

time_limit = 2
# value_functions = npy.zeros(shape=(time_limit,discrete_size,discrete_size))
value_function = npy.zeros((discrete_size,discrete_size))

optimal_policy = npy.zeros(shape=(discrete_size,discrete_size))

# reward_function = npy.loadtxt(str(sys.argv[1]))
reward_function = npy.zeros((discrete_size,discrete_size))
dummy_rew = npy.loadtxt(str(sys.argv[1]))
# reward_function = npy.loadtxt(str(sys.argv[1]))
# reward_function[25:75,25:75]=dummy_rew
# reward_function[50:100,50:100]=dummy_rew
# reward_function[75:125,75:125]=dummy_rew
reward_function[100:150,100:150]=dummy_rew
reward_function /=1000.0


dummy_max = npy.amax(reward_function)

gamma = 0.95


def transition_value(to_state_i,to_state_j,act_index):	
	# trans_mat_1 = [[0.,0.7,0.],[0.1,0.1,0.1],[0.,0.,0.]]
	# trans_mat_2 = [[0.7,0.1,0.],[0.1,0.1,0.],[0.,0.,0.]]

	# trans_mat_1 = [[0.,0.,0.1,0.,0.],[0.,0.05,0.6,0.05,0.],[0.,0.05,0.1,0.05,0.],[0.,0.,0.,0.,0.],[0.,0.,0.,0.,0.]]
	# trans_mat_2 = [[0.1,0.05,0.,0.,0.],[0.05,0.6,0.05,0.,0.],[0.,0.05,0.1,0.,0.],[0.,0.,0.,0.,0.],[0.,0.,0.,0.,0.]]

	trans_mat_1 = [[0.,0.,0.,0.,0.,0.,0.],[0.,0.1,0.05,0.,0.,0.,0.],[0.,0.05,0.6,0.05,0.,0.,0.],[0.,0.,0.05,0.1,0.,0.,0.],[0.,0.,0.,0.,0.,0.,0.],[0.,0.,0.,0.,0.,0.,0.],[0.,0.,0.,0.,0.,0.,0.]]
	trans_mat_2 = [[0.,0.,0.,0.,0.,0.,0.],[0.,0.1,0.05,0.,0.,0.,0.],[0.,0.05,0.6,0.05,0.,0.,0.],[0.,0.,0.05,0.1,0.,0.,0.],[0.,0.,0.,0.,0.,0.,0.],[0.,0.,0.,0.,0.,0.,0.],[0.,0.,0.,0.,0.,0.,0.]]

	trans_mat = npy.zeros(shape=(action_size,transition_space, transition_space))
	trans_mat[0] = trans_mat_1
	trans_mat[1] = npy.rot90(trans_mat_1)
	trans_mat[2] = npy.rot90(trans_mat_1,3)
	trans_mat[3] = npy.rot90(trans_mat_1,2)

	trans_mat[7] = trans_mat_2
	trans_mat[5] = npy.rot90(trans_mat_2)
	trans_mat[4] = npy.rot90(trans_mat_2,2)
	trans_mat[6] = npy.rot90(trans_mat_2,3)

	return trans_mat[act_index,to_state_i,to_state_j]
	
def policy_iteration():
	value_per_action = npy.zeros(8)
	to_state = [0,0]	
	for t in range(0,time_limit-1):
		print "Time step:",t
		for from_state_i in range(0,discrete_size):
			for from_state_j in range(0,discrete_size):
				# value_functions[t+1,state_i,state_j] = 				
				# print "For state:",from_state_i,from_state_j
				value_per_action[:]=reward_function[from_state_i,from_state_j]
				max_val = 0.
				for act in range(0,action_size):
					
					# print "For action:",act
					h = transition_space/2
					for to_state_i in range(-h,h+1):
						for to_state_j in range(-h,h+1):
							to_state[0]=from_state_i+to_state_i
							to_state[1]=from_state_j+to_state_j

							if (to_state[0]>=discrete_size):
								to_state[0]=discrete_size-1
							elif (to_state[0]<=-1):
								to_state[0]=0
							if (to_state[1]>=discrete_size):
								to_state[1]=discrete_size-1
							elif (to_state[1]<=-1):
								to_state[1]=0
							# print "To state:",to_state[0],to_state[1]
							# trans_prob = transition_value([state_i,state_j],[state_i+to_state_i,state_j+to_state_j],act)
							# trans_prob = transition_value([from_state_i,from_state_j],to_state,act)
							trans_prob = transition_value(to_state_i,to_state_j,act)
							# if (trans_prob>0):
								# print "Bahahahhahahahahahaha",trans_prob							
							#BELLMAN UPDATE EQUATION - part 1. 
							value_per_action[act] += trans_prob*(gamma*value_function[to_state[0],to_state[1]])

					# value_per_action[act]+=reward_function[to_state[0],to_state[1]]

					if (value_per_action[act]>max_val):
						max_val = value_per_action[act]
						optimal_policy[from_state_i,from_state_j] = act
						value_function[from_state_i,from_state_j] = max_val
					
# 	for i in range(0,discrete_size):
# 		# for j in range(0,discrete_size):
# 		print optimal_policy[i]

# for i in range(0,discrete_size):
# 	print reward_function[i]

policy_iteration()

# print "These are the value functions."
# for t in range(0,time_limit):
# 	print value_functions[t]


# with file('output_policy.txt','w') as outfile: 
# 	outfile.write('#Policy.\n')
# 	npy.savetxt(outfile,optimal_policy,fmt='%-7.2f')