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
discrete_size = 50
action_size = 8
action_space = [[0,1],[1,0],[0,-1],[-1,0],[1,1],[1,-1],[-1,1],[-1,-1]]

transition_space = 3

basis_functions = npy.loadtxt(str(sys.argv[1]))
basis_functions = basis_functions.reshape((basis_size,discrete_size,discrete_size))

time_limit = 100
value_functions = npy.zeros(shape=(time_limit,discrete_size,discrete_size))

weights = npy.loadtxt(str(sys.argv[2]))
reward_function = npy.zeros(shape=(discrete_size,discrete_size))

reward_function = npy.dot(weights,basis_functions)

# transition_matrix = npy.zeros(shape=(transition_space,transition_space,action_space,transition_space,transition_space))
def transition_value(from_state,to_state,act_index):
	rel_state = from_state - to_state
	actual_state = from_state + action_space[act_index]	
	act_rel_state = actual_state - to_state
	if (npy.linalg.norm(rel_state)>2):
		return 0.
	else:
		if (rel_state==actual_state):
			return 0.7
		if (npy.linalg.norm(rel_state)==0):
			return 0.1
		if (npy.linalg.norm(action_space[act_index])==1):
			if (rel_state==(from_state-action_space[act_index])or(npy.linalg.norm(rel_state)>1)):
				return 0.
			else: 
				return 0.1
		else: 
			if (npy.linalg.norm(rel_state)==1):
				return 0.1
			else:
				return 0.

for t in range(0,time_limit-1):
	for state_i in range(0,discrete_size):
		for state_j in range(0,discrete_size):
			# value_functions[t+1,state_i,state_j] = 
			for act in range(0,action_size):
				for to_state_i in range(-1,1):
					for to_state_j in range(-1,1):
						

