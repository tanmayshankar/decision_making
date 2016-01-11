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

max_dist = 5

action = 'd'
discrete_space_x = 50
discrete_space_y = 50

#Dummy set of variables for the random object spatial querying. 
space_dist = npy.linspace(-max_dist,max_dist,discrete_space_x)


path_plot = npy.zeros(shape=(discrete_space_x,discrete_space_y))

max_path_length=100
current_pose = npy.zeros(2)
start_pose = npy.zeros(2)
# ax.plot_surface(X,Y,path_plot,cmap=plt.cm.jet,cstride=1,rstride=1)
pose_train = npy.zeros(shape=(max_path_length,2))

state_counter = 0
pose_train[state_counter] = start_pose

while (action!='q'):
	action = raw_input("Hit a key now: ")
	print (action)
	# fig = plt.figure()
	if action=='w':
		current_pose[0]+=1
		state_counter+=1
		pose_train[state_counter]=current_pose
		# pose_train.append(current_pose)
	if action=='a':
		current_pose[1]-=1
		state_counter+=1
		pose_train[state_counter]=current_pose
		# pose_train.append(current_pose)
	if action=='d':
		current_pose[1]+=1
		state_counter+=1
		pose_train[state_counter]=current_pose
		# pose_train.append(current_pose)
	if action=='s':
		current_pose[0]-=1
		state_counter+=1
		pose_train[state_counter]=current_pose
		# pose_train.append(current_pose)

	path_plot[current_pose[0]][current_pose[1]]=1
	imshow(path_plot, interpolation='nearest', origin='lower', extent=[0,10,0,10], aspect='auto')
	plt.show(block=False)
	colorbar()
	draw()
	show() 

# pose_train=npy.array(pose_train)
print pose_train
