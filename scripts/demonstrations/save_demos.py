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

action = 'e'
discrete_space_x = 50
discrete_space_y = 50

new_demo = 'y'

#Dummy set of variables for the random object spatial querying. 
space_dist = npy.linspace(-max_dist,max_dist,discrete_space_x)

path_plot = npy.zeros(shape=(discrete_space_x,discrete_space_y))

max_path_length=100
current_pose = [0,0]
# ax.plot_surface(X,Y,path_plot,cmap=plt.cm.jet,cstride=1,rstride=1)
max_number_demos = 50
pose_train = npy.zeros(shape=(max_number_demos,max_path_length,2))

trajectory_lengths = npy.zeros(max_number_demos)

state_counter = 0
pose_train[state_counter] = start_pose

number_demos = 0

while (new_demo!='n'):
	
	state_counter=0	
	
	current_pose[0]= random.randrange(0,discrete_space_x)
	current_pose[1]= random.randrange(0,discrete_space_y)
	
	pose_train[number_demos,state_counter,0] = current_pose[0]
	pose_train[number_demos,state_counter,1] = current_pose[1]

	path_plot[:,:]=0.
	action='e'

	while (action!='q'):	
		# fig = plt.figure()
		if action=='w':
			current_pose[0]+=1
			state_counter+=1
			pose_train[number_demos,state_counter]=current_pose
			# pose_train[number_demos][state_counter]=current_pose
			# pose_train.append(current_pose)
		if action=='a':
			current_pose[1]-=1
			state_counter+=1
			pose_train[number_demos,state_counter]=current_pose
			# pose_train[number_demos][state_counter]=current_pose
			# pose_train.append(current_pose)
		if action=='d':
			current_pose[1]+=1
			state_counter+=1
			pose_train[number_demos,state_counter]=current_pose
			# pose_train[number_demos][state_counter]=current_pose
			# pose_train.append(current_pose)
		if action=='s':
			current_pose[0]-=1
			state_counter+=1
			# pose_train[number_demos][state_counter]=current_pose
			pose_train[number_demos,state_counter]=current_pose
			# pose_train.append(current_pose)

		path_plot[current_pose[0]][current_pose[1]]=1		
		imshow(path_plot, interpolation='nearest', origin='lower', extent=[0,10,0,10], aspect='auto')
		plt.show(block=False)
		colorbar()
		draw()
		show() 

		action = raw_input("Hit a key now: ")
		# print (action)
		
		trajectory_lengths[number_demos] = state_counter+1
		# print pose_train[number_demos,:,:]
	number_demos+=1
	new_demo = raw_input("Do you want to start a new demonstration? ")

with file('trajectories.txt','w') as outfile:
	for data_slice in trajectories:
		npy.savetxt(outfile,data_slice,fmt='%-7.2f')
		outfile.write('# New slice\n')

# print (new_demo)
# print "This is the pose train: ",pose_train

# # trajectories = npy.zeros(shape=(state_counter+1,2))
# # trajectories = npy.zeros(shape=(number_demos,state_counter+1,2))
trajectories = npy.zeros(shape=(number_demos,state_counter+1,2))

trajectory_lengths=trajectory_lengths.astype(int)
	for j in range(0,number_demos):
	 	for i in range(0,trajectory_lengths[j]):
	 		trajectories[j,i,:]=pose_train[j,i,:]
# 		# trajectories[jet][i][:] = pose_train[j][i][:]
#  # pose_train=npy.array(pose_train)

with file('trajectories.txt','w') as outfile:
	# for data_slice in pairwise_value_func:
	for data_slice in trajectories:
		npy.savetxt(outfile,data_slice,fmt='%-7.2f')
		outfile.write('# New slice\n')

print trajectories