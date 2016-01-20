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
import random
from scipy.stats import rankdata
import math
from matplotlib.pyplot import *
import copy
import matplotlib.pyplot as plt

radius_threshold = 10
discrete_size = 100

discrete_space_x = 50
discrete_space_y = 50
max_dist = 5

action = 'e'
new_demo = 'y'

max_path_length=100
current_pose = [0,0]
max_number_demos = 50

trajectory_lengths = npy.zeros(max_number_demos)

state_counter = 0
number_demos = 0

trajectories = [[[0,0],[1,2],[3,4]]]

#Dummy set of variables for the random object spatial querying. 
space_dist = npy.linspace(-max_dist,max_dist,discrete_space_x)

# number_objects = 130
number_objects = 41

lower_bound=0
upper_bound=radius_threshold

rad_dist = npy.linspace(0,radius_threshold,discrete_size)
# print rad_dist

#READING THE spatial distribution function from file. 
pairwise_value_function = npy.loadtxt(str(sys.argv[1]))
# Note that this returned a 2D array!
print pairwise_value_function.shape
pairwise_value_function = pairwise_value_function.reshape((number_objects,number_objects,discrete_size))

value_function = npy.zeros(shape=(discrete_space_x,discrete_space_y))
reward_val = npy.zeros(shape=(discrete_space_x,discrete_space_y))
value_functions = npy.zeros(shape=(number_objects,discrete_space_x,discrete_space_y))
# number_objects = 41
object_confidence = npy.zeros(number_objects)
object_poses = npy.zeros(shape=(number_objects,2))

# number_objects = 41
object_location_function = npy.zeros(shape=(discrete_space_x,discrete_space_y))

def random_obj_positions():
	number_scene_objects = 30
	xbucket=0
	ybucket=0
	for i in range(0,number_scene_objects):		
		trial_label = random.randrange(0,number_objects)
		object_confidence[trial_label] = random.random()

	for i in range(0,number_objects):
		object_poses[i][0] = random.randrange(-max_dist,max_dist)
 		object_poses[i][1] = random.randrange(-max_dist,max_dist)		

 		if object_poses[i][0]<space_dist[0]:
			xbucket=0
		elif object_poses[i][0]>space_dist[len(space_dist)-1]:
			xbucket=len(space_dist)-1
		else:
			for j in range(0,len(space_dist)):	
				if object_poses[i][0]>space_dist[j] and object_poses[i][0]<space_dist[j+1]:
					xbucket=j

		if object_poses[i][1]<space_dist[0]:
			ybucket=0
		elif object_poses[i][1]>space_dist[len(space_dist)-1]:
			ybucket=len(space_dist)-1
		else:
			for j in range(0,len(space_dist)):	
				if object_poses[i][1]>space_dist[j] and object_poses[i][0]<space_dist[j+1]:
					ybucket=j

		object_location_function[xbucket][ybucket] = object_confidence[i]


random_obj_positions()

#REMEMBER, this is outside the lookup_value_add function. 
def lookup_value_add(sample_pt, obj_find_index):
	value_lookup=0
	for alt_obj in range(0,number_objects):	
		
		rad_value = (((sample_pt[0]-object_poses[alt_obj][0])**2)+((sample_pt[1]-object_poses[alt_obj][1])**2))**0.5
		if rad_value<rad_dist[0]:
			bucket=0;
		elif rad_value>rad_dist[len(rad_dist)-1]:
			bucket=len(rad_dist)-1
		else:
			for i in range(0,len(rad_dist)):	
				if rad_value>rad_dist[i] and rad_value<rad_dist[i+1]:
					bucket=i
		# print sample_pt, rad_value, bucket

		value_lookup += pairwise_value_function[obj_find_index][alt_obj][bucket] * object_confidence[alt_obj] / number_objects
	return value_lookup

def calculate_value_function(obj_find_index):	
	for i in range(0,discrete_space_x): 		
		for j in range(0,discrete_space_y):						
			sample = space_dist[i],space_dist[j]		
			x = lookup_value_add(sample, obj_find_index)			
			value_functions[obj_find_index][i][j]=x

calculate_value_function(4)
calculate_value_function(1)
calculate_value_function(3)

weights = npy.ones(3)
weights[0] = 0.7
weights[1] = 0.5
weights[2] = 0.3
weights=weights[:]/weights.sum()

# reward_val+= (1-value_functions[4])*weights[0] + (1-value_functions[1])*weights[1] + (1-value_functions[3])*weights[2] #- costmap*weights[3]
reward_val+= (value_functions[4])*weights[0] + (value_functions[1])*weights[1] + (value_functions[3])*weights[2] #- costmap*weights[3]

path_plot = npy.zeros(shape=(discrete_space_x,discrete_space_y))
path_plot = copy.deepcopy(reward_val)

max_val = npy.amax(path_plot)

####PRINTING THE OBJECT LOCATION FUNCTION
def parse_demonstrations():

	new_demo = 'y'
	state_counter = 0
	number_demos = 0
	while (new_demo!='n'):
	
		state_counter=0	
		current_demo = [[0,0]]
		path_plot = copy.deepcopy(reward_val)

		ax = random.randrange(0,discrete_space_x)
		ay = random.randrange(0,discrete_space_y)

		current_pose[0] = ax
		current_pose[1] = ay
		
		current_demo[0][0] = ax
		current_demo[0][1] = ay

		# path_plot[:,:]=0.

		path_plot[ax,ay]=-max_val/4
		action='e'

		while (action!='q'):		
		
			if action=='w':			
				state_counter+=1	
				current_demo.append([current_pose[0]+1,current_pose[1]])
				current_pose[0]+=1					
			if action=='a':			
				state_counter+=1		
				current_demo.append([current_pose[0],current_pose[1]-1])
				current_pose[1]-=1			
			if action=='d':			
				state_counter+=1
				current_demo.append([current_pose[0],current_pose[1]+1])
				current_pose[1]+=1
			if action=='s':			
				state_counter+=1
				current_demo.append([current_pose[0]-1,current_pose[1]])
				current_pose[0]-=1
			if ((action=='wa')or(action=='aw')):			
				state_counter+=1	
				current_demo.append([current_pose[0]+1,current_pose[1]-1])
				current_pose[0]+=1	
				current_pose[1]-=1						
			if ((action=='sa')or(action=='as')):					
				state_counter+=1		
				current_demo.append([current_pose[0]-1,current_pose[1]-1])
				current_pose[1]-=1
				current_pose[0]-=1		
			if ((action=='sd')or(action=='ds')):					
				state_counter+=1
				current_demo.append([current_pose[0]-1,current_pose[1]+1])
				current_pose[1]+=1
				current_pose[0]-=1
			if ((action=='wd')or(action=='dw')):					
				state_counter+=1
				current_demo.append([current_pose[0]+1,current_pose[1]+1])
				current_pose[0]+=1			
				current_pose[1]+=1			

			path_plot[current_pose[0]][current_pose[1]]=-max_val/4
			imshow(path_plot, interpolation='nearest', origin='lower', extent=[0,10,0,10], aspect='auto')
			plt.show(block=False)
			colorbar()
			draw()
			show() 
			
			action = raw_input("Hit a key now: ")
			
			trajectory_lengths[number_demos] = state_counter+1

		number_demos+=1
		print "Current demo was: ",current_demo
		
		trajectories.append(current_demo)

		new_demo = raw_input("Do you want to start a new demonstration? ")
	
parse_demonstrations()
trajectory_lengths=trajectory_lengths.astype(int)

trajectories.remove(trajectories[0])
print "The trajectories are as follows: ",trajectories

with file('trajectories.txt','w') as outfile:
	# for data_slice in pairwise_value_func:
	for data_slice in trajectories:
		outfile.write('#New slice\n')
		npy.savetxt(outfile,data_slice,fmt='%-7.2f')
		
with file('basis_functions.txt','w') as outfile:
	outfile.write('#New basis 4.\n')
	npy.savetxt(outfile,value_functions[4],fmt='%-7.2f')
	outfile.write('#New basis 3.\n')
	npy.savetxt(outfile,value_functions[3],fmt='%-7.2f')
	outfile.write('#New basis 1.\n')
	npy.savetxt(outfile,value_functions[1],fmt='%-7.2f')

with file('weight_values.txt','w') as outfile: 
	outfile.write('#Weight values.\n')
	npy.savetxt(outfile,weights,fmt='%-7.2f')

with file('trajectory_lengths.txt','w') as outfile: 
	outfile.write('#Trajectory_lengths.\n')
	npy.savetxt(outfile,trajectory_lengths,fmt='%-7.2f')


imshow(object_location_function, interpolation='nearest', origin='lower', extent=[0,10,0,10], aspect='auto')
# imshow(path_plot, interpolation='nearest', origin='lower', extent=[0,10,0,10], aspect='auto')
plt.show(block=False)
colorbar()
draw()
show()

X,Y=npy.meshgrid(space_dist,space_dist)
fig1=plt.figure(1)
ax1 = fig1.add_subplot(111,projection='3d')
ax1.plot_surface(X,Y,reward_val,cmap=plt.cm.jet,cstride=1,rstride=1)
draw()
show()

# # ax1.plot_surface(X,Y,object_location_function,cmap=plt.cm.jet,cstride=1,rstride=1)

# ax2 = fig2.add_subplot(111,projection='3d')
# ax2.plot_surface(X,Y,value_functions[4],cmap=plt.cm.jet,cstride=1,rstride=1)

# ax3 = fig3.add_subplot(111,projection='3d')
# ax3.plot_surface(X,Y,value_functions[1],cmap=plt.cm.jet,cstride=1,rstride=1)

# ax4 = fig4.add_subplot(111,projection='3d')
# ax4.plot_surface(X,Y,value_functions[3],cmap=plt.cm.jet,cstride=1,rstride=1)

# plt.show()



















