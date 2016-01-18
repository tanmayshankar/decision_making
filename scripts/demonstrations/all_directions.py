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

trajectory_lengths = npy.zeros(max_number_demos)

state_counter = 0
number_demos = 0

trajectories = [[[0,0],[1,2],[3,4]]]
	
while (new_demo!='n'):
	
	state_counter=0	
	current_demo = [[0,0]]
	
	ax = random.randrange(0,discrete_space_x)
	ay = random.randrange(0,discrete_space_y)

	current_pose[0] = ax
	current_pose[1] = ay
	
	current_demo[0][0] = ax
	current_demo[0][1] = ay

	path_plot[:,:]=0.
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

		path_plot[current_pose[0]][current_pose[1]]=1		
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
	
trajectory_lengths=trajectory_lengths.astype(int)

trajectories.remove(trajectories[0])
print trajectories

with file('trajectories.txt','w') as outfile:
	# for data_slice in pairwise_value_func:
	for data_slice in trajectories:
		npy.savetxt(outfile,data_slice,fmt='%-7.2f')
		outfile.write('# New slice\n')
