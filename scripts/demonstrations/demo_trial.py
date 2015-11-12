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

# fig = plt.figure(1)
# path_plot = npy.zeros(shape=(discrete_space_x,discrete_space_y))
# X,Y=npy.meshgrid(space_dist,space_dist)
# ax = fig.add_subplot(111,projection='3d')

path_plot = npy.zeros(shape=(discrete_space_x,discrete_space_y))
# Z[4][6]=34
# Z = path_plot



current_pose = npy.zeros(2)
# ax.plot_surface(X,Y,path_plot,cmap=plt.cm.jet,cstride=1,rstride=1)


while (action!='q'):
	action = raw_input("Hit a key now")
	print (action)
	# fig = plt.figure()
	
	if action=='w':
		current_pose[1]+=1
	if action=='a':
		current_pose[0]-=1
	if action=='d':
		current_pose[0]+=1
	if action=='s':
		current_pose[1]-=1

	path_plot[current_pose[0]][current_pose[1]]=1
	imshow(path_plot, interpolation='nearest', origin='lower', extent=[0,10,0,10], aspect='auto')
	plt.show(block=False)
	colorbar()
	draw()
	show()


