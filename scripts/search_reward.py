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
from matplotlib.pyplot import *

radius_threshold = 10
discrete_size = 100

discrete_space_x = 50
discrete_space_y = 50
max_dist = 5

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

value_functions = npy.zeros(shape=(number_objects,discrete_space_x,discrete_space_y))
# number_objects = 41
object_confidence = npy.zeros(number_objects)
object_poses = npy.zeros(shape=(number_objects,2))

# number_objects = 41
object_location_function = npy.zeros(shape=(discrete_space_x,discrete_space_y))

def random_obj_positions():
	number_scene_objects = 30

	for i in range(0,number_scene_objects):		
		trial_label = random.randrange(0,number_objects)
		object_confidence[trial_label] = random.random()

	for i in range(0,number_objects):
		object_poses[i][0] = random.randrange(-max_dist,max_dist)
 		object_poses[i][1] = random.randrange(-max_dist,max_dist)		

 		if object_poses[i][0]<space_dist[0]:
			xbucket=0;
		elif object_poses[i][0]>space_dist[len(space_dist)-1]:
			xbucket=len(space_dist)-1
		else:
			for j in range(0,len(space_dist)):	
				if object_poses[i][0]>space_dist[j] and object_poses[i][0]<space_dist[j+1]:
					xbucket=j

		if object_poses[i][1]<space_dist[0]:
			ybucket=0;
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

# calculate_value_function(4)
# calculate_value_function(1)
# calculate_value_function(3)
# 
# print space_dist

# fig1 = plt.figure(1)
# fig2 = plt.figure(2)
# fig3 = plt.figure(3)
# fig4 = plt.figure(4)
# X,Y=npy.meshgrid(space_dist,space_dist)

weights = npy.ones(4)
weights[0] = 0.7
weights[1] = 0.5
weights[2] = 0.3
weights[3] = 0.01
weights=weights[:]/weights.sum()

# combo_val = value_functions[:]/3
reward_val = value_functions[0]

# for i in range(0,number_objects):
# 	combo_val+= value_functions[i] 
# combo_val /=3



costmap = npy.loadtxt(str(sys.argv[2]))
print costmap.shape
print value_functions[0].shape


# reward_val+= value_functions[4]*weights[0] + value_functions[1]*weights[1] + value_functions[3]*weights[2] + costmap*weights[3]
reward_val = costmap
# print reward_val

robot_pose = npy.zeros(2)
robot_bucket = npy.zeros(2)
robot_pose[0] = 3
robot_pose[1] = 2 

for i in range(0,2):
	# print i
	if (robot_pose[i]<space_dist[0]):
		robot_bucket[i]=0;
	elif (robot_pose[i]>space_dist[len(space_dist)-1]):
		robot_bucket	[i]=len(space_dist)-1
	else:
		for j in range(0,len(space_dist)):	
			if robot_pose[i]>space_dist[j] and robot_pose[i]<space_dist[j+1]:
				robot_bucket[i]=j

def move_one_step(search_start):
	state_deriv = 1
	deriv_rate = 2
	epsilon = 0.01
	reward_derivative = npy.zeros(4)
	max_deriv = 0
	next_state = npy.zeros(2)

	# 0 along x. 
	reward_derivative[0] = reward_val[search_start[0]+state_deriv][search_start[1]] - reward_val[search_start[0]-state_deriv][search_start[1]]
	# 1 along y. 
	reward_derivative[1] = reward_val[search_start[0]][search_start[1]+state_deriv] - reward_val[search_start[0]][search_start[1]-state_deriv]
	# 2 along +x +y. 
	reward_derivative[2] = reward_val[search_start[0]+state_deriv][search_start[1]+state_deriv] - reward_val[search_start[0]-state_deriv][search_start[1]-state_deriv]
	# 3 along +x -y.
	reward_derivative[3] = reward_val[search_start[0]+state_deriv][search_start[1]-state_deriv] - reward_val[search_start[0]-state_deriv][search_start[1]+state_deriv]
		
	max_deriv_arg = npy.argmax(npy.absolute(reward_derivative))
	max_deriv = npy.amax(npy.absolute(reward_derivative))
	# print "Maximum derivative:", max_deriv

	if max_deriv<epsilon:
		print "Well....",max_deriv
		next_state[0] = -1
		next_state[1] = -1
	else:
		print "Index:",max_deriv_arg,max_deriv
		if (max_deriv_arg==0):
			next_state[0] = search_start[0] - reward_derivative[max_deriv_arg] * deriv_rate
			next_state[1] = search_start[1]
		elif (max_deriv_arg==1):
			next_state[0] = search_start[0]
			next_state[1] = search_start[1] - reward_derivative[max_deriv_arg] * deriv_rate
		elif (max_deriv_arg==2):
			next_state[0] = search_start[0] - reward_derivative[max_deriv_arg] * deriv_rate
			next_state[1] = search_start[1] - reward_derivative[max_deriv_arg] * deriv_rate
		elif (max_deriv_arg==3):
			next_state[0] = search_start[0] - reward_derivative[max_deriv_arg] * deriv_rate
			next_state[1] = search_start[1] + reward_derivative[max_deriv_arg] * deriv_rate

	return next_state

path_plot = npy.zeros(shape=(discrete_space_x,discrete_space_y))
path_plot = reward_val

def gradient_search(search_start):
	grad_state = search_start
	try_state = search_start
	# while (!((try_state[0]==-1)&&(try_state[1]==-1))):
	x=0
	while ((try_state[0]!=-1)and(try_state[1]!=-1)):
	# if 1:
		grad_state = try_state
		try_state = move_one_step(grad_state)
		x+=1
		print x, try_state
		path_plot[try_state[0]][try_state[1]]=-1
		# print try_state
	return grad_state

print "Robot bucket:", robot_bucket
print "Robot pose:", robot_pose
print "Resultant bucket:", gradient_search(robot_bucket)


imshow(path_plot, interpolation='nearest', origin='lower', extent=[0,10,0,10], aspect='auto')
plt.show(block=False)
colorbar()


X,Y=npy.meshgrid(space_dist,space_dist)
# fig1=plt.figure(1)
# ax1 = fig1.add_subplot(111,projection='3d')
# ax1.plot_surface(X,Y,reward_val,cmap=plt.cm.jet,cstride=1,rstride=1)
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



















# figd = plt.figure(0)
# axd = figd.add_subplot(111,projection='3d')
# ax = [axd,axd,axd/,axd,axd]
# # ax[i in range(0,5)]=axd
# # fig[i in range(0,5)]=figd
# fig = [figd,figd,figd,figd,figd]

# for i in range(0,5):
# 	calculate_value_function(i)
# 	super_value_function[i]=value_function
# 	fig[i] = plt.figure(i)
# 	ax[i] = fig[i].add_subplot(111,projection='3d')
# 	ax[i].plot_surface(X,Y,super_value_function[i],cmap=plt.cm.jet,cstride=1,rstride=1)
# plt.show()

# def callback(data):
#     # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
#     #rospy.loginfo(rospy.get_caller_id() + "Data read was %s", data.data)
#     trial_element=data.pose.pose.position    
#     print trial_element

#marker_list;



# def ar_marker_callback(msg):
# 	for i in range(0,len(msg.markers)):		
# 		label = msg.markers[i].id
# 		object_confidence[label] = msg.markers[i].confidence
# 		object_poses[label][0] = msg.markers[i].pose.pose.position.discrete_space_x
# 		object_poses[label][1] = msg.markers[i].pose.pose.position.y




# def listener():
#     # The anonymous=True flag means that rospy will choose a unique    
#     # name for our 'listener' node so that multiple listeners can
#     # run simultaneously.
    
#     rospy.init_node('listener', anonymous=True)
#     # rospy.Subscriber("chatter", String, callback)
#     rospy.Subscriber("/ar_pose_marker",AlvarMarkers, ar_marker_callback)

#     # spin() simply keeps python from exiting until this node is stopped
#     rospy.spin()
  
# if __name__ == '__main__':
#     listener()





