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
# # 
# print space_dist

# fig1 = plt.figure(1)
# fig2 = plt.figure(2)
# fig3 = plt.figure(3)
# fig4 = plt.figure(4)
# X,Y=npy.meshgrid(space_dist,space_dist)

weights = npy.ones(3)
weights[0] = 0.7
weights[1] = 0.5
weights[2] = 0.3
# weights[3] = 0.02
weights=weights[:]/weights.sum()

# combo_val = value_functions[:]/3
# reward_val = value_functions[0]

# for i in range(0,number_objects):
# 	combo_val+= value_functions[i] 
# combo_val /=3
# reward_val=copy.deepcopy(combo_val)


# costmap = npy.loadtxt(str(sys.argv[2]))
# print costmap.shape
# print value_functions[0].shape

reward_val+= value_functions[4]*weights[0] + value_functions[1]*weights[1] + value_functions[3]*weights[2] #+ costmap*weights[3]
# reward_val = -0.4*costmap

dummy=100
dummy_x=-1
dummy_y=-1
for i in range(0,discrete_space_x):
	for j in range(0,discrete_space_y):
		if reward_val[i][j]<dummy:
			dummy_x=i
			dummy_y=j
			dummy=reward_val[i][j]

print dummy_x,dummy_y


# # print reward_val

# robot_pose = npy.zeros(2)
# robot_bucket = npy.zeros(2)
# robot_pose[0] = 3
# robot_pose[1] = 2 

# for i in range(0,2):
# 	# print i
# 	if (robot_pose[i]<space_dist[0]):
# 		robot_bucket[i]=0;
# 	elif (robot_pose[i]>space_dist[len(space_dist)-1]):
# 		robot_bucket	[i]=len(space_dist)-1
# 	else:
# 		for j in range(0,len(space_dist)):	
# 			if robot_pose[i]>space_dist[j] and robot_pose[i]<space_dist[j+1]:
# 				robot_bucket[i]=j

def round_off(x):
    n = int(x)
    return n if n-1 < x <= n else n+1

def move_one_step(search_start):
	search_init=search_start
	min_reward = reward_val[search_init[0]][search_init[1]]
	next_state=search_init
	i_c=0
	j_c=0
	for i in range(-1,2):
		for j in range(-1,2):
			print search_init[0]+i,search_init[1]+j,reward_val[search_init[0]+i][search_init[1]+j],search_init
			if reward_val[search_init[0]+i][search_init[1]+j]<min_reward:
				min_reward=reward_val[search_init[0]+i][search_init[1]+j]
				i_c=i
				j_c=j
	next_state[0]=search_init[0]+i_c
	next_state[1]=search_init[1]+j_c
	print "Chosen state:",next_state

	# # 0 along x. 
	# reward_derivative[0] = reward_val[search_start[0]+state_deriv][search_start[1]] - reward_val[search_start[0]-state_deriv][search_start[1]]
	# # 1 along y. 
	# reward_derivative[1] = reward_val[search_start[0]][search_start[1]+state_deriv] - reward_val[search_start[0]][search_start[1]-state_deriv]
	# # 2 along +x +y. 
	# reward_derivative[2] = reward_val[search_start[0]+state_deriv][search_start[1]+state_deriv] - reward_val[search_start[0]-state_deriv][search_start[1]-state_deriv]
	# # 3 along +x -y.
	# reward_derivative[3] = reward_val[search_start[0]+state_deriv][search_start[1]-state_deriv] - reward_val[search_start[0]-state_deriv][search_start[1]+state_deriv]
	
	# # # 4 along -x. 
	# reward_derivative[4] = reward_val[search_start[0]-state_deriv][search_start[1]] - reward_val[search_start[0]+state_deriv][search_start[1]]
	# # # 5 along -y. 
	# reward_derivative[5] = reward_val[search_start[0]][search_start[1]-state_deriv] - reward_val[search_start[0]][search_start[1]+state_deriv]
	# # # 6 along -x -y. 
	# reward_derivative[6] = reward_val[search_start[0]-state_deriv][search_start[1]-state_deriv] - reward_val[search_start[0]+state_deriv][search_start[1]+state_deriv]
	# # # 7 along -x +y.
	# reward_derivative[7] = reward_val[search_start[0]-state_deriv][search_start[1]+state_deriv] - reward_val[search_start[0]+state_deriv][search_start[1]-state_deriv]
	

	return next_state

path_plot = npy.zeros(shape=(discrete_space_x,discrete_space_y))
path_plot = copy.deepcopy(reward_val)
x=0
def gradient_search(search_start):
	grad_state = search_start
	try_state = search_start
	
	max_iter=4000
	

	x=0
	
	buffer_size=10
	convergence_test=npy.zeros(buffer_size)

	while (convergence_test.prod()==0)and(x<max_iter):
		grad_state = try_state
		try_state = move_one_step(grad_state)

		print "State being passed: ",grad_state
		print "State returned: ",try_state
		convergence_test=npy.roll(convergence_test,-1)
		
		# if npy.any(grad_state-try_state):		
		if ((grad_state[0]==try_state[0])and(grad_state[1]==try_state[1])):
			convergence_test[buffer_size-1]=1	
			print "Uh Oh. Maybe."		
		else:			
			convergence_test[buffer_size-1]=0
			print "No problem."
		#For Debugging: 
		x+=1
		# print x, try_state
		path_plot[try_state[0]][try_state[1]]=-0.01
	print "Resultant Bucket:",grad_state
	return grad_state

# path_plot[dummy_x][dummy_y]=-1

# print reward_val

robot_pose = npy.zeros(2)
robot_bucket = npy.zeros(2)
# robot_pose[0] = 3
# robot_pose[1] = 2 
dummy_var=-1000
for i in range(0,discrete_space_x):
	for j in range(0,discrete_space_y):
		if reward_val[i][j]>dummy_var:
			robot_bucket[0]=i
			robot_bucket[1]=j
			dummy_var=reward_val[i][j]

# for i in range(0,2):
# 	# print i
# 	if (robot_pose[i]<space_dist[0]):
# 		robot_bucket[i]=0;
# 	elif (robot_pose[i]>space_dist[len(space_dist)-1]):
# 		robot_bucket	[i]=len(space_dist)-1
# 	else:
# 		for j in range(0,len(space_dist)):	
# 			if robot_pose[i]>space_dist[j] and robot_pose[i]<space_dist[j+1]:
# 				robot_bucket[i]=j

# print "Robot bucket:", robot_bucket
# print "Robot pose:", robot_pose
# # print "Resultant bucket:", 
gradient_search(robot_bucket)

path_plot[robot_bucket[0]][robot_bucket[1]]=-0.01
imshow(path_plot, interpolation='nearest', origin='lower', extent=[0,10,0,10], aspect='auto')
plt.show(block=False)
colorbar()
draw()
show()

X,Y=npy.meshgrid(space_dist,space_dist)
fig1=plt.figure(1)
ax1 = fig1.add_subplot(111,projection='3d')
ax1.plot_surface(X,Y,path_plot,cmap=plt.cm.jet,cstride=1,rstride=1)
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





