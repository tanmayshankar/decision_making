#!/usr/bin/env python
import rospy
import numpy as npy 
import sys
import random
from scipy.stats import rankdata

number_objects = 41
number_actions = 15

object_list = npy.zeros(number_objects)
action_list = npy.zeros((number_actions,1))
search_objs = npy.zeros((number_objects,1))
search_order = npy.zeros((number_objects,1))

obj_aff_matrix = npy.loadtxt(sys.argv[1])
print (obj_aff_matrix.shape)

for j in range(0,number_actions):
	obj_aff_matrix[j] = obj_aff_matrix[j]/sum(obj_aff_matrix[j])

print("Object Affordance Matrix:")
# print(obj_aff_matrix)

number_scene_objects = 10

for i in range(0,number_scene_objects):
	trial_label = random.randrange(0,number_objects)
	object_list[trial_label] = random.random()

# object_list = npy.linspace(0,1,number_objects)
object_list = npy.transpose(object_list)
print("Object List:") 	
print(object_list)
print(object_list.shape)

action_list = npy.dot(obj_aff_matrix,object_list)
for j in range(0,number_actions):
	action_list[j] = action_list[j]/sum(action_list)
print ("Action List:")
print action_list

obj_aff_trans = npy.transpose(obj_aff_matrix)
for j in range(0,number_actions):
	obj_aff_trans[j] = obj_aff_trans[j]/sum(obj_aff_trans[j])

search_objs = npy.dot(obj_aff_trans,action_list)
search_objs = search_objs - object_list 

search_objs = object_list - search_objs    ###THIS SHOULD BE SEARCH_OBJS - OBJECT_LIST, but since argsort is ascending..... 
search_order = npy.argsort(search_objs)
# search_order_try = npy.argsort(search_order+1)
search_order_try = map(int,rankdata(search_objs,method='ordinal'))

print("Search objects:")
print(-search_objs)

# print("Search order:")
# print(search_order+1)

print("Actual search order:")
print(search_order_try)

