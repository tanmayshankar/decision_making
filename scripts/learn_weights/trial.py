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

x=npy.linspace(0,1,5)
f1=npy.linspace(0,1,5)
f2=npy.linspace(0.8,0.3,5)
f3=npy.ones(5)/3
f=npy.transpose([f1,f2,f3])

print f

def frange(start, stop, step):
	i = start
	while i < stop:
		yield i
		i += step

def weights():
	max_val=-100000
	# w_star=[0,0,0]
	for w1 in npy.linspace(0,1,11):
		for w2 in frange(0,1.01-w1,0.1):
			for w3 in frange(0,1.01-w1-w2,0.1):
				w=[w1,w2,w3]				
				sum_val=0
				for t in range(1,x.size):
					sum_val+=(0.95**(t-1))*(npy.dot(w,f[:][t])-npy.dot(w,f[:][t-1]))
				if (sum_val>max_val):
					max_val=sum_val
					print w

weights()