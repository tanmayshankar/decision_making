#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as npy

x = npy.linspace(0, 6*npy.pi, 100)
y = npy.sin(x)

# You probably won't need this if you're embedding things in a tkinter plot...
plt.ion()

fig = plt.figure()
ax = fig.add_subplot(111)
line1, = ax.plot(x, y, 'r-') # Returns a tuple of line objects, thus the comma
# 
# discrete_space_x = 50
# discrete_space_y = 50
# path_plot = npy.zeros(shape=(discrete_space_x,discrete_space_y))
# plt.imshow(path_plot, interpolation='nearest', origin='lower', extent=[0,10,0,10], aspect='auto')

# plt.show()		
for phase in npy.linspace(0, 10*npy.pi, 500):
    line1.set_ydata(npy.sin(x + phase))
    fig.canvas.draw()