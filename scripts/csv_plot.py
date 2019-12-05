import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from parking_plot import *

def plot_primitives(startS, goalS):
	ss = pd.read_csv("startS.csv").values
	sg = pd.read_csv("goalS.csv").values

	xlim = [-62,30]
	ylim = [-40, 40]

	plt.scatter(startS[0], startS[1])
	plt.scatter(goalS[0], goalS[1])

	i=0
	while i<(ss.shape[1]-1):
		plt.plot(ss[:,i], ss[:,1+i] ,'r')
		i+=3

	i=0
	while i<(sg.shape[1]-1):
		plt.plot(sg[:,i], sg[:,1+i], 'b')
		i+=3

	plt.xlim(xlim[0], xlim[1])
	plt.ylim(ylim[0], ylim[1])

	plt.show()

def plot_path(startS, goalS):
	f = pd.read_csv("waypoints.csv").values

	xlim = [-62,30]
	ylim = [-40, 40]

	plt.scatter(startS[0] ,startS[1], s=10)
	plt.scatter(goalS[0], goalS[1], s=10)

	plt.scatter(f[:,0], f[:,1], s=2)
	plt.show()

def plot_swath(startS, goalS):

	f = pd.read_csv("waypoints.csv").values

	xlim = [-62,30]
	ylim = [-40, 40]

	l = 0.8
	w = 2
	fig, ax =plt.subplots(1)



	# Plotting the occupancy grid....................
	map1 = Map(0.2, 0.2)
	# occ_grid = map1.compute_occupancy_grid([44])
	# ax.imshow(occ_grid.T, "Greys")
	map1.plot_parking()

	# Plotting the vehicle path swath
	rect = Rectangle((startS[0], startS[1]), w, l, angle = startS[2]*180/np.pi, linewidth=0.2, edgecolor='r',facecolor='none')
	ax.add_patch(rect)
	rect = Rectangle((goalS[0], goalS[1]), w, l, angle = goalS[2]*180/np.pi, linewidth=0.2, edgecolor='r',facecolor='none')
	ax.add_patch(rect)
	for i in range(f.shape[0]):
		rect = Rectangle((f[i,0], f[i,1]), w, l, angle = f[i,2]*180/np.pi, linewidth=0.2, edgecolor='r',facecolor='none')
		ax.add_patch(rect)

	plt.xlim(xlim[0], xlim[1])
	plt.ylim(ylim[0], ylim[1])
	
	plt.show()

	# --- Saving the trajectory in .npy file
	wpt = f
	wpt[:,1] = -wpt[:,1]
	wpt[:,2] = -wpt[:,2]
	np.save('waypoints.npy', wpt)
	print('Saved the referene trajectory in ............waypoints.npy')



if __name__ =='__main__':	
	PI = np.pi
	start_state = [-15, 30, 3*PI/2]

	goal_state = [-54.12901306152344, -2.4843921661376953,0]
	# start_state = [-5, 15]
	# goal_state = [15,15]

	# plot_primitives(start_state, goal_state)
	# plot_path(start_state, goal_state)
	plot_swath(start_state, goal_state)