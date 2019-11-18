import numpy as np
import matplotlib.pyplot as plt

def traj_roll_out(x0, y0, theta0, num_steps=8, dt=0.1):
	"""
	Function to compute the trajectory from given (x0,yo,theta0).
	Using linear interpolation, based on Kineamtic Bicycle model.
	"""
	L = 2.2	# Longitudinal length of 2.2 m
	# Defining a range of discretized steering command values

	# Motion Primitives for the forward direction..............
	d_theta = 0.08
	v = 2 # Assuming a constant longitudinal velocity of 5 m/s
	delta = np.arange(-np.pi*61/180, d_theta + np.pi*61/180, d_theta)
	print("Number of discretizations for forward motion: {}".format(len(delta)))
	for d in delta:
		x = [x0]
		y = [y0]
		theta = [theta0]
		for i in range(num_steps):
			x.append(x[-1] + v*np.cos(theta[-1])*dt)
			y.append(y[-1] + v*np.sin(theta[-1])*dt)
			theta.append(theta[-1] + v*np.tan(d)*dt/L)
		plt.plot(x,y,'black')

	#  Motion Primitives for the backward direction..............
	d_theta = 0.1
	v = -1.5
	delta = np.arange(-np.pi*61/180, np.pi*61/180 + d_theta, d_theta)
	print("Number of discretizations for backward motion: {}".format(len(delta)))
	for d in delta:
		x = [x0]
		y = [y0]
		theta = [theta0]
		for i in range(num_steps):
			x.append(x[-1] + v*np.cos(theta[-1])*dt)
			y.append(y[-1] + v*np.sin(theta[-1])*dt)
			theta.append(theta[-1] + v*np.tan(d)/dt)
		plt.plot(x,y,'green')

	plt.show()
	print("Motion Primitives Plotted")

if __name__=='__main__':
	traj_roll_out(5,5,0)