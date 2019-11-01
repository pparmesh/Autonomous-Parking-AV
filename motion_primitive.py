import numpy as np
import matplotlib.pyplot as plt

def traj_roll_out(x0, y0, theta0, num_steps=8, dt=0.1):
	"""
	Function to compute the trajectory from given (x0,yo,theta0).
	Using linear interpolation, based on Kineamtic Bicycle model.
	"""
	v=5 # Assuming a constant longitudinal velocity of 5 m/s
	L=2.2	# Longitudinal length of 2.2 m
	# Defining a range of discretized steering command values
	delta=np.arange(-np.pi/4, np.pi/4, 0.05)
	for d in delta:
		x=[x0]
		y=[y0]
		theta=[theta0]
		for i in range(num_steps):
			x.append(x[-1]+v*np.cos(theta[-1])*dt)
			y.append(y[-1]+v*np.sin(theta[-1])*dt)
			theta.append(theta[-1]+v*np.tan(d)*dt/L)
		plt.plot(x,y,'black')
	plt.show()
	print("Motion Primitives Plotted")

if __name__=='__main__':
	traj_roll_out(5,5,0)