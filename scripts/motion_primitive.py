import numpy as np
from math import cos, sin, tan
import matplotlib.pyplot as plt

class motion():
	def __init__(self, x=0, y=0, theta=0, n=8, delt=0.1):
		self.x0 = x
		self.y0 = y
		self.theta0 = theta
		self.dt = delt 		# Simulation fixed time-step 
		self.num_steps = n  # Number of steps
		self.L = 3.0	# vehcile wheel-base length
		self.max_steer = 61		# maximium steering angle (radians)
		self.motion_primitives = []

	def generate_motion_patters(self):
		"""
		Function to pre-compute the set of motion pattersn from the initial state (0,0,0).
		Motion Patterns computed using linear interpolation.
		Based on Kinematic Bicycle model.
		"""

		# Motion primimtives for the forward direction.....................
		d_del = 0.08	
		dt = self.dt
		v = 2	# Assuming a constant longitudinal velocity
		delta = np.arange(-np.pi*self.max_steer/180, d_del + np.pi*self.max_steer/180, d_del)
		print("Number of motion patterns in forward directon: {}".format(len(delta)))
		for d in delta:
			x0 = self.x0
			y0 = self.y0
			theta0 = self.theta0
			p = np.array([x0, y0, theta0])
			
			for i in range(self.num_steps):
				x0 += v*cos(theta0)*dt
				y0 += v*sin(theta0)*dt
				theta0 += v*tan(d)*dt/self.L
				p = np.vstack((p,np.array([x0, y0, theta0])))

			# Adding the motion primitive array to the list
			self.motion_primitives.append(p)

		
		# Motion primitives for the backward direction ...................
		d_del = 0.1
		v = -1.2
		delta = np.arange(-np.pi*self.max_steer/180, d_del + np.pi*self.max_steer/180, d_del)
		print("Number of motion patterns for the backward direction: {}".format(len(delta)))
		for d in delta:
			x0 = self.x0
			y0 = self.y0
			theta0 = self.theta0
			p = np.array([x0, y0, theta0])

			for i in range(self.num_steps):
				x0 += v*cos(theta0)*dt
				y0 += v*sin(theta0)*dt
				theta0 += v*tan(d)*dt/self.L
				p=np.vstack((p, np.array([x0, y0, theta0])))
			# Adding the motion primitive array to the list
			self.motion_primitives.append(p)
		

	def rotate_pattern(self, st2):
		"""
		Function to translate and rotate the motion pattern by theta
		"""
		# dx = st2[0] - self.x0
		# dy = st2[1] - self.y0
		dtheta = st2[2] - self.theta0
		transformT = np.array([[cos(dtheta), -sin(dtheta), self.x0],[sin(dtheta), cos(dtheta), self.y0],[0,0,1]])
		n_p = []
		p0 = self.motion_primitives
		for i in range(len(p0)):
			p0_i = p0[i]
			xyp = np.hstack((p0_i[:,0].reshape(-1,1)-self.x0, p0_i[:,1].reshape(-1,1)-self.y0, np.ones(p0_i.shape[0]).reshape(-1,1))).T
			# xyp = np.hstack((p0_i[:,:2], np.ones(p0_i.shape[0]).reshape(-1,1))).T
			n_xyp = np.dot(transformT, xyp)
			n_theta = p0_i[:,-1]+dtheta
			nn_xyp = np.vstack((n_xyp[0,:].reshape(1,-1)+st2[0]-self.x0, n_xyp[1,:].reshape(1,-1)+st2[1]-self.y0))
			p2 = np.hstack((nn_xyp.T, n_theta.reshape(-1,1)))
			n_p.append(p2)
		return n_p


	def get_motion_patterns(self, state):
		"""
		Function to transform the set of motion patterns to the current state parametrs.
		"""
		patterns = self.motion_primitives
		n_p=[]
		for i in range(len(patterns)):
			p=patterns[i]
			p[:,0] += state[0]
			p[:,1] += state[1]
			n_p.append(p)

		return n_p


	def plot_motion_patterns(self, patterns, st_0, cl = 'black'):
		"""
		Function to plot the motion patterns
		"""
		plt.scatter(st_0[0], st_0[1], s = 16)
		for i in range(len(patterns)):
			plt.plot(patterns[i][:,0], patterns[i][:,1], cl)
			# print(patterns[i])
			# print('________________________________')


if __name__ == '__main__':
	mp = motion(15,15,np.pi/2)
	mp.generate_motion_patters()
	motion_patterns = mp.motion_primitives
	mp.plot_motion_patterns(motion_patterns, [15,15,np.pi/2], 'red')
	st2=[8,8, np.pi/4]
	pattern2 = mp.rotate_pattern(st2)
	mp.plot_motion_patterns(pattern2, st2, 'green')
	plt.show()