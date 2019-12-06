import numpy as np
from math import floor, ceil, sin, cos
import matplotlib.pyplot as plt
from motion_primitive import *

def wrap2pi(a):
    if(a>=3.14):
    	a=0
    return a

def wraptopi(a):
	a = a%(2*np.pi)
	if a<0:
		a+=2*np.pi
	return a

class Map():
	def __init__(self, ddx, ddy):
		self.w = 2.7572021484375
		self.lb = 5.142044059999996/2
		self.lf = 6-self.lb
		self.xlim = [-62,30]
		self.ylim = [-40, 40]
		self.dx = ddx
		self.dy = ddy
		self.spawn_points = np.load('Parking1.npy')
		# Converting RHS to LHS
		self.spawn_points[:,1]=-self.spawn_points[:,1]
		self.spawn_points[:,2]=-self.spawn_points[:,2]


	def plot_parking(self):
		w = self.w
		lf = self.lf
		lb = self.lb

		a = self.spawn_points
		x = a[:,0]
		y = a[:,1]
		theta = a[:,2]
		plt.scatter(x,y,s=2)
		for i in range(len(a)):
			ang = wraptopi(np.pi*theta[i]/180)
			# print(theta[i], ang)
			a = [x[i]+lf*cos(ang)+(w/2)*sin(ang), y[i]+lf*sin(ang)-(w/2)*cos(ang)]
			b = [x[i]+lf*cos(ang)-(w/2)*sin(ang), y[i]+lf*sin(ang)+(w/2)*cos(ang)]
			c = [x[i]-lb*cos(ang)-(w/2)*sin(ang), y[i]-lb*sin(ang)+(w/2)*cos(ang)]
			d = [x[i]-lb*cos(ang)+(w/2)*sin(ang), y[i]-lb*sin(ang)-(w/2)*cos(ang)]
			plt.plot([b[0], c[0], d[0], a[0]], [b[1], c[1], d[1], a[1]],'b')

	def xy2i(self, px, py):
		"""
		Function to convert the parking lot vertices px,py to occupancy grid indices
		"""
		ax = floor((px[0]-self.xlim[0])/self.dx)
		bx = ceil((px[1]-self.xlim[0])/self.dx)
		ay = floor((py[0]-self.ylim[0])/self.dy)
		by = ceil((py[1]-self.ylim[0])/self.dy)
		return np.meshgrid(np.arange(ax,bx+1), np.arange(ay,by+1))

	def occupancy_grid(self):
		"""
		Function to compute the obstacle grid from the queried spawn_points.
		Spawn Points queried from the Carla Map "Parking1"
		"""
		w = self.w
		lf = self.lf
		lb = self.lb
		dx = self.dx
		dy = self.dy
		spts = self.spawn_points
		x = spts[:,0]
		y = spts[:,1]
		cp=1;

		nx, ny = int((self.xlim[1]-self.xlim[0])/dx), int((self.ylim[1]-self.ylim[0])/dy)
		print('Size of the parking lot: {} x {}'.format(nx,ny))
		
		# Initializing a 2D array for the occupancy grid with all zeros
		occ_grid=np.zeros((nx,ny))
		
		# Randomly sampling the empty parking lots
		n=5		# number of parking spots to leave empty
		empty_lot=[59, 48, 39, 44, 10, 70] #np.random.choice(np.arange(spts.shape[0]), n)
		print(empty_lot)
		for i in range(len(spts)):
			#  Computing the parking bounding boxes
			if i in empty_lot:
				cp=5
			else:
				cp=20
				# continue
			# ang = wrap2pi(abs(spts[i,2]*np.pi/180))
			ang = wraptopi(np.pi*spts[i,2]/180)
			# ang = spts[i,2]*np.pi/180
			a = [x[i]+lf*cos(ang)+(w/2)*sin(ang), y[i]+lf*sin(ang)-(w/2)*cos(ang)]
			b = [x[i]+lf*cos(ang)-(w/2)*sin(ang), y[i]+lf*sin(ang)+(w/2)*cos(ang)]
			c = [x[i]-lb*cos(ang)-(w/2)*sin(ang), y[i]-lb*sin(ang)+(w/2)*cos(ang)]
			d = [x[i]-lb*cos(ang)+(w/2)*sin(ang), y[i]-lb*sin(ang)-(w/2)*cos(ang)]
			# a = [x[i]+lf*cos(ang)+(w/2)*sin(ang), x[i]+lf*cos(ang)-(w/2)*sin(ang),
			# x[i]-lb*cos(ang)-(w/2)*sin(ang), x[i]-lb*cos(ang)+(w/2)*sin(ang)]

			# b = [y[i]+lf*sin(ang)-(w/2)*cos(ang), y[i]+lf*sin(ang)+(w/2)*cos(ang),
			#  y[i]-lb*sin(ang)+(w/2)*cos(ang), y[i]-lb*sin(ang)-(w/2)*cos(ang)]

			# pl_x = [spts[i,0]-lb*np.cos(ang)-(w/2)*np.sin(ang),spts[i,0]+lf*np.cos(ang)+(w/2)*np.sin(ang)]
			# pl_y = [spts[i,1]-(w/2)*np.cos(ang)-lb*np.sin(ang), spts[i,1]+(w/2)*np.cos(ang)+lf*np.sin(ang)]
			if(ang == 0):
				pl_x = [c[0], b[0]]
				pl_y = [d[1], c[1]]
			elif (ang <= (0.1+np.pi/2)):
				pl_x = [b[0], a[0]]
				pl_y = [d[1], a[1]]
			elif ang <= (0.1+np.pi):
				pl_x = [b[0], c[0]]
				pl_y = [b[1], a[1]]
			else:
				pl_x = [a[0], b[0]]
				pl_y = [a[1], d[1]]


			# Computing the occupancy grid indices
			occX, occY = self.xy2i(pl_x, pl_y)
			occ_grid[occX, occY]=cp

		plt.imshow(occ_grid.T, "Greys", origin='lower')
		plt.show()


	def compute_occupancy_grid(self, empty_ind):
		"""
		Function to compute the obstacle grid from the queried spawn_points.
		Spawn Points queried from the Carla Map "Parking1"
		"""
		w = self.w
		lf = self.lf
		lb = self.lb
		dx = self.dx
		dy = self.dy
		spts = self.spawn_points
		x = spts[:,0]
		y = spts[:,1]
		cp=1;

		nx, ny = int((self.xlim[1]-self.xlim[0])/dx), int((self.ylim[1]-self.ylim[0])/dy)
		print('Size of the parking lot: {} x {}'.format(nx,ny))
		
		# Initializing a 2D array for the occupancy grid with all zeros
		occ_grid=np.zeros((nx,ny))
		
		# Randomly sampling the empty parking lots
		n=5		# number of parking spots to leave empty
		empty_lot= empty_ind #np.random.choice(np.arange(spts.shape[0]), n)
		print(empty_lot)
		for i in range(len(spts)):
			#  Computing the parking bounding boxes
			if i in empty_lot:
				cp=5
			else:
				cp=20
				# continue
			ang = wraptopi(np.pi*spts[i,2]/180)
			a = [x[i]+lf*cos(ang)+(w/2)*sin(ang), y[i]+lf*sin(ang)-(w/2)*cos(ang)]
			b = [x[i]+lf*cos(ang)-(w/2)*sin(ang), y[i]+lf*sin(ang)+(w/2)*cos(ang)]
			c = [x[i]-lb*cos(ang)-(w/2)*sin(ang), y[i]-lb*sin(ang)+(w/2)*cos(ang)]
			d = [x[i]-lb*cos(ang)+(w/2)*sin(ang), y[i]-lb*sin(ang)-(w/2)*cos(ang)]
			if(ang == 0):
				pl_x = [c[0], b[0]]
				pl_y = [d[1], c[1]]
			elif (ang <= (0.1+np.pi/2)):
				pl_x = [b[0], a[0]]
				pl_y = [d[1], a[1]]
			elif ang <= (0.1+np.pi):
				pl_x = [b[0], c[0]]
				pl_y = [b[1], a[1]]
			else:
				pl_x = [a[0], b[0]]
				pl_y = [a[1], d[1]]
			occX, occY = self.xy2i(pl_x, pl_y)
			occ_grid[occX, occY]=cp
		# plt.imshow(occ_grid.T, "Greys")
		return occ_grid

	def compute_swath(self):
		l = self.l/2
		w = self.w/2
		dx = self.dx
		dy = self.dy
		mp = motion(0, 0, 0)
		mp.generate_motion_patters()
		pat = mp.motion_primitives[0]

		start = [0, 0, 0]
		nx, ny = int((self.xlim[1]-self.xlim[0])/dx), int((self.ylim[1]-self.ylim[0])/dy)
		occ_grid = np.zeros((nx, ny))

		#  Computing the footprint of the vehicle
		ang = wraptopi(np.pi*start[0]/180)
		pl_x = [start[0]-l*cos(ang)-w*sin(ang), start[0]+l*cos(ang)+w*sin(ang)]
		pl_y = [start[1]-w*cos(ang)-l*sin(ang), start[1]+w*cos(ang)+l*sin(ang)]
		X, Y = self.xy2i(pl_x, pl_y)
		self.footprint = np.hstack((X.reshape(-1,1), Y.reshape(-1,1)))
		cp = 25
		for i in range(len(pat)):
			fpt = self.transform_swath(pat[i,:])
			occ_grid[fpt[:,0], fpt[:,1]] = cp
			cp=cp-2	
		occ_grid[self.footprint[:,0], self.footprint[:,1]] =1
		plt.imshow(occ_grid.T, "Greys", origin='lower')
		plt.show()

	def transform_swath(self, st2):
		"""
		Funciton to transform(rotate and translate) the vehicle footprint
		"""
		x0 = floor((0-self.xlim[0])/self.dx)
		y0 = floor((0-self.ylim[0])/self.dy)
		x1 = floor((st2[0]-self.xlim[0])/self.dx)
		y1 = floor((st2[1]-self.ylim[0])/self.dy)
		theta = wraptopi(st2[2])
		print(theta)

		transformT = np.array([[cos(theta), -sin(theta), x0], [sin(theta), cos(theta), y0], [0, 0, 1]])
		p0 = self.footprint
		p0[:,0] -= x0
		p0[:,1] -= y0
		xyp = np.hstack((p0, np.ones((p0.shape[0],1)))).T

		n_xyp = np.dot(transformT, xyp)
		n_xyp = n_xyp.T
		n_xyp[:,0] += x1 -x0
		n_xyp[:,1] += y1 -y0
		return n_xyp[:,:2].astype(int)





if __name__=='__main__':
	map1=Map(0.2, 0.2)
	map1.plot_parking()
	plt.show()
	# map1.compute_swath()
	map1.occupancy_grid()
	
	# ddc = map1.compute_occupancy_grid([59, 48, 39, 44, 10, 70])
	# plt.imshow(ddc.T, 'Greys' , origin='lower')
	# plt.show()