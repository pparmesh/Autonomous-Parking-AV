import numpy as np
from math import floor, ceil, sin, cos
import matplotlib.pyplot as plt


def wrap2pi(a):
    if(a>=3.14):
    	a=0
    return a

class Map():
	def __init__(self):
		self.w = 2.7572021484375
		self.l = 5.142044059999996
		self.xlim = [-62,30]
		self.ylim = [-40, 40]
		self.spawn_points = np.load('Parking1.npy')
		# Converting RHS to LHS
		self.spawn_points[:,1]=-self.spawn_points[:,1]
		self.spawn_points[:,2]=-self.spawn_points[:,2]


	def plot_parking(self):
		w=self.w
		l=self.l

		a=self.spawn_points
		x=a[:,0]
		y=-a[:,1]
		theta=-a[:,2]
		plt.scatter(x,y,s=2)
		for i in range(len(a)):
			ang=theta[i]*np.pi/180
			X=[x[i]+(l/2)*np.cos(ang)+(w/2)*np.sin(ang),x[i]-(l/2)*np.cos(ang)-(w/2)*np.sin(ang),
			x[i]-(l/2)*np.cos(ang)-(w/2)*np.sin(ang),x[i]+(l/2)*np.cos(ang)+(w/2)*np.sin(ang)]
			Y=[y[i]+(w/2)*np.cos(ang)+(l/2)*np.sin(ang),y[i]+(w/2)*np.cos(ang)+(l/2)*np.sin(ang),y[i]-(w/2)*np.cos(ang)-(l/2)*np.sin(ang),y[i]-(w/2)*np.cos(ang)-(l/2)*np.sin(ang)]
			plt.plot(X,Y,'b')

		plt.show()

	def xy2i(self, px, py):
		"""
		Function to convert the parking lot vertices px,py to occupancy grid indices
		"""
		ax = floor((px[0]-self.xlim[0])/self.dx)
		bx = ceil((px[1]-self.xlim[0])/self.dx)
		ay = floor((py[0]-self.ylim[0])/self.dy)
		by = ceil((py[1]-self.ylim[0])/self.dy)
		return np.meshgrid(np.arange(ax,bx+1), np.arange(ay,by+1))

	def occupancy_grid(self, dx=0.2, dy=0.2):
		"""
		Function to compute the obstacle grid from the queried spawn_points.
		Spawn Points queried from the Carla Map "Parking1"
		"""
		w = self.w
		l = self.l
		self.dx = dx
		self.dy = dy
		spts = self.spawn_points
		cp=1;

		nx, ny = int((self.xlim[1]-self.xlim[0])/dx), int((self.ylim[1]-self.ylim[0])/dy)
		print('Size of the parking lot: {} x {}'.format(nx,ny))
		
		# Initializing a 2D array for the occupancy grid with all zeros
		occ_grid=np.zeros((nx,ny))
		
		# Randomly sampling the empty parking lots
		n=5		# number of parking spots to leave empty
		empty_lot=np.random.choice(np.arange(spts.shape[0]), n)
		print(empty_lot)
		for i in range(len(spts)):
			#  Computing the parking bounding boxes
			if i in empty_lot:
				cp=5
			else:
				cp=20
				# continue
			ang = wrap2pi(abs(spts[i,2]*np.pi/180))
			# ang = spts[i,2]*np.pi/180

			pl_x = [spts[i,0]-(l/2)*np.cos(ang)-(w/2)*np.sin(ang),spts[i,0]+(l/2)*np.cos(ang)+(w/2)*np.sin(ang)]
			pl_y = [spts[i,1]-(w/2)*np.cos(ang)-(l/2)*np.sin(ang), spts[i,1]+(w/2)*np.cos(ang)+(l/2)*np.sin(ang)]
			# print(i, pl_x, pl_y, ang)
			# Computing the occupancy grid indices
			occX, occY = self.xy2i(pl_x, pl_y)
			occ_grid[occX, occY]=cp

		plt.imshow(occ_grid.T, "Greys")
		plt.show()

if __name__=='__main__':
	map1=Map()
	map1.plot_parking()
	map1.occupancy_grid()
	# p=map1.spawn_points[2]
	# print(p)
	# l=map1.l
	# w=map1.w
	# ang=p[-1]*np.pi/180
	# x1, x2 = p[0]-(l/2)*np.cos(ang)-(w/2)*np.sin(ang),p[0]+(l/2)*np.cos(ang)+(w/2)*np.sin(ang)
	# y1, y2 =p[1]-(w/2)*np.cos(ang)-(l/2)*np.sin(ang), p[1]+(w/2)*np.cos(ang)+(l/2)*np.sin(ang)
	# print(x1,y1)
	# print(x2,y2)
	# dd=map1.xy2i([x1,x2], [y1,y2])
	# print(dd)