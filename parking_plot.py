import numpy as np
import matplotlib.pyplot as plt


def plot_parking():
	w=2.7572021484375
	l=5.142044059999996

	a=np.load('Parking1.npy')
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

if __name__=='__main__':
	plot_parking()
