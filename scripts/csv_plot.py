import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


if __name__ =='__main__':
	ss = pd.read_csv("startS.csv").values
	sg = pd.read_csv("goalS.csv").values

	startS = [-44.81,-31.04]
	goalS = [-13.5, -31.04]
	# startS = [-5, 15]
	# goalS = [15,15]
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

	plt.show()
