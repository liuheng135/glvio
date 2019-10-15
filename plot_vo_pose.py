import numpy as np 
import matplotlib.pyplot as plt

loaded_data = np.loadtxt("../result/pose.csv",dtype = np.str,delimiter=",")

pos_x = loaded_data[:,5].astype(np.float)
pos_y = loaded_data[:,6].astype(np.float)
pos_z = loaded_data[:,7].astype(np.float)

for i in range(0,len(pos_x)):
    if pos_x[i] > 100 :
        pos_x[i] = 0.0
    elif pos_x[i] < -100:
        pos_x[i] = 0.0

plt.plot(pos_x)
plt.plot(pos_y)
plt.show()
