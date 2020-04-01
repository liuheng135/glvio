import numpy as np 
import matplotlib.pyplot as plt

loaded_data = np.loadtxt("./flow_log.txt",dtype = np.str,delimiter=",")

imu_roll  = loaded_data[:,0].astype(np.float)
imu_pitch = loaded_data[:,1].astype(np.float)

plt.plot(imu_roll,'r')
plt.plot(imu_pitch,'g')

plt.show()
