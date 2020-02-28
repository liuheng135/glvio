import numpy as np 
import matplotlib.pyplot as plt

loaded_data = np.loadtxt("./vio-log",dtype = np.str,delimiter=",")

imu_roll  = loaded_data[:,0].astype(np.float)
imu_pitch = loaded_data[:,1].astype(np.float)

vicon_roll  = loaded_data[:,2].astype(np.float)
vicon_pitch = loaded_data[:,3].astype(np.float)

plt.plot(imu_roll,'r')
plt.plot(imu_pitch,'g')

plt.plot(vicon_roll,'b')
plt.plot(vicon_pitch,'y')

plt.show()
