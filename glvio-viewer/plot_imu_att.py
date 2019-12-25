import numpy as np 
import matplotlib.pyplot as plt

loaded_data = np.loadtxt("../result/imu_att.csv",dtype = np.str,delimiter=",")

imu_roll  = loaded_data[:,0].astype(np.float)
imu_pitch = loaded_data[:,1].astype(np.float)
imu_yaw   = loaded_data[:,2].astype(np.float)

vicon_roll  = loaded_data[:,3].astype(np.float)
vicon_pitch = loaded_data[:,4].astype(np.float)
vicon_yaw   = loaded_data[:,5].astype(np.float)

plt.plot(imu_roll,'r')
plt.plot(imu_pitch,'g')
plt.plot(imu_yaw,'b')

plt.figure()
plt.plot(vicon_roll,'r')
plt.plot(vicon_pitch,'g')
plt.plot(vicon_yaw,'b')

plt.show()
