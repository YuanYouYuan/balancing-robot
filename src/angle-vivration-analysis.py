import matplotlib.pyplot as plt
import numpy as np

file_path = '../data/vibration-0427.csv'

#angle, ax, az, gy, dt
data = np.genfromtxt(file_path, delimiter=',', skip_header=1)
data_length = len(data)
kalman_angle = data[:, 0];
ax = data[:, 1]
az = data[:, 2]
gy = data[:, 3]
dt = data[:, 4]

time = np.cumsum(dt)
acce_angle = (180/np.pi) * np.arctan2(az, ax) - 90.0;
angle_rate = (gy - 124.0) / 131.0
gyro_angle = np.cumsum(angle_rate * dt);


sample_number = 30
mean_angle = np.zeros((data_length, 1))

for i in range(data_length - sample_number):
    mean_angle[i] = np.mean(kalman_angle[i:i+sample_number])

compl_angle = np.zeros((data_length, 1))
compl_k = 0.98
for i in range(1, data_length):
    compl_angle[i] = compl_k * (compl_angle[i-1] + angle_rate[i] * dt[i]) + (1 - compl_k) * acce_angle[i]



fig, axes = plt.subplots(5, 1)
axes[0].plot(time, kalman_angle)
axes[0].set_ylabel('kalman')
axes[1].plot(time, acce_angle)
axes[1].set_ylabel('acce angle')
axes[2].plot(time, gyro_angle)
axes[2].set_ylabel('gyro angle')
axes[3].plot(time, mean_angle)
axes[3].set_ylabel('mean angle')
axes[4].plot(time, compl_angle)
axes[4].set_ylabel('compl angle')
plt.show()




