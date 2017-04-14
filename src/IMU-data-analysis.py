import matplotlib.pyplot as plt
import numpy as np
file_path = '../data/IMU-data.csv'

with open(file_path) as f:
    lines = f.readlines()
    data = [[float(val) for val in line.rstrip().split(',')] for line in lines]

data = np.array(data)
data = data[:1000, :]
data_name = [r"$a_x$", r"$a_y$", r"$a_z$", r"$g_x$", r"$g_y$", r"$g_z$", r"$\theta_a$", r"$\theta_g$"]
data_linestyle = ['-', '-', '-', '-', '-', '-', '--', '--']

plt.subplot(211)
for i in range(3):
    plt.plot(data[:, i], label=data_name[i], linestyle=data_linestyle[i])
plt.legend(loc=1)
    
plt.subplot(212)
for i in range(3,6):
    plt.plot(data[:, i], label=data_name[i], linestyle=data_linestyle[i])
plt.legend(loc=1)

gy_mean = np.mean(data[:, 4])


print(gy_mean)




plt.tight_layout()
plt.ylabel('value')
plt.xlabel('samples')
plt.show()


