import matplotlib.pyplot as plt

fig, axes = plt.subplots(3, 1)
for i in range(3):
    axes[i].plot([1, 2, 3], [1*i, 2*i, 3*i])
plt.show()
