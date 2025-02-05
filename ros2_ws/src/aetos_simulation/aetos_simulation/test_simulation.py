import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

size = 5
step = 1

# Draw grid lines in XY plane
for i in np.arange(-size, size + step, step):
    ax.plot([-size, size], [i, i], [0, 0], 'k', linewidth=0.5)
    ax.plot([i, i], [-size, size], [0, 0], 'k', linewidth=0.5)

# Large black beam obstacle
x_start, x_end = -2, 2  # Beam from x=-2 to x=2
y_start, y_end = 0, 0.2   # Fixed y position
z_start, height = 0, 0.05  # Beam starts at z=0 and has a height of 1

# Create a beam as a thick rectangular block
ax.bar3d(x_start, y_start, z_start, x_end - x_start, 0.5, height, color='black')

# Set labels and limits
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_xlim([-size, size])
ax.set_ylim([-size, size])
ax.set_zlim([0, size])

plt.show()
