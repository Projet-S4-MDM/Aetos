import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

size = 10  

beam_thickness = 0.2
beam_height = 0.2
square_size = 10  
vertical_height = 10  


radius = 2
center_x, center_y, center_z = 0, 0, 5
num_frames = 100  


beams = [
    (-square_size / 2, -square_size / 2, 0, square_size, beam_thickness, beam_height),  
    (-square_size / 2, square_size / 2 - beam_thickness, 0, square_size, beam_thickness, beam_height),  
    (-square_size / 2, -square_size / 2, 0, beam_thickness, square_size, beam_height),  
    (square_size / 2 - beam_thickness, -square_size / 2, 0, beam_thickness, square_size, beam_height),  
]


vertical_beams = [
    (-square_size / 2, -square_size / 2, 0, beam_thickness, beam_thickness, vertical_height),  
    (-square_size / 2, square_size / 2 - beam_thickness, 0, beam_thickness, beam_thickness, vertical_height),  
    (square_size / 2 - beam_thickness, -square_size / 2, 0, beam_thickness, beam_thickness, vertical_height),  
    (square_size / 2 - beam_thickness, square_size / 2 - beam_thickness, 0, beam_thickness, beam_thickness, vertical_height),  
]

anchor_points = [
    (-square_size / 2, -square_size / 2, vertical_height),  
    (-square_size / 2, square_size / 2 - beam_thickness, vertical_height),  
    (square_size / 2 - beam_thickness, -square_size / 2, vertical_height),  
    (square_size / 2 - beam_thickness, square_size / 2 - beam_thickness, vertical_height),  
]


for x, y, z, dx, dy, dz in beams:
    ax.bar3d(x, y, z, dx, dy, dz, color='black')

for x, y, z, dx, dy, dz in vertical_beams:
    ax.bar3d(x, y, z, dx, dy, dz, color='black')

effector, = ax.plot([0], [0], [5], 'ro', markersize=8)  
cables = [ax.plot([], [], [], 'gray', linestyle='-', linewidth=1)[0] for _ in range(4)]  


def update(frame):
    theta = 2 * np.pi * frame / num_frames  
    effector_x = center_x + radius * np.cos(theta)
    effector_y = center_y + radius * np.sin(theta)
    effector_z = center_z  

    
    effector.set_data([effector_x], [effector_y])
    effector.set_3d_properties([effector_z])  

    
    for i, (x, y, z) in enumerate(anchor_points):
        cables[i].set_data([x, effector_x], [y, effector_y])
        cables[i].set_3d_properties([z, effector_z])  

    return [effector] + cables

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_xlim([-size, size])
ax.set_ylim([-size, size])
ax.set_zlim([0, size])


ani = animation.FuncAnimation(fig, update, frames=num_frames, interval=50, blit=False)

plt.show()
