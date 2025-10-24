import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

from src.utility.configs import R_ei

fig = plt.figure(figsize=(7,7))
ax = plt.axes(projection = '3d')
ax.set_box_aspect([1, 1, 1])

# earth basis
length_ratio = 10
xe = np.array([1,0,0])
ye = np.array([0,1,0])
ze = np.array([0,0,1])
a_r = 0.07
ax.quiver(0, 0, 0, xe[0], xe[1], xe[2], length=length_ratio, color='r', linewidth=0.7, arrow_length_ratio=a_r)
ax.text(0+xe[0]*length_ratio+1, 0+xe[1]*length_ratio, 0+xe[2]*length_ratio, r"$x^{(e)}$", color='r')
ax.quiver(0, 0, 0, ye[0], ye[1], ye[2], length=length_ratio, color='g', linewidth=0.7, arrow_length_ratio=a_r)
ax.text(0+ye[0]*length_ratio, 0+ye[1]*length_ratio+1, 0+ye[2]*length_ratio, r"$y^{(e)}$", color='g')
ax.quiver(0, 0, 0, ze[0], ze[1], ze[2], length=length_ratio, color='b', linewidth=0.7, arrow_length_ratio=a_r)
ax.text(0+ze[0]*length_ratio, 0+ze[1]*length_ratio, 0+ze[2]*length_ratio+1, r"$z^{(e)}$", color='b')
ax.set_axis_off() # turn off the plt axis


# inertial basis
xi = R_ei @ xe
yi = R_ei @ ye
zi = R_ei @ ze
a_r = 0.07
ax.quiver(0, 0, 0, xi[0], xi[1], xi[2], length=length_ratio, color='r', linewidth=0.7, arrow_length_ratio=a_r)
ax.text(0+xi[0]*length_ratio+1, 0+xi[1]*length_ratio, 0+xi[2]*length_ratio, r"$x^{(i)}$", color='r')
ax.quiver(0, 0, 0, yi[0], yi[1], yi[2], length=length_ratio, color='g', linewidth=0.7, arrow_length_ratio=a_r)
ax.text(0+yi[0]*length_ratio, 0+yi[1]*length_ratio-1, 0+yi[2]*length_ratio, r"$y^{(i)}$", color='g')
ax.quiver(0, 0, 0, zi[0], zi[1], zi[2], length=length_ratio, color='b', linewidth=0.7, arrow_length_ratio=a_r)
ax.text(0+zi[0]*length_ratio, 0+zi[1]*length_ratio, 0+zi[2]*length_ratio-1, r"$z^{(i)}$", color='b')
ax.set_axis_off() # turn off the plt axis



ax.set_xlim(-10, 10)
ax.set_ylim(-10, 10)
ax.set_zlim(-10, 10)

# ax.set_xlabel("X")
# ax.set_ylabel("Y")
# ax.set_zlabel("Z")
# plt.ion()
# plt.title("Kite path")
ax.view_init(elev=25, azim=-55, roll=0)
ax.set_title("")
plt.tight_layout()
plt.show()