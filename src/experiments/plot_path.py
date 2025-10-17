import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

from src.simulation.functions import path, path_p, path_pp, TJpitch
from src.utility.configs import viviani_type


#TODO: change to path dependant on R, r and d like in paper
# def path(p):
#     x = 53 + 17.7 * (math.cos(p) - math.sin(p))
#     y = 86.6 * math.sin(p/2)
#     z = 53 + 17.7 * (math.cos(p) + math.sin(p))
#     return np.array([x, y, z])

# def path_p(p):
#     x_p = 17.7 * (-math.sin(p) - math.cos(p))
#     y_p = 43.3 * math.cos(p/2)
#     z_p = 17.7 * (math.cos(p) - math.sin(p))
#     return np.array([x_p, y_p, z_p])

# def path_pp(p):
#     x_pp = 17.7 * (-math.cos(p) + math.sin(p))
#     y_pp = -21.65 * math.sin(p/2)
#     z_pp = 17.7 * (-math.sin(p) - math.cos(p))
#     return np.array([x_pp, y_pp, z_pp])


# Store path points
ps = np.arange(0, 4*math.pi, 0.02)
coords = []
x = []
y = []
z = []
for p in ps:
    coord = path(p)
    coords.append(coord)
    x.append(coord[0])
    y.append(coord[1])
    z.append(coord[2])

    
v_current_i = [2,0,0] # wind in x direction

# Plot path
fig = plt.figure(figsize=(7,7))
ax = plt.axes(projection = '3d')
ax.set_box_aspect([1, 2, 1])

# p = 2.15*math.pi
p = 3.4
r_show = path(p)
rp_show = path_p(p)
rpp_show = path_pp(p)
print(rp_show)

# TODO: pre calculate and use as functions
# Stability basis
e1 = rp_show / np.linalg.norm(rp_show)
e3 = -r_show / np.linalg.norm(r_show)
e2 = np.linalg.cross(e3,e1) #TODO or reverse?
e2 /= np.linalg.norm(e2) # Does it need to be normalized?

ax.plot(x, y, z)
# show point
ax.plot(r_show[0], r_show[1], r_show[2], 'ko')
# tangent at show point
# ax.quiver(r_show[0], r_show[1], r_show[2], rp_show[0], rp_show[1], rp_show[2], length=0.5, color='r')

# path basis
ehat_length_ratio = 15
ax.quiver(r_show[0], r_show[1], r_show[2], e1[0], e1[1], e1[2], length=ehat_length_ratio, color='r')
ax.text(r_show[0]+e1[0]*ehat_length_ratio-2, r_show[1]+e1[1]*ehat_length_ratio, r_show[2]+e1[2]*ehat_length_ratio+4, r"$\hat{e}_1$", color='r')
ax.quiver(r_show[0], r_show[1], r_show[2], e2[0], e2[1], e2[2], length=ehat_length_ratio, color='g')
ax.text(r_show[0]+e2[0]*ehat_length_ratio-2, r_show[1]+e2[1]*ehat_length_ratio, r_show[2]+e2[2]*ehat_length_ratio+1, r"$\hat{e}_2$", color='g')
ax.quiver(r_show[0], r_show[1], r_show[2], e3[0], e3[1], e3[2], length=ehat_length_ratio, color='b')
ax.text(r_show[0]+e3[0]*ehat_length_ratio+2, r_show[1]+e3[1]*ehat_length_ratio, r_show[2]+e3[2]*ehat_length_ratio-6, r"$\hat{e}_3$", color='b')

# inertial basis
inertial_length_ratio = 60
ex = np.array([1,0,0])
ey = np.array([0,1,0])
ez = np.array([0,0,1])
a_r = 0.07
ax.quiver(0, 0, 0, ex[0], ex[1], ex[2], length=inertial_length_ratio, color='r', linewidth=0.7, arrow_length_ratio=a_r)
ax.text(0+ex[0]*inertial_length_ratio+4, 0+ex[1]*inertial_length_ratio, 0+ex[2]*inertial_length_ratio, r"$x$", color='r')
ax.quiver(0, 0, 0, ey[0], ey[1], ey[2], length=inertial_length_ratio, color='g', linewidth=0.7, arrow_length_ratio=a_r)
ax.text(0+ey[0]*inertial_length_ratio, 0+ey[1]*inertial_length_ratio+4, 0+ey[2]*inertial_length_ratio, r"$y$", color='g')
ax.quiver(0, 0, 0, ez[0], ez[1], ez[2], length=inertial_length_ratio, color='b', linewidth=0.7, arrow_length_ratio=a_r)
ax.text(0+ez[0]*inertial_length_ratio, 0+ez[1]*inertial_length_ratio, 0+ez[2]*inertial_length_ratio+4, r"$z$", color='b')
ax.set_axis_off() # turn off the plt axis

# current direction
c_length_ratio = 15
ax.quiver(0,-50,0, v_current_i[0], v_current_i[1], v_current_i[2], length=c_length_ratio, color='k', arrow_length_ratio=0.15)
ax.text(0+v_current_i[0]*c_length_ratio+4, -50+v_current_i[1]*c_length_ratio, 0+v_current_i[2]*c_length_ratio-3, r"$\vec{v}_{v_current_i}^{(i)}$", color='k')

# relative to kite
# k_speed_ratio = 1
# ax.quiver(r_show[0], r_show[1], r_show[2], v_current_i[0]-rp_show[0]*k_speed_ratio, v_current_i[1]-rp_show[1]*k_speed_ratio, v_current_i[2]-rp_show[2]*k_speed_ratio, length=1, color='c', arrow_length_ratio=0.15)

# tether 
ax.plot([0, r_show[0]], [0, r_show[1]], [0, r_show[2]], 'k--', linewidth=0.5)

# forces
# F_g 
F_g = np.array([0,0,-20])
# ax.quiver(r_show[0], r_show[1], r_show[2], F_g[0], F_g[1], F_g[2], length=1, color='m', arrow_length_ratio=0.15)


ax.set_xlim(0, 100)
ax.set_ylim(-100, 100)
ax.set_zlim(0, 100)

# ax.set_xlabel("X")
# ax.set_ylabel("Y")
# ax.set_zlabel("Z")
# plt.ion()
# plt.title("Kite path")
ax.view_init(elev=25, azim=-55, roll=0)
ax.set_title("Kite path using the " + viviani_type + " Viviani path.")
plt.tight_layout()


# plt.savefig("3d_view(noforces).png", dpi=300, bbox_inches='tight', pad_inches=0)
# plt.show()

### 2D path views ###
fig, ax = plt.subplots(2,1, figsize=(10,7))

ax[0].plot(y,z)
ax[0].set_title(r"Path in $y-z$ plane")
ax[0].set_xlabel(r"$y$ [m]")
ax[0].set_ylabel(r"$z$ [m]")
ax[0].grid()

ax[1].plot(y,x)
ax[1].set_title(r"Path in $y-x$ plane")
ax[1].set_xlabel(r"$y$ [m]")
ax[1].set_ylabel(r"$x$ [m]")
ax[1].grid()

fig.tight_layout()





### p dependant added alpha

alphas_kite = [TJpitch(p) for p in ps]

# print(alpha_kite)

fig, ax = plt.subplots(2,1, figsize=(8,8))
ax[0].plot(ps, alphas_kite)
ax[0].set_title("Kite-tether angle")
ax[0].set_xlabel("p")
ax[0].set_ylabel(r"TJpitchAngle")
ax[0].grid()

ax[1].plot(ps, z)
ax[1].set_title(r"Height $z$")
ax[1].set_xlabel("p")
ax[1].set_ylabel("z")
ax[1].grid()

plt.show()
