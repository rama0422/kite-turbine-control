import numpy as np
import math
import sympy as sp
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

from src.utility.configs import rho, g, R_path, r_path, a_path, b_path, viviani_type, theta_path, TJpitch_amp, TJpitch_offset, TJpitch_shift,w_test,T_test,efficiency_test,w_limit,T_gen_max


ps = np.arange(0, 4*math.pi, 0.02)
v_current_i = [2,0,0] # wind in x direction


p_sym, R_sym, r_sym, theta_sym = sp.symbols('p_sym R_sym r_sym theta_sym', real=True)

####### Circular cylinder #3###
sub = (R_sym - r_sym)
x = sub * sp.cos(theta_sym) + r_sym * sp.cos(p_sym) * sp.cos(theta_sym) - r_sym * sp.sin(p_sym) * sp.sin(theta_sym)
y = 2 * sp.sqrt(sub * r_sym) * sp.sin(p_sym / 2)
z = sub * sp.sin(theta_sym) + r_sym * sp.cos(p_sym) * sp.sin(theta_sym) + r_sym * sp.sin(p_sym) * sp.cos(theta_sym)
sub = (R_sym - r_sym)
x = sub + r_sym * sp.cos(p_sym)
y = 2 * sp.sqrt(sub * r_sym) * sp.sin(p_sym/2)
z = r_sym * sp.sin(p_sym)

# Rotate
R = sp.Matrix([[sp.cos(theta_sym), 0, -sp.sin(theta_sym)],
            [0, 1, 0],
            [sp.sin(theta_sym), 0, sp.cos(theta_sym)]])
x,y,z = R @ sp.Matrix([x,y,z])

# differentiate
x_p, y_p, z_p = sp.diff(x, p_sym), sp.diff(y, p_sym), sp.diff(z, p_sym)
x_pp, y_pp, z_pp = sp.diff(x_p, p_sym), sp.diff(y_p, p_sym), sp.diff(z_p, p_sym)

path_lamb = sp.lambdify((p_sym, R_sym, r_sym, theta_sym), [x, y, z], 'numpy')
path_p_lamb = sp.lambdify((p_sym, R_sym, r_sym, theta_sym), [x_p, y_p, z_p], 'numpy')
path_pp_lamb = sp.lambdify((p_sym, R_sym, r_sym, theta_sym), [x_pp, y_pp, z_pp], 'numpy')

def path(p):
    return np.array(path_lamb(p, R_path, r_path, theta_path))

x_cyl = []
y_cyl = []
z_cyl = []
for p in ps:
    coord = path(p)
    x_cyl.append(coord[0])
    y_cyl.append(coord[1])
    z_cyl.append(coord[2])

###### Elliptic cylinder######
p_sym, R_sym, a_sym, b_sym, theta_sym = sp.symbols('p_sym R_sym a_sym b_sym theta_sym', real=True)
# u_sym = p_sym / 2
sub = (R_sym - a_sym)
x = sub + a_sym * sp.cos(p_sym)
# y = sp.sign(sp.sin(p_sym)) * sp.sqrt(R_sym**2 - (sub + a_sym * sp.cos(p_sym))**2 - b_sym**2 * sp.sin(p_sym)**2)

# y = sp.sqrt(R_sym**2 - (sub + a_sym * sp.cos(2*u_sym))**2 - b_sym**2 * sp.sin(2*u_sym)**2)

eps = 1e-8
period = 4*sp.pi
y_half = sp.sqrt(R_sym**2 - (sub + a_sym * sp.cos(p_sym))**2 - b_sym**2 * sp.sin(p_sym)**2 + eps)
y = sp.Piecewise((y_half, (p_sym % period >= 0) & (p_sym % period <= 2*sp.pi)), (-y_half, (p_sym % period > 2*sp.pi) & (p_sym % period < 4*sp.pi)))
z = b_sym * sp.sin(p_sym)

# Rotate
R = sp.Matrix([[sp.cos(theta_sym), 0, -sp.sin(theta_sym)],
            [0, 1, 0],
            [sp.sin(theta_sym), 0, sp.cos(theta_sym)]])
x,y,z = R @ sp.Matrix([x,y,z])

# differentiate
x_p, y_p, z_p = sp.diff(x, p_sym), sp.diff(y, p_sym), sp.diff(z, p_sym)
x_pp, y_pp, z_pp = sp.diff(x_p, p_sym), sp.diff(y_p, p_sym), sp.diff(z_p, p_sym)

path_lamb = sp.lambdify((p_sym, R_sym, a_sym, b_sym, theta_sym), [x, y, z], 'numpy')
path_p_lamb = sp.lambdify((p_sym, R_sym, a_sym, b_sym, theta_sym), [x_p, y_p, z_p], 'numpy')
path_pp_lamb = sp.lambdify((p_sym, R_sym, a_sym, b_sym, theta_sym), [x_pp, y_pp, z_pp], 'numpy')

# a_path = 3
# b_path = 6
def path(p):
    return np.array(path_lamb(p, R_path, a_path, b_path, theta_path))

x_ellip = []
y_ellip = []
z_ellip = []
for p in ps:
    coord = path(p)
    x_ellip.append(coord[0])
    y_ellip.append(coord[1])
    z_ellip.append(coord[2])


p = 0
r_show = path(p)

############ ploting ###############

# Plot path
fig = plt.figure(figsize=(10,10))
ax = plt.axes(projection = '3d')
ax.set_box_aspect([1, 2, 1])

# tether 
ax.plot([0, r_show[0]], [0, r_show[1]], [0, r_show[2]], 'k--', linewidth=0.8)
ax.text(16, 0, 1.5, r"$\theta$", color='k')

# inertial basis
inertial_length_ratio = 60
ex = np.array([1,0,0])
ey = np.array([0,1,0])
ez = np.array([0,0,1])
a_r = 0.03
ax.quiver(0, 0, 0, ex[0], ex[1], ex[2], length=inertial_length_ratio, color='r', linewidth=0.7, arrow_length_ratio=a_r)
ax.text(0+ex[0]*inertial_length_ratio+2, 0+ex[1]*inertial_length_ratio, 0+ex[2]*inertial_length_ratio, r"$x$", color='r')
ax.quiver(0, 0, 0, ey[0], ey[1], ey[2], length=inertial_length_ratio, color='g', linewidth=0.7, arrow_length_ratio=a_r)
ax.text(0+ey[0]*inertial_length_ratio, 0+ey[1]*inertial_length_ratio+2, 0+ey[2]*inertial_length_ratio, r"$y$", color='g')
ax.quiver(0, 0, 0, ez[0], ez[1], ez[2], length=inertial_length_ratio, color='b', linewidth=0.7, arrow_length_ratio=a_r)
ax.text(0+ez[0]*inertial_length_ratio, 0+ez[1]*inertial_length_ratio, 0+ez[2]*inertial_length_ratio+2, r"$z$", color='b')
ax.set_axis_off() # turn off the plt axis

# # current direction
# c_length_ratio = 15
# ax.quiver(0,-25,0, v_current_i[0], v_current_i[1], v_current_i[2], length=c_length_ratio, color='k', arrow_length_ratio=0.06)
# ax.text(0+v_current_i[0]*c_length_ratio+4, -25+v_current_i[1]*c_length_ratio, 0+v_current_i[2]*c_length_ratio-3, r"$\vec{v}_{current}^{(i)}$", color='k')

# path
# cylinder
ax.plot(x_cyl, y_cyl, z_cyl, label=r"Cylindrical")

# ellipse
ax.plot(x_ellip, y_ellip, z_ellip, label=r"Elliptic")

ax.set_xlim(0, 60)
ax.set_ylim(-60, 60)
ax.set_zlim(0, 60)

# ax.set_xlabel("X")
# ax.set_ylabel("Y")
# ax.set_zlabel("Z")
# plt.ion()
# plt.title("Kite path")
ax.view_init(elev=16, azim=-35, roll=0)
ax.set_title("Cylidrical vs. Elliptic Viviani's curve")
ax.legend(bbox_to_anchor=(0.8, 0.8))
plt.tight_layout()


### 2D path views ###
# fig, ax = plt.subplots(2,1, figsize=(10,7))

plt.figure(figsize=(10,4))
plt.plot(y_cyl,z_cyl, label=r"Cylindrical")
plt.plot(y_ellip,z_ellip, label=r"Elliptic")
plt.title(r"Paths in $y-z$ plane")
plt.xlabel(r"$y$ [m]")
plt.ylabel(r"$z$ [m]")
plt.legend()
plt.grid()

plt.figure(figsize=(10,4))
plt.plot(y_cyl,x_cyl, label=r"Cylindrical")
plt.plot(y_ellip,x_ellip, label=r"Elliptic")
plt.title(r"Paths in $y-x$ plane")
plt.xlabel(r"$y$ [m]")
plt.ylabel(r"$x$ [m]")
plt.legend()
plt.grid()

plt.figure(figsize=(6,6))
plt.plot(x_cyl,z_cyl, label=r"Cylindrical")
plt.plot(x_ellip,z_ellip, label=r"Elliptic")
plt.title(r"Paths in $x-z$ plane")
plt.xlabel(r"$x$ [m]")
plt.ylabel(r"$z$ [m]")
plt.legend()
plt.grid()



plt.show()