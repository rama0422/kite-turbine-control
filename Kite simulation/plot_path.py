import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d


#TODO: change to path dependant on R, r and d like in paper
def path(p):
    x = 53 + 17.7 * (math.cos(p) - math.sin(p))
    y = 86.6 * math.sin(p/2)
    z = 53 + 17.7 * (math.cos(p) + math.sin(p))
    return np.array([x, y, z])

def path_p(p):
    x_p = 17.7 * (-math.sin(p) - math.cos(p))
    y_p = 43.3 * math.cos(p/2)
    z_p = 17.7 * (math.cos(p) - math.sin(p))
    return np.array([x_p, y_p, z_p])

def path_pp(p):
    x_pp = 17.7 * (-math.cos(p) + math.sin(p))
    y_pp = -21.65 * math.sin(p/2)
    z_pp = 17.7 * (-math.sin(p) - math.cos(p))
    return np.array([x_pp, y_pp, z_pp])


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

    
current = [2,0,0] # wind in x direction

# Plot path
fig = plt.figure(figsize=(7,7))
ax = plt.axes(projection = '3d')

p = 1*math.pi+0.5
p_start = path(p)
pp_start = path_p(p)
print(pp_start)

ax.plot(x, y, z)
ax.plot(p_start[0], p_start[1], p_start[2], 'ro') # start point
ax.quiver(p_start[0], p_start[1], p_start[2], pp_start[0], pp_start[1], pp_start[2], length=0.5, color='r') # tangent at start point
ax.quiver(0,0,0, current[0], current[1], current[2], length=15, color='g')

ax.set_xlim(0, 100)
ax.set_ylim(-100, 100)
ax.set_zlim(0, 100)

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
# plt.ion()
plt.title("Kite path")
plt.show()