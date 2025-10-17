import numpy as np
import math

# Enviorment
rho = 1025
v_current_i = np.array([2,0,0])
g = 9.81

# Path
R_path = 60
theta_path = 15 / 180 * np.pi # 15 deg gives similar elevation as in data #TODO:
viviani_type = "elliptic" # ["cylindrical", "elliptic"]

# Cylindrical
r_path = 5 # elevation changes around 10 m durign cycle (from data) d = 5*2 = 10

# Elliptic
a_path = 3
b_path = 6

# Path dependant pitch angle (TJPitchAngle)
TJpitch_offset = 0
TJpitch_amp = 2
TJpitch_shift = math.pi/3

# Kite
S = 4.9*1.5 #TODO from image and known width
m = 2700    # from website
vol = m/rho # for neutral buoyancy


# Turbine
N_gear = 6.1 # TODO: does turbine or gen rotate faster
r_turb = 0.5
# A_turb = math.pi *r_turb**2
J_gen = 5
# P_gen_max = 2000
T_gen_max = 700
T_gen_max_w = 400
w_gen_max = 4000 / 60 * 2 * np.pi
w_gen_max_T = 1000 / 60 * 2 * np.pi

eff_gear = 0.99
kp = 30
ki = 20
# T_gen_el_limit = 2000

# Controllers
w_ref_base = 2000 / 60 * 2 * np.pi

# # og for real data:
# P_mean_init = 6*1e4
# F_tether_mean_init = 1.8*1e5
# og_controller_div_factor = 105
# og_controller_tsr_const = 57.5

# og for simulation:
P_mean_init = 7*1e4
F_tether_mean_init = 3*1e5
og_controller_div_factor = 250
og_controller_tsr_const = 57.5


# Simulation
dt = 0.02
t_end = 40