import numpy as np

# Enviorment
rho = 1025
v_current_i = np.array([1.5,0,0])
g = 9.81

# Path

# Kite
S = 4.9*1.5 #TODO from image and known width
m = 2700    # from website
vol = m/rho # for neutral buoyancy


# Turbine
N_gear = 5 # TODO: does turbine or gen rotate faster
r_turb = 0.5
# A_turb = math.pi *r_turb**2
J_gen = 5
# P_gen_max = 2000
T_gen_max = 500
w_gen_max = 3000
w_gen_max_T = 1500

eff_gear = 0.99
kp = 500
ki = 500
# T_gen_el_limit = 2000

# Controller
w_ref_base = 1500 / 60 * 2 * np.pi


# Simulation
dt = 0.02
t_end = 100