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
a_path = 2
b_path = 5.2

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
J_gen = 3
J_turb = 0.5
# P_gen_max = 2000
T_gen_max = 700
T_gen_max_w = 400
w_gen_max = 4000 / 60 * 2 * np.pi
w_gen_max_T = 1000 / 60 * 2 * np.pi

eff_gear = 0.98
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

# IMU magnetometer
h_e = np.array([14361.1, -798.5, 49628.8])/1e1 # divide by 10 to match data, eart frame magnetic field at faroe islands 2025-10-23 "https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml?#igrfwmm"
#"https://repository.library.noaa.gov/view/noaa/71569"
#noise for magentometer: https://www.researchgate.net/publication/272749646_Magnetometer-Augmented_IMU_Simulator_In-Depth_Elaboration

angle_ei = -10 * math.pi / 180

#TODO: where should this be placed, not in config?
R_ei = np.array([[math.cos(angle_ei), -math.sin(angle_ei),  0],
                [-math.sin(angle_ei), -math.cos(angle_ei),  0],
                [0,                                     0, -1]])
h_i = R_ei @ h_e


# Sensors
noise_configs = {"Elevation": [0, 0.2*1e-2*60/3, 0], #TODO
                 "TetherForce": [0, 2*1e-2*200*1e3/3, 0], #TODO
                 "TJPitchAngle": [0, 1*1e-2*90/3, 0], #TODO
                 "GeneratorSpdRpm": [0, 0.01*1e-2*10000/3, 0], #TODO
                 "Power": [0, 1*1e-2*100000/1e3/3, 0], #TODO
                 "Torque": [0, 2*1e-2*1000/3, 0], #TODO
                 "AccX": [0, 0.2/3, 0], #TODO
                 "AccY": [0, 0.2/3, 0],
                 "AccZ": [0, 0.4/3, 0],
                 "GyroX": [0, 0.4/3, 0], #TODO
                 "GyroY": [0, 1/3, 0],
                 "GyroZ": [0, 0.01/3, 0],
                 "MagX": [0, 10/3, 0], #TODO
                 "MagY": [0, 80/3, 0],
                 "MagZ": [0, 30/3, 0]}

# Simulation
dt = 0.02
t_end = 40
dt_sim_log = 0.01
dt_measurement_log = 0.02
dt_controller = 0.02