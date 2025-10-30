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
N_gear = 6.1 # TODO: 
r_turb = 0.5
# A_turb = math.pi *r_turb**2
J_gen = 2.0
J_turb = 0.5
# P_gen_max = 2000

"""From generator test"""
T_gen_max = 399.43 #Rated torque
T_gen_max_w = 319.83
w_gen_max = 3189.99 / 60 * 2 * np.pi
w_gen_max_T = 2559.61 / 60 * 2 * np.pi

w_limit = 5000 / 60 * 2 * np.pi
w_test = np.array([3189.99,2559.61,2511.99,2464.43,2416.90,2369.39,2321.90,2263.40,2204.96,2146.56,2088.20,2029.91,1946.22,1862.60,1779.07,1695.61,1612.26,1528.99,1445.63,1362.48,1279.37])
T_test = np.array([319.82,399.43,386.84,373.77,360.19,346.04,331.35,317.47,302.85,287.46,271.20,254.02,238.79,222.18,204.05,184.11,162.11,137.65,110.45,79.58,44.33])
efficiency_test = np.array([93.60,93.40,93.36,93.30,93.24,93.17,93.09,93.03,92.95,92.86,92.74,92.60,92.46,92.30,92.07,91.77,91.34,90.75,89.71,88.08,84.19])/100



eff_gear = 0.99
kp = 25
ki = 10
time_const_T_gen = 0.015
# T_gen_el_limit = 2000

# Controllers
w_ref_base = 2000 / 60 * 2 * np.pi

# # og for real data:
# P_mean_init = 6*1e4
# F_tether_mean_init = 1.8*1e5
# og_controller_div_factor = 105
# og_controller_tsr_const = 57.5

# og for simulation:
P_mean_init = 5*1e4
F_tether_mean_init = 2.2*1e5
og_controller_div_factor = 140
og_controller_tsr_const = 57.5

# IMU magnetometer
h_e = np.array([14361.1, -798.5, 49628.8])/1e1 # divide by 10 to match data, eart frame magnetic field at faroe islands 2025-10-23 "https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml?#igrfwmm"
#"https://repository.library.noaa.gov/view/noaa/71569"
#noise for magentometer: https://www.researchgate.net/publication/272749646_Magnetometer-Augmented_IMU_Simulator_In-Depth_Elaboration

angle_ei = 190 * math.pi / 180

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
                 "GyroX": [0,0,0], #[0, 0.4/3, 0], #TODO CURRENTLY no noise to check stability..
                 "GyroY": [0,0,0], #[0, 1/3, 0],
                 "GyroZ": [0,0,0], #[0, 0.01/3, 0],
                 "MagX": [0, 10/3, 0], #TODO
                 "MagY": [0, 80/3, 0],
                 "MagZ": [0, 30/3, 0]}

# Simulation
dt = 0.005
t_start = 0
t_end = 30
dt_sim_log = 0.01
dt_measurement_log = 0.02
dt_controller = 0.02

p0 = 0.1
pdot0 = 0.6
w0_gen = 2100 / 60 * 2 * math.pi
I0 = 0
T0_gen_el = 0