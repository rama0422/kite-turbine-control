import math
import numpy as np
import sympy as sp

from src.utility.configs import rho, g, R_path, r_path, theta_path, TJpitch_amp, TJpitch_offset, TJpitch_shift


########### KITE #############
# TODO: make sure these are only calculated once per run
# path functions

p_sym, R_sym, r_sym, theta_sym = sp.symbols('p_sym R_sym r_sym theta_sym', real=True)

# Circular cylinder
sub = (R_sym - r_sym)
x = sub * sp.cos(theta_sym) + r_sym * sp.cos(p_sym) * sp.cos(theta_sym) - r_sym * sp.sin(p_sym) * sp.sin(theta_sym)
y = 2 * sp.sqrt(sub * r_sym) * sp.sin(p_sym / 2)
z = sub * sp.sin(theta_sym) + r_sym * sp.cos(p_sym) * sp.sin(theta_sym) + r_sym * sp.sin(p_sym) * sp.cos(theta_sym)
sub = (R_sym - r_sym)
x = sub + r_sym * sp.cos(p_sym)
y = 2 * sp.sqrt(sub * r_sym) * sp.sin(p_sym/2)
z = r_sym * sp.sin(p_sym)

# Rotate
R = np.array([[sp.cos(theta_sym), 0, -sp.sin(theta_sym)],
              [0, 1, 0],
              [sp.sin(theta_sym), 0, sp.cos(theta_sym)]])
x,y,z = R @ np.array([x,y,z]).T

# differentiate
x_p, y_p, z_p = sp.diff(x, p_sym), sp.diff(y, p_sym), sp.diff(z, p_sym)
x_pp, y_pp, z_pp = sp.diff(x_p, p_sym), sp.diff(y_p, p_sym), sp.diff(z_p, p_sym)

path_lamb = sp.lambdify((p_sym, R_sym, r_sym, theta_sym), [x, y, z], 'numpy')
path_p_lamb = sp.lambdify((p_sym, R_sym, r_sym, theta_sym), [x_p, y_p, z_p], 'numpy')
path_pp_lamb = sp.lambdify((p_sym, R_sym, r_sym, theta_sym), [x_pp, y_pp, z_pp], 'numpy')

def path(p):
    return np.array(path_lamb(p, R_path, r_path, theta_path))

def path_p(p):
    return np.array(path_p_lamb(p, R_path, r_path, theta_path))

def path_pp(p):
    return np.array(path_pp_lamb(p, R_path, r_path, theta_path))



# # Elliptic cylinder
# p_sym, R_sym, a_sym, b_sym, theta_sym = sp.symbols('p_sym R_sym a_sym b_sym theta_sym', real=True)

# #TODO: not differtiable
# sub = (R_sym - a_sym)
# x = sub + a_sym * sp.cos(p_sym)
# y = sp.sign(sp.sin(p_sym)) * sp.sqrt(R_sym**2 - (sub + a_sym * sp.cos(p_sym))**2 - b_sym**2 * sp.sin(p_sym)**2)
# z = b_sym * sp.sin(p_sym)

# # Rotate
# R = np.array([[sp.cos(theta_sym), 0, -sp.sin(theta_sym)],
#               [0, 1, 0],
#               [sp.sin(theta_sym), 0, sp.cos(theta_sym)]])
# x,y,z = R @ np.array([x,y,z]).T

# # differentiate
# x_p, y_p, z_p = sp.diff(x, p_sym), sp.diff(y, p_sym), sp.diff(z, p_sym)
# x_pp, y_pp, z_pp = sp.diff(x_p, p_sym), sp.diff(y_p, p_sym), sp.diff(z_p, p_sym)

# path_lamb = sp.lambdify((p_sym, R_sym, a_sym, b_sym, theta_sym), [x, y, z], 'numpy')
# path_p_lamb = sp.lambdify((p_sym, R_sym, a_sym, b_sym, theta_sym), [x_p, y_p, z_p], 'numpy')
# path_pp_lamb = sp.lambdify((p_sym, R_sym, a_sym, b_sym, theta_sym), [x_pp, y_pp, z_pp], 'numpy')

# a_path = 1
# b_path = 2
# def path(p):
#     return np.array(path_lamb(p, R_path, a_path, b_path, theta_path))

# def path_p(p):
#     return np.array(path_p_lamb(p, R_path, a_path, b_path, theta_path))

# def path_pp(p):
#     return np.array(path_pp_lamb(p, R_path, a_path, b_path, theta_path))


#  TJPitchAngle in degrees (tether-joint)
def TJpitch(p):
    return TJpitch_amp * math.sin(p + TJpitch_shift) + TJpitch_offset

# def path(p):
#     sub = (R_path - r_path)
#     x = sub * math.cos(theta_path) + r_path * math.cos(p) * math.cos(theta_path) - r_path * math.sin(p)**2 
#     y = 2 * math.sqrt(sub * r_path) * math.sin(p/2)
#     z = sub * math.sin(theta_path) + r_path * math.cos(p) * math.sin(theta_path) + r_path * math.sin(p) * math.cos(theta_path)
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

# def stabilityBasis:
#     e1 = 

# https://www.osti.gov/servlets/purl/1907523
#TODO: alpha in degrees?
# def C_L(alpha):
#     alpha = alpha * 180 / math.pi
#     c = 6.25*1e-2 * alpha + 1.33*1e-1
#     return c

# def C_D(alpha):
#     alpha = alpha * 180 / math.pi
#     c = 2.442*1e-4 * alpha**2 + 1.06*1e-3 * alpha + 2.22*1e-3
#     return c

# https://odr.chalmers.se/server/api/core/bitstreams/e9a88fb7-0b47-4155-857d-599210900918/content
# https://www.researchgate.net/publication/308400130_Modeling_and_Simulation_of_Tethered_Undersea_Kites
def C_L(alpha):
    alpha = alpha * 180 / math.pi
    if (abs(alpha) > 20):
        c = 5.15*1e-10 * alpha**5 + 7.3*1e-24 * alpha**4 - 9.06*1e-6 * alpha**3 - 9.06*1e-20 * alpha**2 + 0.0405 * alpha + 0.2
    else:
        c = -2.27*1e-4 * alpha**3 - 1.65*1e-19 * alpha**2 + 0.123 * alpha + 0.2
    return c

def C_D(alpha, C_L):
    alpha = alpha * 180 / math.pi
    c = 0.05 + C_L**2 / (math.pi * 0.9 * 3) # see sources
    return c

def R_sc(a):
    a = -a
    return (np.array([[math.cos(a),  0, math.sin(a)],
                      [0,            1, 0],
                      [-math.sin(a), 0, math.cos(a)]]))


############## TURBINE #############
def Cp(TSR):
    x = TSR
    y = -0.1538643 + 0.4473311*x - 0.09631951*x**2 + 0.003482307*x**3
    return max(y,0)

def Cf(TSR):
    x = TSR
    y = 0.88 - 0.1187302*x + 0.02369048*x**2 - 0.004722222*x**3
    return max(y,0)


        #  500               500        
        # 4000               250        
        # 1000               440        
        # 2000               360        
        # 3000               290       
        # 
def MaxTorqueSpeed(T_in, w_in, T_max, T_max_w, w_max, w_max_T, m, b):
    T = abs(T_in)
    if (w_in < w_max_T ):
        T_out = min(T, T_max)
        w_out = w_in
    elif (w_in > w_max):
        w_out = w_in #TODO: or lock it?
        T_out = T_max_w
    else:
        T_max_temp =  m * w_in + b
        T_out = min(T, T_max_temp)
        w_out = w_in

    if (T_in < 0):
        T_out = -T_out

    return T_out, w_out
     

# if (w_gen < self.w_gen_max_T):
#             T_gen_el = max(min(T_gen_el, self.T_gen_max), -self.T_gen_max)
#         else:
#             temp_w = w_gen*60/(2*math.pi)
#             temp_max_T = 965.0754 - 0.3595477*temp_w + 0.00003567839*temp_w**2
#             T_gen_el = max(min(T_gen_el, temp_max_T), -temp_max_T)