import math
import numpy as np
import sympy as sp

from src.utility.configs import rho, g, R_path, r_path, theta_path


########### KITE #############
# TODO: make sure these are only calculated once per run
# path functions
p_sym, R_sym, r_sym, theta_sym = sp.symbols('p_sym R_sym r_sym theta_sym', real=True)

sub = (R_sym - r_sym)
x = sub * sp.cos(theta_sym) + r_sym * sp.cos(p_sym) * sp.cos(theta_sym) - r_sym * sp.sin(p_sym) * sp.sin(theta_sym)
y = 2 * sp.sqrt(sub * r_sym) * sp.sin(p_sym / 2)
z = sub * sp.sin(theta_sym) + r_sym * sp.cos(p_sym) * sp.sin(theta_sym) + r_sym * sp.sin(p_sym) * sp.cos(theta_sym)

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

#TODO: Add variable C_L, C_D
def C_L(alpha):
    c = 0.15
    return c

def C_D(alpha):
    c = 0.0275
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