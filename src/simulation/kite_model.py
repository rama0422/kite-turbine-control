from src.utility.configs import rho
import math
import numpy as np


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

# def stabilityBasis:
#     e1 = 

#TODO
def C_L(alpha):
    c = 0.15
    return c

def C_D(alpha):
    c = 0.0275
    return c

def R_sc(a):
    a = -a
    return (np.array([[math.cos(a), 0, math.sin(a)],
                      [0, 1, 0],
                      [-math.sin(a), 0, math.cos(a)]]))


class Kite:
    def __init__(self, S, m, vol):
        self.S = S
        self.m = m
        self.vol = vol

        # # Variables
        # self.F_aero = np.array([0,0,0])
        # self.F_grav = np.array([0,0,0])
        # self.F_buoy = np.array([0,0,0])
        # self.F_hydro = np.array([0,0,0])
        # self.F_net = np.array([0,0,0])

        # Data logging
        self.data_log = {"ts": [],
                         "Fs_aero": [],
                         "Fs_grav": [],
                         "Fs_buoy": [],
                         "Fs_hydro": [],
                         "Fs_net": []}

        
    def kiteDynamics(self, t, x, v_current_i):
        p = x[0]
        pdot = x[1]

        r = path(p)
        r_p = path_p(p)
        r_pp = path_pp(p)

        # TODO: pre calculate and use as functions
        # Stability basis
        e1 = r_p / np.linalg.norm(r_p)
        e3 = -r / np.linalg.norm(r)
        e2 = np.linalg.cross(e3,e1) #TODO or reverse?
        e2 /= np.linalg.norm(e2) # Does it need to be normalized?

        # print(e1, e2, e3)

        # Rot matrices 
        # R_si = np.array([e1,e2,e3])
        R_si = np.column_stack([e1, e2, e3])

        # print(R_si)

        # Relative water velocity
        v_kite_i = r_p * pdot
        v_rel_i = v_current_i - v_kite_i

        v_rel_s = R_si.T @ v_rel_i

        alpha_ic = np.atan2(-v_rel_s[2], -v_rel_s[0])# angle between axis e1 and v_rel_s
        # print(v_rel_s)
        v_rel_c = np.array([math.sqrt(v_rel_s[2]**2 + v_rel_s[0]**2), 0, 0])

        v_rel_abs = v_rel_c[0]

        # Forces in current frame
        alpha = alpha_ic #TODO

        F_L = 0.5 * rho * v_rel_abs**2 * S * C_L(alpha)
        F_D = 0.5 * rho * v_rel_abs**2 * S * C_D(alpha)

        F_aero_c = np.array([-F_D, 0, -F_L])

        F_aero_s = R_sc(-alpha_ic).T @ F_aero_c #TODO: check alpha sign -/+??

        F_aero_i = R_si @ F_aero_s

        # Forces in inertial frame
        g_i = np.array([0, 0, -g])
        F_mg = m * g_i # gravity force

        z_i = np.array([0 ,0, 1])
        F_b = rho * vol * g * z_i # buoyancy force

        F_tot_i = F_aero_i + F_mg + F_b # thether force unknown here

        # print(F_aero_i, F_mg, F_b)

        #
        pdotdot = (np.dot(e1, F_tot_i) - m * (np.dot(e1, r_pp) * pdot**2)) / (m * np.linalg.norm(r_p))

        F_thether = m * (np.dot(e3, r_pp) * pdot**2) - np.dot(e3, F_tot_i)
