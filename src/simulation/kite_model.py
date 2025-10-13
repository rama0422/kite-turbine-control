import math
import numpy as np

from src.utility.configs import rho, g
from src.simulation.functions import path, path_p, path_pp, C_L, C_D, R_sc


class Kite:
    def __init__(self, S, m, vol):
        self.S = S
        self.m = m
        self.vol = vol

        # # Variables
        # self.F_aero = np.array([0,0,0])
        # self.F_grav = np.array([0,0,0])
        # self.F_buoy = np.array([0,0,0])
        # self.F_tot = np.array([0,0,0])
        self.F_thether = 0

        #TODO: retrive acc and gyro data for simulated IMU
        #TODO: add measurments with noise?

        # Data logging #TODO: as np arrays for efficiency? give steps using dt and t_end to class
        self.data_log = {"ts": [],
                         "r": [],
                         "r_p": [],
                         "r_pp": [],
                         "v_kite_i": [],
                         "v_rel_i": [],
                         "v_rel_abs": [],
                         "alpha": [],
                         "Fs_aero": [],
                         "Fs_grav": [],
                         "Fs_buoy": [],
                         "Fs_tot": [],
                         "Fs_thether": []}

    # Caluclate kinematics of the kites current position on path
    def kinematics(self, p):
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
        R_si = np.column_stack([e1, e2, e3])

        return r, r_p, r_pp, e1, e2, e3, R_si

    # Calculate velocities of kite and absolute angle of attack
    def relativeVelocity(self, pdot, r_p, R_si, v_current_i):
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

        return v_kite_i, v_rel_i, v_rel_s, v_rel_c, v_rel_abs, alpha
    
    # CAlculate forces acting on kite and their resulting acceleration along the path
    def accleration(self, pdot, r_p, r_pp, e1, e3, R_si, v_rel_abs, alpha, F_turb = 0):
        F_L = 0.5 * rho * v_rel_abs**2 * self.S * C_L(alpha)
        F_D = 0.5 * rho * v_rel_abs**2 * self.S * C_D(alpha)

        F_aero_c = np.array([-F_D, 0, -F_L])

        F_aero_s = R_sc(-alpha).T @ F_aero_c #TODO: check alpha sign -/+??

        F_aero_i = R_si @ F_aero_s

        # Forces in inertial frame
        g_i = np.array([0, 0, -g])
        F_mg_i = self.m * g_i # gravity force

        z_i = np.array([0 ,0, 1])
        F_b_i = rho * self.vol * g * z_i # buoyancy force

        # turbine force
        F_turb_s = np.array([-F_turb, 0, 0])
        F_turb_i = R_si @ F_turb_s

        F_tot_i = F_aero_i + F_mg_i + F_b_i + F_turb_i # thether force unknown here

        # print(F_aero_i, F_mg, F_b)

        pdotdot = (np.dot(e1, F_tot_i) - self.m * (np.dot(e1, r_pp) * pdot**2)) / (self.m * np.linalg.norm(r_p))

        F_thether = self.m * (np.dot(e3, r_pp) * pdot**2) - np.dot(e3, F_tot_i)

        return pdotdot, F_aero_i, F_mg_i, F_b_i, F_tot_i, F_thether
        

    # Stand alone kite dynamics with inserted F_turb
    def kiteDynamics(self, t, x, v_current_i, F_turb = 0):
        p = x[0]
        pdot = x[1]

        r, r_p, r_pp, e1, e2, e3, R_si = self.kinematics(p)

        v_kite_i, v_rel_i, v_rel_s, v_rel_c, v_rel_abs, alpha = self.relativeVelocity(pdot, r_p, R_si, v_current_i)

        pdotdot, F_aero_i, F_mg_i, F_b_i, F_tot_i, F_thether = self.accleration(pdot, r_p, r_pp, e1, e3, R_si, v_rel_abs, alpha, F_turb)
        self.F_thether = F_thether

        # data logging
        self.data_log["ts"].append(t)
        self.data_log["r"].append(r)
        self.data_log["r_p"].append(r_p)
        self.data_log["r_pp"].append(r_pp)
        self.data_log["v_kite_i"].append(v_kite_i)
        self.data_log["v_rel_i"].append(v_rel_i)
        self.data_log["v_rel_abs"].append(v_rel_abs)
        self.data_log["alpha"].append(alpha)
        self.data_log["Fs_aero"].append(F_aero_i)
        self.data_log["Fs_grav"].append(F_mg_i)
        self.data_log["Fs_buoy"].append(F_b_i)
        self.data_log["Fs_tot"].append(F_tot_i)
        self.data_log["Fs_thether"].append(F_thether)

        return [pdot, pdotdot]