import numpy as np
import math

from src.utility.configs import w_ref_base, g
from src.simulation.functions import R_pb, R_pc

class FullSystemModel:
    def __init__(self, kite, turbine, controller=None):
        self.kite = kite
        self.turbine = turbine
        self.controller = controller

        # TODO: can be done in the three kite functions instead and then be callabel with self.kite.data_log
        self.data_log = {   "ts": [],
                            "r": [],
                            "r_p": [],
                            "r_pp": [],
                            "v_kite_i": [],
                            "v_rel_i": [],
                            "v_rel_abs": [],
                            "alpha_pc": [],
                            "alpha_pb": [],
                            "alpha": [],
                            "Fs_aero_i": [],
                            "Fs_grav_i": [],
                            "Fs_buoy_i": [],
                            "Fs_tot_i": [],
                            "Fs_thether_abs": [],
                            "Fs_aero_p": [],
                            "Fs_turb_p": [],
                            "acc_i": [],
                            "acc_p": [],
                            "acc_b": []}



    def systemDynamics(self, t, x, v_current_i, w_ref = w_ref_base):
        p = x[0]
        pdot = x[1]
        w_gen = x[2]
        I = x[3]

        r, r_p, r_pp, e1, e2, e3, R_pi = self.kite.kinematics(p)

        v_kite_i, v_rel_i, v_rel_s, v_rel_c, v_rel_abs, alpha_pc, alpha_pb, alpha = self.kite.relativeVelocity(p, pdot, r_p, R_pi, v_current_i)

        # check if a controller is used and if at least one time step have passed, if so update w_ref using controller
        if ((self.controller != None) & (len(self.data_log["Fs_thether_abs"]) > 0)):
            # print("Controller used")
            P_last = self.turbine.data_log["P_gen_out"][-1]
            F_tether_last = self.data_log["Fs_thether_abs"][-1]
            w_ref = self.controller.getSpeedRef(P_last, F_tether_last)

        #TODO: turbin should in reality get v_rel from the body frame (frame rotated with alpha_pb)
        [wdot_gen, Idot] = self.turbine.turbineDynamics(t, [w_gen, I], -v_rel_s[0], w_ref)
        # F_turb = self.turbine.F_turb

        pdotdot, F_aero_i, F_mg_i, F_b_i, F_tot_i, F_thether, F_aero_p, F_turb_p = self.kite.accleration(pdot, r_p, r_pp, e1, e3, R_pi, v_rel_c[0], alpha, self.turbine.F_turb)
        self.F_thether = F_thether

        # IMU measurements
        # acceleations
        acc_i = (r_pp * pdot**2 + r_p * pdotdot) # not added since we have zero bouyance + gravity,  + np.array([0,0,g])
        acc_p = R_pi.T @ acc_i
        acc_b = R_pb(alpha_pb) @ acc_p

        # angular velocities
        # diffrence between angles obtained from r at time t and t-1

        # data logging
        self.data_log["ts"].append(t)
        self.data_log["r"].append(r)
        self.data_log["r_p"].append(r_p)
        self.data_log["r_pp"].append(r_pp)
        self.data_log["v_kite_i"].append(v_kite_i)
        self.data_log["v_rel_i"].append(v_rel_i)
        self.data_log["v_rel_abs"].append(v_rel_abs)
        self.data_log["alpha_pc"].append(alpha_pc)
        self.data_log["alpha_pb"].append(alpha_pb)
        self.data_log["alpha"].append(alpha)

        self.data_log["Fs_aero_i"].append(F_aero_i)
        self.data_log["Fs_grav_i"].append(F_mg_i)
        self.data_log["Fs_buoy_i"].append(F_b_i)
        self.data_log["Fs_tot_i"].append(F_tot_i)
        self.data_log["Fs_thether_abs"].append(F_thether)
        self.data_log["Fs_aero_p"].append(F_aero_p)
        self.data_log["Fs_turb_p"].append(F_turb_p)

        self.data_log["acc_i"].append(acc_i)
        self.data_log["acc_p"].append(acc_p)
        self.data_log["acc_b"].append(acc_b)

        #turbine data log is found in turbine object

        return [pdot, pdotdot, wdot_gen, Idot]