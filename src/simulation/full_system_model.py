import numpy as np
import math

from src.utility.configs import w_ref_base

class FullSystemModel:
    def __init__(self, kite, turbine):
        self.kite = kite
        self.turbine = turbine
        #TODO: add controller

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
                            "Fs_aero": [],
                            "Fs_grav": [],
                            "Fs_buoy": [],
                            "Fs_tot": [],
                            "Fs_thether": []}



    def systemDynamics(self, t, x, v_current_i, w_ref = w_ref_base):
        p = x[0]
        pdot = x[1]
        w_gen = x[2]
        I = x[3]

        r, r_p, r_pp, e1, e2, e3, R_si = self.kite.kinematics(p)

        v_kite_i, v_rel_i, v_rel_s, v_rel_c, v_rel_abs, alpha_pc, alpha_pb, alpha = self.kite.relativeVelocity(p, pdot, r_p, R_si, v_current_i)

        [wdot_gen, Idot] = self.turbine.turbineDynamics(t, [w_gen, I], v_rel_abs, w_ref)
        # F_turb = self.turbine.F_turb

        pdotdot, F_aero_i, F_mg_i, F_b_i, F_tot_i, F_thether = self.kite.accleration(pdot, r_p, r_pp, e1, e3, R_si, v_rel_abs, alpha, self.turbine.F_turb)
        self.F_thether = F_thether

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

        self.data_log["Fs_aero"].append(F_aero_i)
        self.data_log["Fs_grav"].append(F_mg_i)
        self.data_log["Fs_buoy"].append(F_b_i)
        self.data_log["Fs_tot"].append(F_tot_i)
        self.data_log["Fs_thether"].append(F_thether)

        #turbine data log is found in turbine object

        return [pdot, pdotdot, wdot_gen, Idot]