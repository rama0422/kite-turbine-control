import numpy as np
import math

# from src.utility.configs import r_turb, J_gen, T_gen_max, w_gen_max, w_gen_max_T, N_gear, eff_gear, kp, ki, rho
from src.utility.configs import rho
from src.simulation.functions import Cp, Cf, MaxTorqueSpeed,T_cap,Efficiency_lookup



class Turbine:
    def __init__(self, r_turb, J_gen, J_turb, T_gen_max, T_gen_max_w, w_gen_max, w_gen_max_T, w_limit, N_gear, eff_gear, kp, ki):
        # turbine params
        self.r_turb = r_turb
        self.A_turb = math.pi *r_turb**2
        self.J_gen = J_gen
        self.J_turb = J_turb
        self.T_gen_max = T_gen_max
        self.T_gen_max_w = T_gen_max_w
        self.w_gen_max = w_gen_max
        self.w_gen_max_T = w_gen_max_T
        self.N_gear = N_gear
        self.eff_gear = eff_gear
        self.kp = kp
        self.ki = ki

        self.w_limit = w_limit



        # variables
        self.F_turb = 0
        self.P_gen_out = 0
        self.T_gen_el = 0

        # fit T-w curve #TODO: ok to be done here?

        ws = np.array([w_gen_max_T, w_gen_max])
        Ts = np.array([T_gen_max, T_gen_max_w])

        self.m, self.b = np.polyfit(ws, Ts, 1)

        # data logging
        self.data_log = {"ts": [],
                         "Fs_turb": [],
                         "Ts_gen_mech": [],
                         "Ts_gen_el_ref_uncliped": [],
                         "Ts_gen_el_ref": [],
                         "Ts_gen_el":[],
                         "ws_ref": [],
                         "errors": [],
                         "TSRs": [],
                         "P_gen_out": [],
                         "vs_rel": []}


    def turbineDynamics(self, t, x, v_rel, w_ref, log_bool=True):
        w_gen = x[0]
        I = x[1]
        T_gen_el = x[2]

        # turbine
        w_turb = w_gen / self.N_gear
        TSR = (w_turb * self.r_turb) / v_rel

        P_turb = 1/2 * rho * self.A_turb * v_rel**3 * Cp(TSR)
        T_turb = P_turb / w_turb if w_turb != 0 else 0
        F_turb = 1/2 * rho * self.A_turb * v_rel**2 * Cf(TSR)
        
        # shaft/generator
        T_gen_mech = self.eff_gear * (T_turb / self.N_gear)
        wdot_gen = (T_gen_mech - T_gen_el) / (self.J_gen + self.J_turb / self.N_gear**2)

        eff = Efficiency_lookup(T_gen_el, w_gen)
        P_gen_out = T_gen_el * w_gen * eff

        # PI torque ref controller
        """Modified: w_error>0 (want to slow down) -> Te>0 -> BREAKING TORQUE -> Decrease w' """
        w_error = w_gen - w_ref
        T_gen_el_ref_uncliped = self.kp * w_error + self.ki * I
        # T_gen_el_ref = T_cap(T_gen_el_ref_uncliped, w_gen, self.w_limit, self.w_gen_max_T, self.T_gen_max, self.m, self.b)
        # T_gen_el_ref, _ = MaxTorqueSpeed(T_gen_el_ref_uncliped, w_gen, self.T_gen_max, self.T_gen_max_w, self.w_gen_max, self.w_gen_max_T, self.m, self.b)
        T_gen_el_ref =T_gen_el_ref_uncliped

        Idot = w_error

        # torque dynamics https://www.sciencedirect.com/science/article/pii/S1364032115006814
        time_const_T_gen = 0.1
        Tdot_gen = (T_gen_el_ref_uncliped - T_gen_el) / time_const_T_gen
        T_gen_el, _ = MaxTorqueSpeed(T_gen_el, w_gen, self.T_gen_max, self.T_gen_max_w, self.w_gen_max, self.w_gen_max_T, self.m, self.b)

        #T_gen_el = MaxTorqueSpeed(w_gen, self.T_gen_max, self.T_gen_max_w, self.w_gen_max, self.w_gen_max_T, self.m, self.b)
        #TODO: add limit to turbine to not exceed w_gen

        # T_gen_el = T_cap(T_gen_el_uncliped,w_gen,self.w_limit,self.w_gen_max_T,self.T_gen_max,self.m,self.b) 
        # T_gen_el, _ = MaxTorqueSpeed(T_gen_el_uncliped, w_gen, self.T_gen_max, self.T_gen_max_w, self.w_gen_max, self.w_gen_max_T, self.m, self.b)

        eff = Efficiency_lookup(T_gen_el,w_gen)
        P_gen_out = T_gen_el * w_gen * eff



        # store variables
        self.F_turb = F_turb
        self.P_gen_out = P_gen_out
        self.T_gen_el = T_gen_el

        # log data
        if log_bool:
            self.data_log["ts"].append(t)
            self.data_log["Fs_turb"].append(F_turb)
            self.data_log["Ts_gen_mech"].append(T_gen_mech)
            self.data_log["Ts_gen_el_ref_uncliped"].append(T_gen_el_ref_uncliped)
            self.data_log["Ts_gen_el_ref"].append(T_gen_el_ref)
            self.data_log["Ts_gen_el"].append(T_gen_el)
            self.data_log["ws_ref"].append(w_ref)
            self.data_log["errors"].append(w_error)
            self.data_log["TSRs"].append(TSR)
            self.data_log["P_gen_out"].append(P_gen_out)
            self.data_log["vs_rel"].append(v_rel)

        return np.array([wdot_gen, Idot, Tdot_gen])