import numpy as np
import math

# from src.utility.configs import r_turb, J_gen, T_gen_max, w_gen_max, w_gen_max_T, N_gear, eff_gear, kp, ki, rho
from src.utility.configs import rho
from src.simulation.functions import Cp, Cf, MaxTorqueSpeed



class Turbine:
    def __init__(self, r_turb, J_gen, J_turb, T_gen_max, T_gen_max_w, w_gen_max, w_gen_max_T, N_gear, eff_gear, kp, ki):
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

        # variables
        self.F_turb = 0
        self.P_gen_out = 0
        self.T_gen_el = 0

        # fit T-w curve
        ws = np.array([w_gen_max_T, w_gen_max])
        Ts = np.array([T_gen_max, T_gen_max_w])

        self.m, self.b = np.polyfit(ws, Ts, 1)

        # data logging
        self.data_log = {"ts": [],
                         "Fs_turb": [],
                         "Ts_gen_mech": [],
                         "Ts_gen_el_uncliped": [],
                         "Ts_gen_el":[],
                         "ws_ref": [],
                         "errors": [],
                         "TSRs": [],
                         "P_gen_out": [],
                         "vs_rel": []}


    def turbineDynamics(self, t, x, v_rel, w_ref):
        w_gen = x[0]
        I = x[1]

        w_turb = w_gen / self.N_gear

        # w_ref = 1500 / 60 * 2 * math.pi + 30*math.sin(0.5*t)

        # v_rel = 7
        # v_rel = 5 + 1.5*math.sin(0.5*t)
        TSR = (w_turb * self.r_turb) / v_rel

        P_turb = 1/2 * rho * self.A_turb * v_rel**3 * Cp(TSR)
        # T_turb = 1/2 * rho * A_turb * v_rel**2 * r_turb * Cp(TSR) # TODO: should be Cq
        T_turb = P_turb / w_turb if w_turb != 0 else 0
        F_turb = 1/2 * rho * self.A_turb * v_rel**2 * Cf(TSR)
        

        T_gen_mech = self.eff_gear * (T_turb / self.N_gear)

        w_error = w_ref - w_gen
        T_gen_el_uncliped = self.kp * w_error + self.ki * I # TODO: add some sort of delay/inertia to T_gen to no jump to much with discretet w_ref

        # T_gen_el = max(min(T_gen_el, T_gen_el_limit), -T_gen_el_limit) # TODO: add speed dependant max torque
        # if (w_gen < self.w_gen_max_T):
        #     T_gen_el = max(min(T_gen_el, self.T_gen_max), -self.T_gen_max)
        # else:
        #     temp_w = w_gen*60/(2*math.pi)
        #     temp_max_T = 965.0754 - 0.3595477*temp_w + 0.00003567839*temp_w**2
        #     T_gen_el = max(min(T_gen_el, temp_max_T), -temp_max_T)

        T_gen_el, w_gen = MaxTorqueSpeed(T_gen_el_uncliped, w_gen, self.T_gen_max, self.T_gen_max_w, self.w_gen_max, self.w_gen_max_T, self.m, self.b)
        #TODO: add limit to turbine to not exceed w_gen

        P_gen_out = -T_gen_el * w_gen
        #TODO: add loses through Torque-speed curve of generator

        # if (t <= print_time):
        #     print("w_gen:", w_gen, " I:", I, " TSR:", TSR, " T_turb:", T_turb, " T_mech:", T_gen_mech, " w_error:", w_error, " T_gen_el:", T_gen_el)
        #     # *60/(2*math.pi)

        # store variables
        self.F_turb = F_turb
        self.P_gen_out = P_gen_out
        self.T_gen_el = T_gen_el

        # calculate derivatives
        Idot = w_error
        wdot_gen = (T_gen_mech + T_gen_el) / (self.J_gen + self.J_turb / self.N_gear**2)

        # data logging
        # if ((t - self.t_last_log) >= self.dt_log):
        #     self.t_last_log = t
        self.data_log["ts"].append(t)
        self.data_log["Fs_turb"].append(F_turb)
        self.data_log["Ts_gen_mech"].append(T_gen_mech)
        self.data_log["Ts_gen_el_uncliped"].append(T_gen_el_uncliped)
        self.data_log["Ts_gen_el"].append(T_gen_el)
        self.data_log["ws_ref"].append(w_ref)
        self.data_log["errors"].append(w_error)
        self.data_log["TSRs"].append(TSR)
        self.data_log["P_gen_out"].append(P_gen_out)
        self.data_log["vs_rel"].append(v_rel)

        return [wdot_gen, Idot]