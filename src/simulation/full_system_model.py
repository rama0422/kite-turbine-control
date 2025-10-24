import numpy as np
import math
import scipy

# from src.utility.configs import w_ref_base, g
from src.simulation.functions import R_pb, R_pc

class FullSystemModel:
    def __init__(self, kite, turbine, w_ref_base, dt_controller, dt_measurement_log, h_i, controller=None, sensors=None):
        self.kite = kite
        self.turbine = turbine
        self.controller = controller
        self.sensors = sensors
        self.h_i = h_i

        self.dt_controller = dt_controller
        self.dt_measurement_log = dt_measurement_log

        # variables
        self.R_pi_last = np.identity(3)
        self.t_last = -0.02
        self.t_controller_last = -1
        self.w_ref = w_ref_base
        self.t_measurement_last = -1

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
                            "acc_b": [],
                            "omega_b": [],
                            "h_b": []}



    def systemDynamics(self, t, x, v_current_i):
        p = x[0]
        pdot = x[1]
        w_gen = x[2]
        I = x[3]

        r, r_p, r_pp, e1, e2, e3, R_pi = self.kite.kinematics(p)

        v_kite_i, v_rel_i, v_rel_s, v_rel_c, v_rel_abs, alpha_pc, alpha_pb, alpha = self.kite.relativeVelocity(p, pdot, r_p, R_pi, v_current_i)

        # check if a controller is used and if at least one time step have passed, if so update w_ref using controller
        if ((self.controller != None) & (len(self.data_log["Fs_thether_abs"]) > 0) & (t - self.t_controller_last >= self.dt_controller)):
            # print("Controller used")
            self.t_controller_last = t

            # P_last = self.turbine.data_log["P_gen_out"][-1] # TODO: realistic that is checks last logged value? or do we want real last value
            # F_tether_last = self.data_log["Fs_thether_abs"][-1]
            P_last = self.sensors.noise_measurments["Power"][-1] * 1e3 # TODO: realistic that is checks last logged measurment value? or do we want real last value
            F_tether_last = self.sensors.noise_measurments["TetherForce"][-1]
            self.w_ref = self.controller.getSpeedRef(t, P_last, F_tether_last)

        #TODO: turbin should in reality get v_rel from the body frame (frame rotated with alpha_pb)
        [wdot_gen, Idot] = self.turbine.turbineDynamics(t, [w_gen, I], -v_rel_s[0], self.w_ref)
        # F_turb = self.turbine.F_turb

        pdotdot, F_aero_i, F_mg_i, F_b_i, F_tot_i, F_thether, F_aero_p, F_turb_p = self.kite.accleration(pdot, r_p, r_pp, e1, e3, R_pi, v_rel_c[0], alpha, self.turbine.F_turb)
        self.F_thether = F_thether


        # IMU measurements
        R_pb_calc = R_pb(alpha_pb)
        # acceleations
        acc_i = (r_pp * pdot**2 + r_p * pdotdot) # g and b not added since we have zero bouyance + gravity,  + np.array([0,0,g])
        acc_p = R_pi.T @ acc_i
        acc_b = R_pb_calc @ acc_p

        # angular velocities #TODO: does not curently work, think it gets very large for y and z sometimes.
        dR_pi = self.R_pi_last.T @ R_pi
        # skewed = scipy.linalg.logm(dR_pi)
        skewed = self.so3MLog(dR_pi)
        dt = t - self.t_last
        omega_p = np.array([skewed[2,1], skewed[0,2], skewed[1,0]]) / (dt)

        omega_b = R_pb_calc @ omega_p

        self.R_pi_last = R_pi
        self.t_last = t

        # Magnetometer
        
        h_b = R_pb_calc @ R_pi @ self.h_i

        # sensor values and logging
        if ((self.sensors != None) & (t - self.t_measurement_last >= self.dt_measurement_log)):
            # change values to obtaine data similar to the real data
            measurments = { "Elevation": r[2], # z
                            "TetherForce": self.F_thether,
                            "TJPitchAngle": alpha_pb * 180 / math.pi,
                            "GeneratorSpdRpm": w_gen * 60 / (2*math.pi),
                            "Power": self.turbine.P_gen_out / 1e3,
                            "Torque": self.turbine.T_gen_el,
                            "AccX": acc_b[0],
                            "AccY": acc_b[1],
                            "AccZ": acc_b[2],
                            "GyroX": omega_b[0],
                            "GyroY": omega_b[1],
                            "GyroZ": omega_b[2],
                            "MagX": h_b[0],
                            "MagY": h_b[1],
                            "MagZ": h_b[2]}

            self.sensors.addNoise(measurments, t)

        # data logging
        # if ((t - self.t_last_log) >= self.dt_log):
        #     self.t_last_log = t
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
        self.data_log["omega_b"].append(omega_b)
        self.data_log["h_b"].append(h_b)

        #turbine data log is found in turbine object

        return [pdot, pdotdot, wdot_gen, Idot]
    
    # faster then scipy.linalg.logm
    def so3MLog(self, R):
        c = (np.trace(R) - 1.0) / 2.0
        c = np.clip(c, -1.0, 1.0)
        theta = np.arccos(c)
        if theta < 1e-8:
            return 0.5 * (R - R.T)
        return theta / (2.0 * np.sin(theta)) * (R - R.T)