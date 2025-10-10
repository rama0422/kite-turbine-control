import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

from src.simulation.kite_model import Kite
from src.simulation.turbine_model import Turbine
from src.simulation.full_system_model import FullSystemModel
from src.utility.configs import rho, g, S, m, vol, r_turb, J_gen, T_gen_max, w_gen_max, w_gen_max_T, N_gear, eff_gear, kp, ki, dt, t_end


kite = Kite(S, m, vol)
trubine = Turbine(r_turb, J_gen, T_gen_max, w_gen_max, w_gen_max_T, N_gear, eff_gear, kp, ki)
kiteSystem = FullSystemModel(kite, trubine)

# simulation params
v_current_i = np.array([2,0,0])
p0 = 0
pdot0 = 0.2
w0_gen = 1400 / 60 * 2 * math.pi
I0 = 0
x0 = [p0, pdot0, w0_gen, I0]

# systemDynamics(self, t, x, v_current_i, w_ref = w_ref_base)
sol = solve_ivp(lambda t, x: kiteSystem.systemDynamics(t, x, v_current_i), [0, t_end], x0, max_step = dt)

print(sol)

ts = np.array(kiteSystem.data_log["ts"])
rs = np.array(kiteSystem.data_log["r"])
rs_p = np.array(kiteSystem.data_log["r_p"])
rs_pp= np.array(kiteSystem.data_log["r_pp"])
vs_kite_i = np.array(kiteSystem.data_log["v_kite_i"])
vs_rel_i = np.array(kiteSystem.data_log["v_rel_i"])
vs_rel_abs = np.array(kiteSystem.data_log["v_rel_abs"])
alphas = np.array(kiteSystem.data_log["alpha"])
Fs_aero_i = np.array(kiteSystem.data_log["Fs_aero"])
Fs_grav_i = np.array(kiteSystem.data_log["Fs_grav"])
Fs_buoy_i = np.array(kiteSystem.data_log["Fs_buoy"])
Fs_tot_i = np.array(kiteSystem.data_log["Fs_tot"])
Fs_thether = np.array(kiteSystem.data_log["Fs_thether"])

# ts1 = np.array(kiteSystem.turbine.data_log["ts"])
Fs_turb = np.array(kiteSystem.turbine.data_log["Fs_turb"])
Ts_gen_mech = np.array(kiteSystem.turbine.data_log["Ts_gen_mech"])
Ts_gen_el = np.array(kiteSystem.turbine.data_log["Ts_gen_el"])
ws_ref = np.array(kiteSystem.turbine.data_log["ws_ref"])
errors = np.array(kiteSystem.turbine.data_log["errors"])
TSRs = np.array(kiteSystem.turbine.data_log["TSRs"])
# vs_rel = np.array(kiteSystem.turbine.data_log["vs_rel"])

# print(ts1.shape == ts.shape)

fix, ax = plt.subplots(4,2, figsize=(15,9))

ax[0,0].plot(sol.t, sol.y[0])

ax[1,0].plot(sol.t, sol.y[1])

ax[2,0].plot(sol.t, sol.y[2])

ax[3,0].plot(sol.t, sol.y[3])

ax[0,1].plot(ts, np.linalg.norm(Fs_aero_i, axis=1))
ax[0,1].plot(ts, np.linalg.norm(Fs_buoy_i, axis=1))
ax[0,1].plot(ts, np.linalg.norm(Fs_grav_i, axis=1))
ax[0,1].plot(ts, Fs_turb)
ax[0,1].plot(ts, np.linalg.norm(Fs_tot_i, axis=1))
ax[0,1].plot(ts, Fs_thether)
ax[0,1].set_title(r"Forces over time")
ax[0,1].legend([r"$F_{aero}$", r"$F_{buoy}$", r"$F_{grav}$", r"$F_{turb}$", r"$F_{tot}$", r"$F_{thether}$"])
ax[0,1].set_xlabel("Time [s]")

ax[1,1].plot(ts, np.linalg.norm(vs_kite_i, axis=1))
ax[1,1].plot(ts, np.linalg.norm(vs_rel_i, axis=1))
ax[1,1].set_title("Linear kite speed")
ax[1,1].set_xlabel("Time [s]")
ax[1,1].legend([r"$|v_{kite,i}|$", r"$|v_{rel,i}|$"])

plt.tight_layout(pad=1.0)
plt.show()