import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

from src.simulation.kite_model import Kite
from src.simulation.turbine_model import Turbine
from src.simulation.full_system_model import FullSystemModel
from src.utility.configs import rho, g, S, m, vol, r_turb, J_gen, T_gen_max, T_gen_max_w, w_gen_max, w_gen_max_T, N_gear, eff_gear, kp, ki, dt, t_end
from src.simulation.functions import path


kite = Kite(S, m, vol)
trubine = Turbine(r_turb, J_gen, T_gen_max, T_gen_max_w, w_gen_max, w_gen_max_T, N_gear, eff_gear, kp, ki)
kiteSystem = FullSystemModel(kite, trubine)

# simulation params
v_current_i = np.array([2,0,0])
p0 = 0
pdot0 = 0.2
w0_gen = 1900 / 60 * 2 * math.pi
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
alphas_pc = np.array(kiteSystem.data_log["alpha_pc"])
alphas_pb = np.array(kiteSystem.data_log["alpha_pb"])
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
P_gen_out = np.array(kiteSystem.turbine.data_log["P_gen_out"])
# vs_rel = np.array(kiteSystem.turbine.data_log["vs_rel"])

# print(ts1.shape == ts.shape)

# Get linear positions
xs, ys, zs = rs.T

####### Kite kinematic plots ######
fix, ax = plt.subplots(3,2, figsize=(15,9))

ax[0,0].plot(sol.t, sol.y[0] % (4*math.pi))
ax[0,0].set_title(r"$p(t)$")
ax[0,0].grid()

ax[1,0].plot(sol.t, sol.y[1])
ax[1,0].set_title(r"$\dot{p}(t)$")
ax[1,0].grid()

ax[2,0].plot(ts, xs)
ax[2,0].plot(ts, ys)
ax[2,0].plot(ts, zs)
ax[2,0].set_title(r"Linear positions")
ax[2,0].legend([r"$x$", r"$y$", r"$z$"])
ax[2,0].set_xlabel("Time [s]")
ax[2,0].grid()

ax[0,1].plot(ts, np.linalg.norm(vs_kite_i, axis=1))
ax[0,1].plot(ts, np.linalg.norm(vs_rel_i, axis=1))
# ax[1,1].plot(ts, np.linalg.norm(v_current_i, axis=1))
ax[0,1].axhline(y=np.linalg.norm(v_current_i), color='g')
ax[0,1].set_title("Linear velocities")
# ax[0,1].set_xlabel("Time [s]")
ax[0,1].legend([r"$|v_{kite,i}|$", r"$|v_{rel,i}|$", r"$|v_{current,i}|$"])
ax[0,1].grid()

ax[1,1].plot(ts, alphas_pc * 180 / math.pi)
ax[1,1].plot(ts, alphas_pb * 180 / math.pi)
ax[1,1].plot(ts, alphas * 180 / math.pi)
ax[1,1].set_title("Alphas")
ax[1,1].legend([r"$\alpha_{pc}$", r"$\alpha_{pb}$", r"$\alpha$ (AoA)"])
ax[1,1].set_ylabel("Angle [deg]")
ax[1,1].set_xlabel("Time [s]")
ax[1,1].grid()

plt.tight_layout(pad=1.0)


###### plot forces ######
fig, ax = plt.subplots(2,2, figsize=(15,9))
ax[0,0].plot(ts, np.linalg.norm(Fs_aero_i, axis=1))
ax[0,0].plot(ts, np.linalg.norm(Fs_buoy_i, axis=1))
ax[0,0].plot(ts, np.linalg.norm(Fs_grav_i, axis=1))
ax[0,0].plot(ts, Fs_turb)
ax[0,0].plot(ts, np.linalg.norm(Fs_tot_i, axis=1))
ax[0,0].plot(ts, Fs_thether)
ax[0,0].set_title(r"Forces over time")
ax[0,0].legend([r"$F_{aero}$", r"$F_{buoy}$", r"$F_{grav}$", r"$F_{turb}$", r"$F_{tot}$", r"$F_{thether}$"])
ax[0,0].set_xlabel("Time [s]")
ax[0,0].grid()

plt.tight_layout(pad=1.0)



##### plot generator stats ####
fig, ax = plt.subplots(4,2, figsize=(15,9))

ax[0,0].plot(sol.t, sol.y[2] * 60 / (2*math.pi))
# ax[0].plot(sol.t, np.ones(len(sol.t)) * w_ref)
ax[0,0].plot(ts, ws_ref * 60 / (2*math.pi))
ax[0,0].set_title(r"Rotor speed $\omega_{gen}$")
ax[0,0].legend([r"$\omega_{gen}$", r"$\omega_{gen,ref}$"])
ax[0,0].grid()

ax[1,0].plot(sol.t, sol.y[3])
ax[1,0].plot(ts, errors)
ax[1,0].set_title(r"PI errors")
ax[1,0].legend([r"$I$", r"$\omega_{gen,ref} - \omega_{gen}$"])
ax[1,0].grid()

ax[2,0].plot(ts, -Ts_gen_el)
ax[2,0].plot(ts, Ts_gen_mech)
ax[2,0].axhline(y=0, color='k', linestyle='--', linewidth=0.7)
ax[2,0].set_title(r"Torques")
ax[2,0].legend([r"$T_{gen,el}$", r"$T_{gen,mech}$"])
ax[2,0].grid()

ax[3,0].plot(ts, Fs_turb)
ax[3,0].set_title(r"$F_{turb}$")
ax[3,0].grid()

ax[0,1].plot(ts, TSRs)
ax[0,1].set_title("TSR")
ax[0,1].grid()

ax[1,1].plot(ts, vs_rel_abs)
ax[1,1].plot(ts, ws_ref-200)
ax[1,1].set_title(r"$v_{rel}$ and $\omega_{gen,ref}$")
ax[1,1].legend([r"$v_{rel}$", r"$\omega_{gen,ref}(moved)$"])
ax[1,1].grid()

ax[2,1].plot(ts, P_gen_out/1000)
ax[2,1].axhline(y=0, color='k', linestyle='--', linewidth=0.7)
ax[2,1].set_title(r"Generator power [kW]")
ax[2,1].grid()

plt.tight_layout(pad=1.0)


# ax[2,0].plot(sol.t, sol.y[2]* 60 / (2*math.pi))
# ax[2,0].plot(ts, ws_ref * 60 / (2*math.pi))
# ax[2,0].legend([r"$\omega_{gen}$", r"$\omega_{gen,ref}$"])
# ax[2,0].set_title(r"$\omega_{gen}(t)$")

# ax[3,0].plot(sol.t, sol.y[3])
# ax[3,0].set_title(r"$I(t)$")

# ax[0,1].plot(ts, np.linalg.norm(Fs_aero_i, axis=1))
# ax[0,1].plot(ts, np.linalg.norm(Fs_buoy_i, axis=1))
# ax[0,1].plot(ts, np.linalg.norm(Fs_grav_i, axis=1))
# ax[0,1].plot(ts, Fs_turb)
# ax[0,1].plot(ts, np.linalg.norm(Fs_tot_i, axis=1))
# ax[0,1].plot(ts, Fs_thether)
# ax[0,1].set_title(r"Forces over time")
# ax[0,1].legend([r"$F_{aero}$", r"$F_{buoy}$", r"$F_{grav}$", r"$F_{turb}$", r"$F_{tot}$", r"$F_{thether}$"])
# ax[0,1].set_xlabel("Time [s]")

# ax[1,1].plot(ts, np.linalg.norm(vs_kite_i, axis=1))
# ax[1,1].plot(ts, np.linalg.norm(vs_rel_i, axis=1))
# # ax[1,1].plot(ts, np.linalg.norm(v_current_i, axis=1))
# ax[1,1].axhline(y=np.linalg.norm(v_current_i))
# ax[1,1].set_title("Linear velocities")
# ax[1,1].set_xlabel("Time [s]")
# ax[1,1].legend([r"$|v_{kite,i}|$", r"$|v_{rel,i}|$", r"$|v_{current,i}|$"])

# ax[2,1].plot(ts, P_gen_out/1000)
# ax[2,1].axhline(y=0, color='k', linestyle='--', linewidth=0.7)
# ax[2,1].set_title(r"Generator power [kW]")

# ax[3,1].plot(ts, alphas_pc * 180 / math.pi)
# ax[3,1].plot(ts, alphas_pb * 180 / math.pi)
# ax[3,1].plot(ts, alphas * 180 / math.pi)
# ax[3,1].set_title("Alphas")
# ax[3,1].legend([r"$\alpha_{pc}$", r"$\alpha_{pb}$", r"$\alpha$ (AoA)"])
# ax[3,1].set_ylabel("Angle [deg]")
# ax[3,1].set_xlabel("Time [s]")



plt.show()