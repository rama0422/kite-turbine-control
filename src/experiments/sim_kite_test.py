import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

from src.simulation.kite_model import Kite
from src.utility.configs import rho, g, S, m, vol, dt, t_end


kite = Kite(S, m, vol)

# Simulation params
v_current_i = np.array([2,0,0])
F_turb = 0
p0 = 0
pdot0 = 0.2
x0 = [p0, pdot0]

sol = solve_ivp(lambda t, x: kite.kiteDynamics(t, x, v_current_i, F_turb), [0, t_end], x0, max_step = dt)
# sol = solve_ivp(lambda t, x: turbine.turbineDynamics(t, x, v_rel_fun(t), w_ref_fun(t)), [0, t_end], x0, max_step = dt)

print(sol)

# plot
ts = np.array(kite.data_log["ts"])
rs = np.array(kite.data_log["r"])
rs_p = np.array(kite.data_log["r_p"])
rs_pp= np.array(kite.data_log["r_pp"])
vs_kite_i = np.array(kite.data_log["v_kite_i"])
vs_rel_i = np.array(kite.data_log["v_rel_i"])
vs_rel_abs = np.array(kite.data_log["v_rel_abs"])
alphas_pc = np.array(kite.data_log["alpha_pc"])
alphas_pb = np.array(kite.data_log["alpha_pb"])
alphas = np.array(kite.data_log["alpha"])
Fs_aero_i = np.array(kite.data_log["Fs_aero"])
Fs_grav_i = np.array(kite.data_log["Fs_grav"])
Fs_buoy_i = np.array(kite.data_log["Fs_buoy"])
Fs_tot_i = np.array(kite.data_log["Fs_tot"])
Fs_thether = np.array(kite.data_log["Fs_thether"])


# print(vs_kite_i.shape)
fig, ax = plt.subplots(4,2, figsize=(15,9))

ax[0,0].plot(sol.t, sol.y[0])
# ax[0].plot(sol.t, np.ones(len(sol.t)) * w_ref)
ax[0,0].set_title(r"$p(t)$")
ax[0,0].legend([r"$p$"])
ax[0,0].set_xlabel("Time [s]")

ax[1,0].plot(sol.t, sol.y[1])
ax[1,0].set_title(r"$\dot{p}(t)$")
ax[1,0].set_xlabel("Time [s]")

ax[2,0].plot(ts, np.linalg.norm(vs_kite_i, axis=1))
ax[2,0].set_title("Kite speed over time")
ax[2,0].set_xlabel("Time [s]")

# plt.subplot(5,1,4)
ax[3,0].plot(ts, alphas * 180 / math.pi)
ax[3,0].set_title("Angle of attack over time")
ax[3,0].set_xlabel("Time [s]")

ax[0,1].plot(ts, np.linalg.norm(Fs_aero_i, axis=1))
ax[0,1].plot(ts, np.linalg.norm(Fs_buoy_i, axis=1))
ax[0,1].plot(ts, -np.linalg.norm(Fs_grav_i, axis=1))
ax[0,1].set_title(r"Aerodynamic, buoyancy and gravity forces over time")
ax[0,1].legend([r"$F_{aero}$", r"$F_{buoy}$", r"$F_{grav}$"])
ax[0,1].set_xlabel("Time [s]")

ax[1,1].plot(ts, Fs_thether)
ax[1,1].set_title("Thether force over time")
ax[1,1].set_xlabel("Time [s]")

fig.tight_layout(pad=1.0)


plt.tight_layout()
plt.show()
