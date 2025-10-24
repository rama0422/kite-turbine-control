import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

from src.simulation.turbine_model import Turbine
from src.utility.configs import r_turb, J_gen, J_turb, T_gen_max, T_gen_max_w, w_gen_max, w_gen_max_T, w_ref_base, w_limit, N_gear, eff_gear, kp, ki, rho, dt, t_end



turbine = Turbine(r_turb, J_gen, J_turb, T_gen_max, T_gen_max_w, w_gen_max, w_gen_max_T,w_limit, N_gear, eff_gear, kp, ki)


# time varying v_rel and w_ref functions
def v_rel_fun(t):
    return 5 + 1.5*math.sin(0.5*t)

def w_ref_fun(t):
    return w_ref_base + 10*math.sin(0.5*t)


# Simulation params
w0_gen = 1900 / 60 * 2 * math.pi
I0 = 0
x0 = [w0_gen, I0]

sol = solve_ivp(lambda t, x: turbine.turbineDynamics(t, x, v_rel_fun(t), w_ref_fun(t)), [0, t_end], x0, max_step = dt)

print(sol)

# print(len( turbine.data_log["ts"]))
ts = np.array(turbine.data_log["ts"])
Fs_turb = np.array(turbine.data_log["Fs_turb"])
Ts_gen_mech = np.array(turbine.data_log["Ts_gen_mech"])
Ts_gen_el = np.array(turbine.data_log["Ts_gen_el"])
ws_ref = np.array(turbine.data_log["ws_ref"])
errors = np.array(turbine.data_log["errors"])
TSRs = np.array(turbine.data_log["TSRs"])
P_gen_out = np.array(turbine.data_log["P_gen_out"])
vs_rel = np.array(turbine.data_log["vs_rel"])

fig, ax = plt.subplots(4,2, figsize=(15,9))

ax[0,0].plot(sol.t, sol.y[0] * 60 / (2*math.pi))
# ax[0].plot(sol.t, np.ones(len(sol.t)) * w_ref)
ax[0,0].plot(ts, ws_ref * 60 / (2*math.pi))
ax[0,0].set_title(r"Rotor speed $\omega_{gen}$")
ax[0,0].legend([r"$\omega_{gen}$", r"$\omega_{gen,ref}$"])


ax[1,0].plot(sol.t, sol.y[1])
ax[1,0].plot(ts, errors)
ax[1,0].set_title(r"PI errors")
ax[1,0].legend([r"$I$", r"$\omega_{gen,ref} - \omega_{gen}$"])

ax[2,0].plot(ts, Ts_gen_el)
ax[2,0].plot(ts, Ts_gen_mech)
ax[2,0].axhline(y=0, color='k', linestyle='--', linewidth=0.7)
ax[2,0].set_title(r"Torques")
ax[2,0].legend([r"$T_{gen,el}$", r"$T_{gen,mech}$"])

ax[3,0].plot(ts, Fs_turb)
ax[3,0].set_title(r"$F_{turb}$")

ax[0,1].plot(ts, TSRs)
ax[0,1].set_title("TSR")

ax[1,1].plot(ts, vs_rel)
ax[1,1].plot(ts, ws_ref-200)
ax[1,1].set_title(r"$v_{rel}$ and $\omega_{gen,ref}$")
ax[1,1].legend([r"$v_{rel}$", r"$\omega_{gen,ref}(moved)$"])

ax[2,1].plot(ts, P_gen_out/1000)
ax[2,1].axhline(y=0, color='k', linestyle='--', linewidth=0.7)
ax[2,1].set_title(r"Generator power [kW]")


plt.tight_layout()
plt.show()
