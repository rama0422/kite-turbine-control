import numpy as np
import math
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

# plt.close('all')

# def Cp(TSR):
#     y = -0.4993241 + 0.3509036*TSR - 0.04359362*TSR**2 + 0.001469591*TSR**3 #https://www.researchgate.net/figure/A-typical-Cp-A-l-for-a-tidal-turbine-4-Only-indicative-values-are-shown-here-these_fig4_340372849
#     return max(y,0)

# https://www.sciencedirect.com/science/article/pii/S0960148125018798

# https://upcommons.upc.edu/server/api/core/bitstreams/cf859e70-09aa-47d0-9add-931b4a520e36/content
def Cp(TSR):
    x = TSR
    y = -0.1538643 + 0.4473311*x - 0.09631951*x**2 + 0.003482307*x**3
    return max(y,0)

def Cf(TSR):
    x = TSR
    y = 0.88 - 0.1187302*x + 0.02369048*x**2 - 0.004722222*x**3
    return max(y,0)


N_gear = 5 # TODO: does turbine or gen rotate faster
r_turb = 0.5
A_turb = math.pi *r_turb**2

#TODO: J depends on H which depends on generetor specififc stuff
# H_gen = 
J_gen = 5

# P_gen_max = 2000
T_gen_max = 500
w_gen_max = 3000 / 60 * 2 * math.pi
w_gen_max_T = 1500 / 60 * 2 * math.pi

eff_gear = 0.99

kp = 500
ki = 500
T_gen_el_limit = 2000

rho = 1025

# w_ref = 500 / 60 * 2 * math.pi

# Simulation params
w0_gen = 1400 / 60 * 2 * math.pi
I0 = 0

x0 = [w0_gen, I0]

t_step = 0.02
t_range = (0, 100)


# v_rel = 7 #TODO


ts = []
Fs_turb = []
Ts_gen_mech = []
Ts_gen_el =[]
ws_ref = []
errors = []
TSRs = []
vs_rel = []

print_time = 2

def turbineDynamics(t, x):
    w_gen = x[0]
    I = x[1]

    w_turb = w_gen / N_gear

    w_ref = 1500 / 60 * 2 * math.pi + 30*math.sin(0.5*t)

    # v_rel = 7
    v_rel = 5 + 1.5*math.sin(0.5*t)
    TSR = (w_turb * r_turb) / v_rel

    P_turb = 1/2 * rho * A_turb * v_rel**3 * Cp(TSR)
    # T_turb = 1/2 * rho * A_turb * v_rel**2 * r_turb * Cp(TSR) # TODO: should be Cq
    T_turb = P_turb / w_turb if w_turb != 0 else 0
    F_turb = 1/2 * rho * A_turb * v_rel**2 * Cf(TSR)
    

    T_gen_mech = eff_gear * (T_turb / N_gear)

    w_error = w_ref - w_gen
    T_gen_el = kp * w_error + ki * I

# 3000 / 60 * 2 * math.pi

    # T_gen_el = max(min(T_gen_el, T_gen_el_limit), -T_gen_el_limit) # TODO: add speed dependant max torque
    if (w_gen < w_gen_max_T):
        T_gen_el = max(min(T_gen_el, T_gen_max), -T_gen_max)
    else:
        temp_w = w_gen*60/(2*math.pi)
        temp_max_T = 965.0754 - 0.3595477*temp_w + 0.00003567839*temp_w**2
        T_gen_el = max(min(T_gen_el, temp_max_T), -temp_max_T)


    P_gen_out = T_gen_el * w_gen
    #TODO: add loses through Torque-speed curve of generator

    # if (t <= print_time):
    #     print("w_gen:", w_gen, " I:", I, " TSR:", TSR, " T_turb:", T_turb, " T_mech:", T_gen_mech, " w_error:", w_error, " T_gen_el:", T_gen_el)
    #     # *60/(2*math.pi)

    Idot = w_error
    wdot_gen = (T_gen_mech + T_gen_el) / J_gen

    ts.append(t)
    Fs_turb.append(F_turb)
    Ts_gen_el.append(-T_gen_el)
    Ts_gen_mech.append(T_gen_mech)
    ws_ref.append(w_ref)
    errors.append(w_error)
    TSRs.append(TSR)
    vs_rel.append(v_rel)

    return [wdot_gen, Idot]


sol = solve_ivp(turbineDynamics, t_range, x0, max_step = t_step)

print(sol)

fig, ax = plt.subplots(4,2, figsize=(15,9))

ax[0,0].plot(sol.t, sol.y[0])
# ax[0].plot(sol.t, np.ones(len(sol.t)) * w_ref)
ax[0,0].plot(ts, ws_ref)
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
ax[1,1].plot(ts, np.array(ws_ref)-150)
ax[1,1].set_title(r"$v_{rel}$ and $\omega_{gen,ref}$")
ax[1,1].legend([r"$v_{rel}$", r"$\omega_{gen,ref}$"])



plt.tight_layout()
plt.show()


