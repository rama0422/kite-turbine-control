import numpy as np
import math
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

def Cp(TSR):
    y = -0.4993241 + 0.3509036*TSR - 0.04359362*TSR**2 + 0.001469591*TSR**3 #https://www.researchgate.net/figure/A-typical-Cp-A-l-for-a-tidal-turbine-4-Only-indicative-values-are-shown-here-these_fig4_340372849
    return max(y,0)


N_gear = 1.5 # TODO: does turbine or gen rotate faster
r_turb = 0.5
A_turb = math.pi *r_turb**2

#TODO: J depends on H which depends on generetor specififc stuff
# H_gen = 
J_gen = 5

eff_gear = 0.99

kp = 300
ki = 20
T_gen_el_limit = 2000

rho = 1025

# w_ref = 500 / 60 * 2 * math.pi

# Simulation params
w0_gen = 520 / 60 * 2 * math.pi
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

print_time = 2

def turbineDynamics(t, x):
    w_gen = x[0]
    I = x[1]

    w_turb = w_gen / N_gear

    w_ref = 500 / 60 * 2 * math.pi + math.sin(0.5*t)

    # v_rel = 7
    v_rel = 5 + 1.5*math.sin(0.5*t)
    TSR = (w_turb * r_turb) / v_rel

    T_turb = 1/2 * rho * A_turb * v_rel**2 * r_turb * Cp(TSR)
    F_turb = 1/2 * rho * A_turb * v_rel**2 * Cp(TSR)
    P_turb = 1/2 * rho * A_turb * v_rel**3 * Cp(TSR)

    T_gen_mech = eff_gear * (T_turb / N_gear)

    w_error = w_ref - w_gen
    T_gen_el = kp * w_error + ki * I
    T_gen_el = max(min(T_gen_el, T_gen_el_limit), -T_gen_el_limit)

    if (t <= print_time):
        print("w_gen:", w_gen, " I:", I, " TSR:", TSR, " T_turb:", T_turb, " T_mech:", T_gen_mech, " w_error:", w_error, " T_gen_el:", T_gen_el)
        # *60/(2*math.pi)

    Idot = w_error
    wdot_gen = (T_gen_mech + T_gen_el) / J_gen

    ts.append(t)
    Fs_turb.append(F_turb)
    Ts_gen_el.append(-T_gen_el)
    Ts_gen_mech.append(T_gen_mech)
    ws_ref.append(w_ref)

    return [wdot_gen, Idot]


sol = solve_ivp(turbineDynamics, t_range, x0, max_step = t_step)

print(sol)

fig, ax = plt.subplots(4,1, figsize=(8,12))

ax[0].plot(sol.t, sol.y[0])
# ax[0].plot(sol.t, np.ones(len(sol.t)) * w_ref)
ax[0].plot(ts, ws_ref)
ax[0].set_title(r"$\omega_{gen}$")


ax[1].plot(sol.t, sol.y[1])
ax[1].set_title(r"$I$ (integral error)")

ax[2].plot(ts, Fs_turb)
ax[2].set_title(r"$F_{turb}$")

ax[3].plot(ts, Ts_gen_el)
ax[3].plot(ts, Ts_gen_mech)


plt.show()


