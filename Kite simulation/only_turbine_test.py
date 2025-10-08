import numpy as np
import math
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

def Cp(TSR):
    return -0.4993241 + 0.3509036*TSR - 0.04359362*TSR**2 + 0.001469591*TSR**3 #https://www.researchgate.net/figure/A-typical-Cp-A-l-for-a-tidal-turbine-4-Only-indicative-values-are-shown-here-these_fig4_340372849


N_gear = 1.5 # TODO: does turbine or gen rotate faster
r_turb = 0.5
A_turb = math.pi *r_turb**2

#TODO: J depends on H which depends on generetor specififc stuff
# H_gen = 
J_gen = 5

eff_gear = 0.99

kp = 200
ki = 50

rho = 1025

w_ref = 500 / 60 * 2 * math.pi

# Simulation params
w0_gen = 520 / 60 * 2 * math.pi
I0 = 0

x0 = [w0_gen, I0]

t_step = 0.02
t_range = (0, 10)


# v_rel = 7 #TODO


ts = []
Fs_turb = []


def turbineDynamics(t, x):
    w_gen = x[0]
    I = x[1]

    w_turb = w_gen / N_gear

    v_rel = 7 + 2*math.sin(0.5*t)
    TSR = (w_turb * r_turb) / v_rel

    T_turb = 1/2 * rho * A_turb * v_rel**2 * r_turb * Cp(TSR)
    F_turb = 1/2 * rho * A_turb * v_rel**2 * Cp(TSR)
    P_turb = 1/2 * rho * A_turb * v_rel**3 * Cp(TSR)

    T_gen_mech = eff_gear * (T_turb / N_gear)

    w_error = w_ref - w_gen
    T_gen_el = kp * w_error + ki * I

    print("w_gen:", w_gen*60/(2*math.pi), " I:", I, " TSR:", TSR, " T_turb:", T_turb, " T_mech:", T_gen_mech, " w_error:", w_error, " T_gen_el:", T_gen_el)

    Idot = w_error
    wdot_gen = (T_gen_mech + T_gen_el) / J_gen

    ts.append(t)
    Fs_turb.append(F_turb)

    return [wdot_gen, Idot]


sol = solve_ivp(turbineDynamics, t_range, x0, max_step = t_step)

print(sol)

fig, ax = plt.subplots(2,1, figsize=(8,12))

ax[0].plot(sol.t, sol.y[0])
ax[0].plot(sol.t, np.ones(len(sol.t)) * w_ref)

ax[1].plot(ts, Fs_turb)

plt.show()


