import math
import numpy as np
import matplotlib.pyplot as plt

from src.utility.configs import T_gen_max, T_gen_max_w, w_gen_max, w_gen_max_T
from src.simulation.functions import C_L, C_D, Cp, Cf, MaxTorqueSpeed

# lift and drag coefficients
alphas = np.linspace(-70, 70, 100) * math.pi / 180

C_Ls = [C_L(alpha) for alpha in alphas]
C_Ds = [C_D(alpha, C_L(alpha)) for alpha in alphas]

# fig, ax = plt.subplots(1,1, figsize=(12,8))
fig = plt.figure(figsize=(9,6))

plt.plot(alphas * 180 / math.pi, C_Ls, label=r"$C_L$")
plt.plot(alphas * 180 / math.pi, C_Ds, label=r"$C_D$")
plt.title(r"Aero coefficients$")
plt.xlabel("AoA")
plt.ylabel(r"$C_L$, $C_D$")
plt.legend()
# ax[0].set_ylabel("$C_L$")
plt.grid()

# ax[0].plot(alphas * 180 / math.pi, C_Ls)
# ax[0].plot(alphas * 180 / math.pi, C_Ds)
# ax[0].set_title(r"Lift coefficient $C_L$")
# ax[0].set_xlabel("AoA")
# ax[0].legend(r"$C_L$", r"$C_D$")
# # ax[0].set_ylabel("$C_L$")
# ax[0].grid()

# ax[1].plot(alphas * 180 / math.pi, C_Ds)
# ax[1].set_title(r"Drag coefficient $C_D$")
# ax[1].set_xlabel("AoA")
# ax[1].set_ylabel("$C_D$")
# ax[1].grid()

plt.tight_layout()
# plt.show()


# power and force coefficients
TSRs = np.linspace(0, 7, 100)

C_ps = [Cp(TSR) for TSR in TSRs]
C_fs = [Cf(TSR) for TSR in TSRs]

fig, ax = plt.subplots(1,2, figsize=(15,5))

ax[0].plot(TSRs, C_ps)
ax[0].set_title(r"Power coefficient $C_P$")
ax[0].set_xlabel("Tip speed ratio $\\lambda$")
ax[0].set_ylabel("$C_P$")
ax[0].grid()

ax[1].plot(TSRs, C_fs)
ax[1].set_title(r"Force coefficient $C_F$")
ax[1].set_xlabel("Tip speed ratio $\\lambda$")
ax[1].set_ylabel("$C_F$")
ax[1].grid()

plt.tight_layout()
# plt.show()

## Torque-speed
# fit T-w curve
ws = np.array([w_gen_max_T, w_gen_max])
Ts = np.array([T_gen_max, T_gen_max_w])

m, b = np.polyfit(ws, Ts, 1)

print(m,b)

l = 100
ws_in = np.linspace(0, w_gen_max+200, l)
torques_in = np.ones(l) * -1000

# l = 10000
# # ws_in = np.linspace(1280 / 60 * (2*math.pi), 1340 / 60 * (2*math.pi), l)
# ws_in = np.ones(l) * 1320 / 60 * (2*math.pi)
# # torques_in = np.ones(l) * 0
# torques_in = np.linspace(100,-100,l)

tws = np.column_stack((ws_in, torques_in))
# print(tws)
torques_out, ws_out = zip(*[MaxTorqueSpeed(tw[1], tw[0], T_gen_max, T_gen_max_w, w_gen_max, w_gen_max_T, m, b) for tw in tws])

fig = plt.figure()

plt.plot(np.array(ws_out) * 60 / (2*math.pi), torques_out)
# plt.ylim([0,T_gen_max+100])
plt.xlabel(r"$\omega$")
plt.ylabel(r"T")
plt.title(r"$T-\omega$ curve")
plt.grid()
plt.show()