import math
import numpy as np
import matplotlib.pyplot as plt

from src.simulation.functions import C_L, C_D, Cp, Cf

# lift and drag coefficients
alphas = np.linspace(-100, 100, 100) * math.pi / 180

C_Ls = [C_L(alpha) for alpha in alphas]
C_Ds = [C_D(alpha) for alpha in alphas]

fig, ax = plt.subplots(1,2, figsize=(10,5))

ax[0].plot(alphas * 180 / math.pi, C_Ls)
ax[0].set_title(r"Lift coefficient $C_L$")
ax[0].set_xlabel("AoA")
ax[0].set_ylabel("$C_L$")

ax[1].plot(alphas * 180 / math.pi, C_Ds)
ax[1].set_title(r"Drag coefficient $C_D$")
ax[1].set_xlabel("AoA")
ax[1].set_ylabel("$C_D$")

plt.tight_layout()
plt.show()


# power and force coefficients
TSRs = np.linspace(0, 7, 100)

C_ps = [Cp(TSR) for TSR in TSRs]
C_fs = [Cf(TSR) for TSR in TSRs]

fig, ax = plt.subplots(1,2, figsize=(10,5))

ax[0].plot(TSRs, C_ps)
ax[0].set_title(r"Power coefficient $C_P$")
ax[1].set_xlabel("Tip speed ratio $\\lambda$")
ax[1].set_ylabel("$C_P$")

ax[1].plot(TSRs, C_fs)
ax[1].set_title(r"Force coefficient $C_F$")
ax[1].set_xlabel("Tip speed ratio $\\lambda$")
ax[1].set_ylabel("$C_F$")

plt.tight_layout()
plt.show()