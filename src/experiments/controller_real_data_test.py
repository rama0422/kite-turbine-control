import numpy as np
import math
import pandas as pd
import matplotlib.pyplot as plt

from src.utility.configs import P_mean_init, F_tether_mean_init, og_controller_div_factor, og_controller_tsr_const
from src.controllers.og_controller import OgController


# use real P, F and w_ref data
df = pd.read_csv('Data/KiteData_20231202_f4_flood.csv', delimiter= ";")


columns = df.columns

for column in columns:
    df[column] = df[column].str.replace(',', '.', regex=False).astype(float)

df_subset = df.loc[7000*50:7100*50, ["time", "Power", "TetherForce", "TargetSpeed"]]

ts = df_subset["time"].to_numpy()
Ps = df_subset["Power"].to_numpy() * 1e3
Fs = df_subset["TetherForce"].to_numpy()
ws_ref_gt = df_subset["TargetSpeed"].to_numpy()

P_mean = np.mean(Ps)
F_mean = np.mean(Fs)
print("P: ", P_mean, "F: ", F_mean)
# print(len(w_ref_gt))

og_controller = OgController(P_mean_init, F_tether_mean_init, og_controller_div_factor, og_controller_tsr_const)

ws_ref = []

for i in range(len(Ps)):
    P = Ps[i]
    F = Fs[i]

    w_ref = og_controller.getSpeedRef(P, F)
    ws_ref.append(w_ref)

Ps_running_mean = np.array(og_controller.data_log["P_running_mean"])
Fs_running_mean = np.array(og_controller.data_log["F_tether_running_mean"])

fig, ax = plt.subplots(3,1, figsize=(8,7))

ax[0].plot(ts, Ps)
ax[0].plot(ts, Ps_running_mean)
ax[0].set_title("Power")
ax[0].grid()

ax[1].plot(ts, Fs)
ax[1].plot(ts, Fs_running_mean)
ax[1].set_title("Tether force")
ax[1].grid()

ax[2].plot(ts, ws_ref)
ax[2].plot(ts, ws_ref_gt)
ax[2].set_title("Speed reference")
ax[2].legend([r"$\omega_{ref,generated}$", r"$\omega_{ref,gt}$"])
ax[2].grid()

plt.tight_layout()
plt.show()



