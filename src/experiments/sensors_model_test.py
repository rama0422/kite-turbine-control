import numpy as np
import math
import matplotlib.pyplot as plt

from src.simulation.sensors_model import SensorsModel


# mean, variance and bias for each sensor/measurment #TODO: mean and bias? one enough right?
# noise_configs = {"Elevation": [0, 0.1*1e-2*60/3, 0], #https://www.novasub.com/wp-content/uploads/UDPS-Underwater-Depth-Pressure-sensor.pdf
#                  "TetherForce": [0, 0.1*1e-2*200*1e3/3, 0] #https://www.vetek.com/en/dynamics/WebFiles/document/056487f4-2a65-43b2-8ba2-a270cccbc67f/Datasheet_K2145_V1.pdf
#                  }

noise_configs = {"Elevation": [0, 0.2*1e-2*60/3, 0],
                 "TetherForce": [0, 5*1e-2*200*1e3/3, 0]}


gt_measurments = {"Elevation": [4.5*np.sin(p)+13 for p in np.linspace(0,8*math.pi,1000)],
                  "TetherForce": [1e5*np.sin(p+math.pi/3)+1.8e5 for p in np.linspace(0,8*math.pi,1000)]}

sensors_model = SensorsModel(noise_configs)

for i in range(len(gt_measurments["Elevation"])):
    measurments = {"Elevation": gt_measurments["Elevation"][i],
                   "TetherForce": gt_measurments["TetherForce"][i]}

    sensors_model.addNoise(measurments)

fig, ax = plt.subplots(1,2, figsize=(12,6))

ax[0].plot(sensors_model.noise_measurments["Elevation"])
ax[0].plot(gt_measurments["Elevation"], linewidth = 1)
ax[0].grid()

ax[1].plot(sensors_model.noise_measurments["TetherForce"])
ax[1].plot(gt_measurments["TetherForce"], linewidth = 1)
ax[1].grid()

plt.tight_layout()
plt.show()