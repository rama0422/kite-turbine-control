import numpy as np
import math


class SensorsModel:
    def __init__(self, noise_configs):
        self.noise_config = noise_configs

        self.noise_measurments = {"Elevation": [], 
                                  "TetherForce": [],
                                  "TJPitchAngle": [],
                                  "GeneratorSpdRpm": [],
                                  "Power": [],
                                  "Torque": []}

    # Add gausian noise and bias to simulation measurments
    def addNoise(self, gt_measurments):
        noised_dict = {}
        for key, value in gt_measurments.items():
            mean = self.noise_config[key][0]
            std = self.noise_config[key][1]
            noised = value + np.random.normal(mean, std)

            noised_dict[key] = noised
            self.noise_measurments[key].append(noised)
        
        return noised_dict

    def addDelay(self):
        pass #TODO: do we want this?