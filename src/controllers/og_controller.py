import numpy as np
import math


## Controller takes in tether force and produced power and returns a generator speed reference

class OgController:
    def __init__(self, P_mean_init, F_tether_mean_init, div_factor, TSR_const):
        # params
        self.P_mean_init = P_mean_init
        self.F_tether_mean_init = F_tether_mean_init
        self.div_factor = div_factor
        self.TSR_const = TSR_const

        # variables
        self.P_running_mean = self.P_mean_init
        self.F_tether_running_mean = self.F_tether_mean_init

        self.data_log = {"P_running_mean": [],
                         "F_tether_running_mean": [],
                         "w_ref": []}
        
    def getSpeedRef(self, P, F_tether):
        self.updateRunningMeans(P, F_tether)

        W_ref_P_mean = self.TSR_const * self.P_running_mean**(1/3)

        w_ref = (F_tether - self.F_tether_running_mean) / self.div_factor + W_ref_P_mean

        # store data
        self.data_log["P_running_mean"].append(self.P_running_mean)
        self.data_log["F_tether_running_mean"].append(self.F_tether_running_mean)
        self.data_log["w_ref"].append(w_ref)

        return w_ref

    def updateRunningMeans(self, P, F_tether):
        self.P_running_mean = (self.P_running_mean*0.9999 + P*0.0001) #TODO: calculate real means
        self.F_tether_running_mean = (self.F_tether_running_mean*0.9999 + F_tether*0.0001)

