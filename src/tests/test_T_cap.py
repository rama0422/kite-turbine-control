import unittest
import numpy as np
import math

from src.simulation.functions import T_cap
from src.utility.configs import *

ws = np.array([w_gen_max_T, w_gen_max])
Ts = np.array([T_gen_max, T_gen_max_w])
m, b = np.polyfit(ws, Ts, 1)

class TestTCap(unittest.TestCase):
    def testInNormalRegion(self):
        T_in = np.array([300, 10, 389])
        w_in = np.array([2000, 1, 2550]) / 60 * (2 * math.pi)
        T_out_true = T_in

        for i in range(len(T_in)):
            T_out = T_cap(T_in[i], w_in[i], w_limit, w_gen_max, T_gen_max, m, b)

            self.assertAlmostEqual(T_out, T_out_true[i])

    def testInFieldWeakiningRegion(self):
        T_in = np.array([200, 10, 319])
        w_in = np.array([2600, 3100, 3189]) / 60 * (2 * math.pi)
        T_out_true = T_in

        for i in range(len(T_in)):
            T_out = T_cap(T_in[i], w_in[i], w_limit, w_gen_max, T_gen_max, m, b)

            self.assertAlmostEqual(T_out, T_out_true[i])

    def testAboveNormalRegion(self):
        T_in = np.array([403, 450, 1000])
        w_in = np.array([100, 2500, 2000]) / 60 * (2 * math.pi)
        T_out_true = np.array([T_gen_max, T_gen_max, T_gen_max])

        for i in range(len(T_in)):
            T_out = T_cap(T_in[i], w_in[i], w_limit, w_gen_max, T_gen_max, m, b)

            self.assertAlmostEqual(T_out, T_out_true[i])

    def testAboveFieldWeakiningRegion(self):
        T_in = np.array([380, 380, 250])
        w_in = np.array([2900, 3500, 4500]) / 60 * (2 * math.pi)
        T_out_true = m * w_in + b

        for i in range(len(T_in)):
            T_out = T_cap(T_in[i], w_in[i], w_limit, w_gen_max, T_gen_max, m, b)

            self.assertAlmostEqual(T_out, T_out_true[i])

if __name__ == "__main__":
    unittest.main()

# T_gen_max = 399.43 #Rated torque
# T_gen_max_w = 319.83
# w_gen_max = 3189.99 / 60 * 2 * np.pi
# w_gen_max_T = 2559.61 / 60 * 2 * np.pi

# def T_cap(T_requested,speed_rpm,speedLimit,corner_point_speed,corner_point_torque,m,b):
#     cap_dn = m*speed_rpm + b

#     if((speed_rpm <= speedLimit) & (speed_rpm > 0)):
#         Torque_cap = np.where(speed_rpm <= corner_point_speed,corner_point_torque, np.where(speed_rpm <= speedLimit, np.maximum(0, cap_dn),np.nan))
#         T_capped = np.clip(T_requested, -Torque_cap, Torque_cap)
#     else:
#         # print("Over/under speed: ", speed_rpm)
#         T_capped = 0

#     return T_capped
     