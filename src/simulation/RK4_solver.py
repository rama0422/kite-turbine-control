import numpy as np

class RK4Solver:
    def __init__(self, dynamic_function, x0, t0):
        self.func = dynamic_function
        self.x = np.array(x0)
        self.t = float(t0)

        self.xs = [self.x.copy()]
        self.ts = [self.t]

    def step(self, dt, *args):
        x = self.x
        t = self.t

        k1 = self.func(t, x, False, *args)
        k2 = self.func(t + 0.5 * dt, x + 0.5 * dt * k1, False, *args)
        k3 = self.func(t + 0.5 * dt, x + 0.5 * dt * k2, False, *args)
        k4 = self.func(t + dt, x + dt * k3, True, *args) # log only during the last function call

        self.x = x + (1.0 / 6.0) * (k1 + 2 * k2 + 2* k3 + k4) * dt
        self.t += dt

        self.xs.append(self.x.copy())
        self.ts.append(self.t)

        return self.x
    
    def solveSteps(self, dt, t_end, *args):

        n_steps = (int)((t_end - self.t) / dt)
        # print(n_steps)

        for _ in range(n_steps):
            self.step(dt, *args) # pass water current as args to system function
        
        print("Success")