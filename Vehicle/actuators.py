import numpy as np
import scipy.signal as signal
from Vehicle.satelliteConstants import *

# class ReactionWheel:
#     def __init__(self):
    
#         self.noise = 0.5
#         self.bias = 0
#         self.max_torque = 75
    
#     def get_torque(self, torque):
#         torque = np.random.normal(torque + self.bias, self.noise)
#         if torque > self.max_torque:
#             return self.max_torque
#         return torque
    
class ReactionWheel:
    def __init__(self, tf, ts):
        # Source: Feiler2003 
        K = 1
        T = 0.1

        self.noise = 0.5
        self.bias = 0
        self.max_torque = 75

        # Create Transfer function
        n = [K]
        d = [T, 1]
        self.sys = signal.TransferFunction(n, d)

        self.X  = np.array([[0.0]]) # Initaial Condistions
        self.T  = np.linspace(0, int(tf/ts)*ts, num=int(tf/ts)+1)
        self.U  = np.array([[0.0]])   
        self.U_real = np.array([[0.0]])

    def get_torque(self, u_current, t):
        
        if u_current > ACTUATOR_MAX_TORQUE:
            u_current = ACTUATOR_MAX_TORQUE
        if u_current < -ACTUATOR_MAX_TORQUE:
            u_current = -ACTUATOR_MAX_TORQUE
                
        
        if not t == 0:
            self.U = np.append(self.U, u_current)
        tout, y, x = signal.lsim(self.sys, self.U, self.T[:min(enumerate(self.T), key=lambda x: abs(x[1]-t))[0] + 1], self.X)
        
        # x and y contains 2 simulation points the newer one is the correct one.
        if not tout[-1] == 0:
            self.X = x[-1] # update state   
            self.U_real = np.append(self.U_real, y[-1])   
            return y[-1] #
        else:
            self.U_real = np.append(self.U_real, self.X[0][0])
            return self.X[0][0] 