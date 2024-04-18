from Vehicle.satelliteConstants import *
import numpy as np

class Satellite:
    def __init__(self):
        self.mass = MASS  # Rocket Starts Fully Fueled
        
        # Calculate Moment of Inertia Tensor
        self.I = I_COM_BF
        self.I_prev = I_COM_BF
        
        # Initialize the satellite's rotation matrix
        self.I_principle, self.R = self.calculate_R()
        self.I_principle_prev = self.I_principle
        
    def calculate_R(self):
        value, vector = np.linalg.eig(self.I)
        I_prinicple = np.diag(value)
        R = vector
        return I_prinicple, R