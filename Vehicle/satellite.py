from Vehicle.satelliteConstants import *
from Simulator.util import *
from Vehicle.rotor import *
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
        
        # Initialize Rotor
        self.rotor = Rotor()
        
    def calculate_R(self):
        value, vector = np.linalg.eig(self.I)
        R = vector
        
        #Resort to fix from eig
        indices = np.argsort(np.abs(np.diagonal(self.I)))[::-1]
        value = value[indices]
        R = R[:, indices]
        I_prinicple = np.diag(value)
        
        
        # Normalize Matrix
        for i in range(len(R)):
            R[i] = normalize_vector(R[i])
        return I_prinicple, R