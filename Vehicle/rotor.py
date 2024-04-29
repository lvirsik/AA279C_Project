import numpy as np
from Vehicle.satelliteConstants import *

class Rotor:
    def __init__(self):
        self.mass = ROTOR_MASS  # Rocket Starts Fully Fueled
        self.angular_speed = ROTOR_SPEED
        self.start_direction = ROTOR_DIRECTION
        self.direction = ROTOR_DIRECTION
        
        # Calculate Moment of Inertia Tensor
        self.I = I_ROTOR
        