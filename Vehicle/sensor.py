import numpy as np
from Vehicle.satelliteConstants import *

class Sensor:
    def __init__(self):
        self.mass = SENSOR_MASS
        self.type = SENSOR_TYPE[0]
        self.ideal = 0 # Introudce bias and noise or not
        self.bias = 0
        self.noise = 0
        self.number = 0 # A way to identify each sensor by name

        self.obersvation_history = 0

    def set_sensor_params(self, bool, sensor_type):
        self.ideal = bool
        self.type = sensor_type
        if (self.ideal == True):
            self.bias = SENSOR_BIAS
            self.noise = SENSOR_NOISE
        
