import numpy as np
from Vehicle.satelliteConstants import *
from Simulator.simulationConstants import *
from Simulator.util import *

class Sensor:
    def __init__(self, type, ideal_bool, star_num):
        self.mass = SENSOR_MASS
        self.type = type
        self.ideal = ideal_bool # Introudce bias and noise or not
        self.bias = 0
        self.noise = SENSOR_SIGMA
        self.star_num = star_num

        if (self.ideal == True):
            self.set_sensor_params(True)
        elif (self.ideal == False):
            self.set_sensor_params(False)

    def set_sensor_params(self, bool):
        self.ideal = bool
        if (self.ideal == False):
            if (self.type == "Sun Sensor"):
                self.bias = SUN_SENSOR_BIAS
                self.noise = SUN_SENSOR_NOISE
        
            if (self.type == "Star Tracker"):
                self.bias = STAR_TRACKER_NOISE
                self.noise = STAR_TRACKER_BIAS
        
            if (self.type == "Gyroscope"):
                self.bias = IMU_NOISE
                self.noise = IMU_BIAS

    # Returns a singular estimated rotation observation at a given time
    def reading(self, state):
        current_state = state 
        current_pos = current_state[0:3] # 3D Vector in ECI? (JCI?)
        current_rot = current_state[6:10] # Quaternions
        Inertial2Body = np.linalg.inv(q2R(current_rot))
        guassian_sd = self.noise
        noise = np.random.normal(0, guassian_sd, np.shape(current_pos))

        if (self.type == "Sun Sensor"):
            SAT2SUN = current_pos + JUPITER2SUN
            sun_direction = normalize_vector(SAT2SUN)
            sun_direction = np.dot(Inertial2Body, sun_direction) # We need it in body frame
            noisy_sun = sun_direction + noise
            return noisy_sun
        
        if (self.type == "Star Tracker"):
            if (self.star_num == 1):
                SAT2STAR = current_pos + JUPITER2STAR
            if (self.star_num == 2):
                SAT2STAR = current_pos + JUPITER2STAR2
            if (self.star_num == 3):
                SAT2STAR = current_pos + JUPITER2STAR3
            if (self.star_num == 4):
                SAT2STAR = current_pos + JUPITER2STAR4
            star_direction = normalize_vector(SAT2STAR)
            star_direction = np.dot(Inertial2Body, star_direction) # We need it in body frame
            noisy_star = star_direction + noise
            return noisy_star
        
        if (self.type == "Gyroscope"):
            current_w = current_state[10:13]
            noisy_observation = current_w + noise
            return noisy_observation
          
        else:
            print("Bad Sensor Name Input")

    # Returns estimated rotational history
    def get_est_traj(self, traj):
        rot_history = traj[:, 6:10] # Quaternions
        guassian_sd = self.noise

        noise = np.random.normal(0, guassian_sd, np.shape(rot_history))
        noisy_rot_history = rot_history + noise

        if (self.type == "Sun Sensor"):
            return noisy_rot_history
        
        if (self.type == "Star Tracker"):
            return noisy_rot_history
        
        if (self.type == "Gyroscope"):
            return noisy_rot_history
    
        
        else:
            print("Bad Sensor Name Input")
        
        
        
        
