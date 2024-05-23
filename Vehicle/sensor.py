import numpy as np
from Vehicle.satelliteConstants import *
from Simulator.simulationConstants import *
from Simulator.util import *

class Sensor:
    def __init__(self, type, ideal_bool, star_num):
        self.mass = SENSOR_MASS
        self.type = type
        self.ideal = ideal_bool # Introudce bias and noise or not
        self.bias = 0.0
        self.noise = 0.0
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
                self.bias = STAR_TRACKER_BIAS
                self.noise = STAR_TRACKER_NOISE
        
            if (self.type == "Gyroscope"):
                self.bias = IMU_BIAS
                self.noise = IMU_NOISE

    # Returns a singular estimated rotation observation at a given time
    def get_sensor_observation(self, state):
        current_state = state 
        current_pos = current_state[0:3] # 3D Vector in ECI? (JCI?)
        current_rot = current_state[6:10] # Quaternions
        Inertial2Body = np.linalg.inv(q2R(current_rot))#

        # Different for each type of sensors
        guassian_sd = self.noise
        guassian_bias = self.bias
        noise = np.random.normal(guassian_bias, guassian_sd, np.shape(current_pos))

        if (self.type == "Sun Sensor"):
            SAT2SUN = current_pos + JUPITER2SUN
            sun_direction = normalize_vector(SAT2SUN)
            sun_direction = np.dot(Inertial2Body, sun_direction) # We need it in body frame

            # sun_direction = SunSensor_hardwaremodel(self)
            noisy_sun = sun_direction + noise
            return noisy_sun
         
        if (self.type == "Star Tracker"):
            if (self.star_num == 1):
                SAT2STAR = current_pos + JUPITER2STAR
            if (self.star_num == 2):
                SAT2STAR = current_pos + JUPITER2STAR2
            star_direction = normalize_vector(SAT2STAR)
            star_direction = np.dot(Inertial2Body, star_direction) # We need it in body frame

            # sun_direction = StarTracker_hardwaremodel(self)
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

    def SunSensor_hardwaremodel(self, sun_vector):
        
        alpha = 0 # From Manufacturer
        S = 0 # From Manufacturer
        theta = 0 
        I = alpha * S * np.cos(theta)

        return I
    
    def StarTracker_hardwaremodel(self, x_ccd, y_ccd):

        # Camera intrinsic parameters
        f = STARTRACKER_FOCAL 
        x0, y0 = STARTRACKER_CENTROID
        dy, dx = STARTRACKER_PIXEL
        k = STARTRACKER_DISTORTION

        xdotdot = x_ccd - x0
        ydotdot = y_ccd - y0

        xdot = xdotdot
        ydot = ydotdot * (dy/dx) #Length of pixel along y and along x

        ddot = np.sqrt(xdot**2 + ydot**2)

        x = (1 + k*ddot**2) * xdot
        y = (1 + k*ddot**2) * ydot

        # Vector from centroid (of image plane) to pinhole
        r_hat = np.linalg.norm(np.array([x, y, f]))

        return r_hat
    
    def IMU_hardwaremodel(self):
        deltaTime = 0
        c = 0
        A = 0

        w = deltaTime * c**2 / (4*A)
        return w

        
        
        
        
