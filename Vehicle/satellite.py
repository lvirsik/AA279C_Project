from Vehicle.satelliteConstants import *
from Simulator.util import *
from Vehicle.rotor import *
from Vehicle.sensor import *
from Simulator.enviornmentConstants import *
from Vehicle.actuators import *
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
        #self.rotor = Rotor()

        # Initialize Sensors (We have 2 star trackers, sun seonsrs and IMUs) True = ideal, False = not ideal
        self.starTracker1 = Sensor("Star Tracker", True, 1)
        self.starTracker2 = Sensor("Star Tracker", True, 2)
        self.starTracker3 = Sensor("Star Tracker", True, 3)
        self.starTracker4 = Sensor("Star Tracker", True, 4)
        self.sunSensor1 = Sensor("Sun Sensor", True, 1)
        self.sunSensor2 = Sensor("Sun Sensor", True, 1)
        self.imu1 = Sensor("Gyroscope", True, 1)
        self.imu2 = Sensor("Gyroscope", True, 1)

        self.surfaces = SURFACES
        self.Cs = Cs
        self.Cd = Cd
        self.Ca = Ca
        
        self.Cdrag = COEFF_DRAG

        self.gg_torque_history = np.zeros((1, 3))
        self.mag_torque_history = np.zeros((1, 3))
        self.srp_torque_history = np.zeros((1,3))
        self.drag_torque_history = np.zeros((1,3))
        self.torque_history = np.zeros((1,3))
        
        self.actuator_torques = np.array([0,0,0])
        self.x_act = ReactionWheel(FINAL_TIME, TIMESTEP)
        self.y_act = ReactionWheel(FINAL_TIME, TIMESTEP)
        self.z_act = ReactionWheel(FINAL_TIME, TIMESTEP)
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
    
    def get_magnetic_dipole(self):
        """ IN BODY FRAME"""
        return np.array([0, mu0 * NUM_COILS * SURF_IN_COIL * CURRENT, 0])
    
    def set_actuators(self, u, t):
        self.actuator_torques = np.array([self.x_act.get_torque(u[0], t), self.y_act.get_torque(u[1], t), self.z_act.get_torque(u[2], t)]) * 20