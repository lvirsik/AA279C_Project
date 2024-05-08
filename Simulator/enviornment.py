import numpy as np
from Simulator.util import *
from Simulator.enviornmentConstants import *


def get_gravGrad_Torque(state, satellite):
    R = q2R(state)
    
    r = state[0:3]
    r_norm = np.linalg.norm(r)
    R = q2R(state)
    r_body_axes = np.dot(np.linalg.inv(R), r)
    c = r_body_axes / r_norm
    mu = MU_JUPITER
    
    torque = (3 * mu / (r_norm ** 3)) * np.cross(np.dot(satellite.I, c), c)
    return torque

def get_magnetic_Torque(state, satellite, t):
    R = q2R(state)
    r = state[0:3]
    r_norm = np.linalg.norm(r)
    r_unit = np.array(normalize_vector(r))

    # Get Satellite Dipole in body frame
    m = satellite.get_magnetic_dipole()
    # Get magnetic field
    jupiter_theta = get_jupiter_theta(t)
    R_JCJF2JCI = np.array([[np.cos(jupiter_theta), -np.sin(jupiter_theta), 0],
                           [np.sin(jupiter_theta), np.cos(jupiter_theta), 0],
                           [0, 0, 1]])
    dipole_direction_JCJF = np.array([np.cos(JUPITER_DIPOLE_ANGLE_OFFSET), 0, np.sin(JUPITER_DIPOLE_ANGLE_OFFSET)])
    jupiter_dipole_direction = np.dot(R_JCJF2JCI, dipole_direction_JCJF)
    B_JCI = -((R_JUPITER**3)*(B0_JUPITER)/(r_norm**3)) * ((3 * np.dot(jupiter_dipole_direction, r_unit) * r_unit) - jupiter_dipole_direction)
    B_body = np.dot(np.linalg.inv(R), B_JCI)
    
    torque = np.cross(m, B_body)
    return torque
    
def get_jupiter_theta(t):
    return STARTING_THETA_JUPITER + (2 * np.pi * (t / ROT_PERIOD_JUPITER))
    
    