import numpy as np
from Simulator.simulationConstants import *
from Graphics.visualizeEllipsoids import *

def orbital_dynamics(ECI):
    rECI = ECI[0:3]
    vECI = ECI[3:6]
    
    mu = MU_JUPITER
    r = np.linalg.norm(rECI)

    statedot = np.zeros((6))
    statedot[0:3] = vECI # put velocity into xdot position
    statedot[3:6] = (-mu / r**2) * np.array([rECI[0]/r, rECI[1]/r, rECI[2]/r]) # Forces on Object
    
    return statedot

def rotational_dynamics(state, satellite, dt):
    
    ### TODO: state[6:9] is EULER ANGLES and state[9:12] is W VECTOR AND THIS IS NOT CORRECT
    
    w = state[9:12]
    torque = np.array([0,0,0])
    I_dot = (satellite.I_principle - satellite.I_principle_prev) / dt
    alphas = np.dot(np.linalg.inv(satellite.I_principle), torque - np.cross(w, np.dot(satellite.I_principle, w)) - np.dot(I_dot, w))
    
    statedot = np.zeros((6))

    statedot[0:3] = w
    statedot[3:6] = alphas
    return statedot