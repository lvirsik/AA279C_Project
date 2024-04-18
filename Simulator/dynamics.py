import numpy as np
from Simulator.simulationConstants import *
from Graphics.visualizeEllipsoids import *

def OE_2_ECI(oes):
    """ Convert orbital elements to Cartesian in ECI """
    mu = MU_JUPITER
    a = oes[0]
    e = oes[1]
    i = oes[2]
    RAAN = oes[3]
    w = oes[4]
    v = oes[5]
    
    p = a * (1 - e**2)
    r = p / (1 + e*np.cos(v))
    rPQW = np.array([r*np.cos(v), r*np.sin(v), 0]).T
    vPQW = np.array([np.sqrt(mu/p) * -np.sin(v), np.sqrt(mu/p)*(e + np.cos(v)), 0]).T
    
    R1 = np.array([[np.cos(-RAAN), np.sin(-RAAN), 0],
                   [-np.sin(-RAAN), np.cos(-RAAN), 0],
                   [0,       0,       1]])
    R2 = np.array([[1,     0,        0],
                   [0, np.cos(-i), np.sin(-i)],
                   [0, -np.sin(-i), np.cos(-i)]])
    R3 = np.array([[np.cos(-w), np.sin(-w), 0],
                   [-np.sin(-w), np.cos(-w), 0],
                   [0,        0,        1]])
    R = np.dot(np.dot(R1, R2), R3)
    rECI = np.dot(R, rPQW)
    vECI = np.dot(R, vPQW)
    ECI = np.concatenate((rECI, vECI))
    return ECI

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
    w = get_w_from_EulerAngle(state).T[0]
    torque = np.array([0,0,0])
    I_dot = (satellite.I_principle - satellite.I_principle_prev) / dt
    alphas = np.dot(np.linalg.inv(satellite.I_principle), torque - np.cross(w, np.dot(satellite.I_principle, w)) - np.dot(I_dot, w))
    
    statedot = np.zeros((6))
    statedot[0:3] = w
    statedot[3:6] = alphas
    return statedot