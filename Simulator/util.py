import numpy as np
from Simulator.simulationConstants import *

def get_w_from_EulerAngle(trajectory):
     # 1 2 3 = phi theta psi = yaw pitch roll
    if (np.size(trajectory) == (12)): 
        euler_angles = trajectory[6:9]
        euler_angle_rates = trajectory[9:12]

        theta = euler_angles[1]
        psi = euler_angles[2]
        
        phiDot = euler_angle_rates[0]
        thetaDot = euler_angle_rates[1]
        psiDot = euler_angle_rates[2]
    elif (np.size(trajectory) == (6)): 
        euler_angles = trajectory[0:3]
        euler_angle_rates = trajectory[3:6]

        theta = euler_angles[1]
        psi = euler_angles[2]
        
        phiDot = euler_angle_rates[0]
        thetaDot = euler_angle_rates[1]
        psiDot = euler_angle_rates[2]
    else:
        euler_angles = trajectory[:, 6:9]
        euler_angle_rates = trajectory[:, 9:12]

        theta = euler_angles[:, 1]
        psi = euler_angles[:,2]
        
        phiDot = euler_angle_rates[:, 0]
        thetaDot = euler_angle_rates[:,1]
        psiDot = euler_angle_rates[:, 2]

    # Angular velocity from euler angles and euler angles rates
    w = np.array([[phiDot * np.sin(theta) * np.sin(psi) + thetaDot * np.cos(psi)],
                  [phiDot * np.sin(theta) * np.cos(psi) - thetaDot * np.sin(psi)],
                  [phiDot * np.cos(theta) + psiDot]])
    
    return w

def get_EulerAngle_from_w(w, state):
    phi = state[6]
    theta = state[7]
    psi = state[8]
    
    phidot = (w[0]*np.sin(psi) + w[1]*np.cos(psi))/np.sin(theta)
    thetadot = w[0]*np.cos(psi) - w[1]*np.sin(psi)
    psidot = w[2] - (w[0]*np.sin(psi) + w[1]*np.cos(psi))*(1/np.tan(theta))
    
    return [phidot, thetadot, psidot]

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