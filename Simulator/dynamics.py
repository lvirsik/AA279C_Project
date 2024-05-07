import numpy as np
from Simulator.simulationConstants import *
from Simulator.util import *
from Graphics.visualizeEllipsoids import *
import cmath

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
    """ Done in Body Axes"""
    
    # Check for rotor
    if hasattr(satellite, 'rotor'):
        # For now, assume torque on rotor is zero
        rotor_I = satellite.rotor.I
        rotor_r = satellite.rotor.direction
        wr = satellite.rotor.angular_speed
        wr_dot = 0
    else:
        rotor_I = 0
        rotor_r = np.array([0,0,0])
        wr = 0
        wr_dot = 0
        
    
    q = normalize_vector(state[6:10])
    w = state[10:13]
    R = q2R(state)

    torque = np.array([0.0,0.0,0.0])
    torque += get_gravGrad_T(state, satellite)
    satellite.torque_history.append(torque)

    I_dot = (satellite.I - satellite.I_prev) / dt
    alphas = np.dot(np.linalg.inv(satellite.I), torque - 
                    np.cross(w, np.dot(satellite.I, w)) - 
                    np.dot(I_dot, w) -
                    rotor_I * wr_dot * rotor_r - 
                    rotor_I * wr * np.cross(w, rotor_r))

    statedot = np.zeros((7))
    statedot[0:4] = qdot(q, w)

    statedot[4:7] = alphas
    return statedot

def qdot(q, w):
    wx = w[0]
    wy = w[1]
    wz = w[2]
    big_omega = np.array([[0, wz, -wy, wx],
                          [-wz, 0, wx, wy],
                          [wy, -wx, 0, wz],
                          [-wx, -wy, -wz, 0]])
    qdot = 0.5 * np.dot(big_omega, q)
    return qdot

def EAdot(EAs, w):
    wx = w[0]
    wy = w[1]
    wz = w[2]
    phi = EAs[0]
    theta = EAs[1]
    psi = EAs[2]
    phidot = ((wx*np.sin(psi)) + (wy*np.cos(psi)))/np.sin(theta)
    thetadot = (wx*np.cos(psi)) - (wy*np.sin(psi))
    psidot = wz - ((wx*np.sin(psi)) + wy*np.cos(psi))/np.tan(theta)
    return np.array([phidot, thetadot, psidot])

def axially_symmetric_analytical_solution(t, I, state_initial):
    Ix = I[0,0]
    Iy = I[1,1]
    Iz = I[2,2]
    
    wz0 = state_initial[2]
    wy0 = state_initial[1]
    wx0 = state_initial[0]

    lambd = (Iz - Ix) * wz0 / Ix
    
    wxy = (wx0 + 1j*wy0)*np.e**(1j*lambd*t)
    wy = np.imag(wxy)
    wx = np.real(wxy)
    wz = wz0
    
    return np.array([wx, wy, wz])

def get_gravGrad_T(state, satellite):
    R = q2R(state)
    RTN_state = calculate_RTN(state)
    principle_axis = (R*satellite.R).T

    angles_R2P = angles_between_matrix(RTN_state, principle_axis)

    a = INITIAL_OEs[0] * 100
    mu = MU_JUPITER
    n = np.sqrt(mu/a**3)
    
    c = [1.0, -angles_R2P[2], angles_R2P[1]]
    torque = 3*(n**2)*(np.cross(c, np.dot(satellite.I, c)))

    return torque
    