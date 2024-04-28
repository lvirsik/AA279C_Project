import numpy as np
from Simulator.simulationConstants import *
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
    
    q = normalize_vector(state[6:10])
    
    w = state[10:13]
  
    torque = np.array([0,0,0])
    I_dot = (satellite.I - satellite.I_prev) / dt
    alphas = np.dot(np.linalg.inv(satellite.I), torque - np.cross(w, np.dot(satellite.I, w)) - np.dot(I_dot, w))

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
    