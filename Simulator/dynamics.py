import numpy as np
from Simulator.simulationConstants import *
from Simulator.util import *
from Graphics.visualizeEllipsoids import *
from Simulator.enviornment import *
import cmath
import copy

def orbital_dynamics(ECI):
    rECI = ECI[0:3]
    vECI = ECI[3:6]
    
    mu = MU_JUPITER
    r = np.linalg.norm(rECI)

    statedot = np.zeros((6))
    statedot[0:3] = vECI # put velocity into xdot position
    statedot[3:6] = (-mu / r**2) * np.array([rECI[0]/r, rECI[1]/r, rECI[2]/r]) # Forces on Object
    
    return statedot

def rotational_dynamics(state, satellite, t, dt, u=np.array([0,0,0])):
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
    
    # Deal with Torques
    gg_torque = get_gravGrad_Torque(state, satellite)
    m_torque = get_magnetic_Torque(state, satellite, t)
    srp_torque = get_SRP_torque(state, satellite, t)
    drag_torque = get_drag_torque(state, satellite, t)
    satellite.gg_torque_history = np.vstack((satellite.gg_torque_history, gg_torque))
    satellite.mag_torque_history = np.vstack((satellite.mag_torque_history, m_torque))
    satellite.srp_torque_history = np.vstack((satellite.srp_torque_history, srp_torque))
    satellite.drag_torque_history = np.vstack((satellite.drag_torque_history, drag_torque))
    torque = gg_torque + m_torque + srp_torque + drag_torque + satellite.actuator_torques + u
    satellite.torque_history = np.vstack((satellite.torque_history, torque))
    
    # Euler Equations
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

def state_space_dynamics_wrapper(state, satellite, t, dt, u):
    positional_state = orbital_dynamics(state[0:6])
    rotational_state = rotational_dynamics(state, satellite, t, dt, u=u)
    return np.hstack((positional_state, rotational_state))

def compute_A(state, u, satellite, t, dt):
    h = 0.0001
    jacobian = np.zeros((len(state), len(state)))
    for i in range(len(state)):
        state_plus = copy.copy(state).astype(float)
        state_minus = copy.copy(state).astype(float)
        state_plus[i] = state_plus[i] + h
        state_minus[i] = state_minus[i] - h
        statedot_plus = state_space_dynamics_wrapper(state_plus, satellite, t, dt, u)
        statedot_minus = state_space_dynamics_wrapper(state_minus, satellite, t, dt, u)
        jacobian[i] = (statedot_plus - statedot_minus) / (2 * h)
    return jacobian.T

def compute_B(state, u, satellite, t, dt):
    h = 0.0001
    jacobian = np.zeros((len(u), len(state)))
    for i in range(len(u)):
        u_plus = copy.copy(u).astype(float)
        u_minus = copy.copy(u).astype(float)
        u_plus[i] = u[i] + h
        u_minus[i] = u[i] - h
        statedot_plus = state_space_dynamics_wrapper(state, satellite, t, dt, u_plus)
        statedot_minus = state_space_dynamics_wrapper(state, satellite, t, dt, u_minus)
        jacobian[i] = (statedot_plus - statedot_minus) / (2 * h)
    return jacobian.T
    