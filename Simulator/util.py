import numpy as np
import math
from Simulator.simulationConstants import *
from scipy.spatial.transform import Rotation

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

def calculate_L_Inertial(satellite, state):
    w = state[10:13]
    I = satellite.I
    L_BF = np.dot(I, w)

    # Rotate into inertial frame
    q = state[6:10]
    R = q2R(q)
    L_inertial = np.dot(R, L_BF)
    if hasattr(satellite, 'rotor'):
        L_inertial = L_inertial + np.dot(R, (satellite.rotor.I * ((satellite.rotor.angular_speed * satellite.rotor.direction) - w)))
        print(L_inertial)
    return L_inertial
       
def calculate_w_inertial(satellite, state):
    w = state[10:13]
    # Rotate into inertial frame
    q = state[6:10]
    R = q2R(q)
    w_inertial = np.dot(R, w)
    return w_inertial

def calculate_RTN(state):
    r = state[0:3]
    v = state[3:6]
    R = normalize_vector(r)

    T = normalize_vector(v - np.dot((np.dot(r, v))/np.dot(r, r), r))
    N = normalize_vector(np.cross(R, T))
    RTN = np.matrix([R, T, N]).T
    return RTN

def get_EulerAngle_from_w(w, state):
    phi = state[6]
    theta = state[7]
    psi = state[8]
    
    phidot = (w[0]*np.sin(psi) + w[1]*np.cos(psi))/np.sin(theta)
    thetadot = w[0]*np.cos(psi) - w[1]*np.sin(psi)
    psidot = w[2] - (w[0]*np.sin(psi) + w[1]*np.cos(psi))*(1/np.tan(theta))
    
    return [phidot, thetadot, psidot]

def normalize_vector(vector):
    # Compute the length of the vector
    length = math.sqrt(sum(component**2 for component in vector))
    
    # Normalize each component
    normalized_vector = [component / length for component in vector]
    
    return normalized_vector

def R2q(m):
    #q0 = qw
    t = np.matrix.trace(m)
    q = np.asarray([0.0, 0.0, 0.0, 0.0], dtype=np.float64)

    if(t > 0):
        t = np.sqrt(t + 1)
        q[3] = 0.5 * t
        t = 0.5/t
        q[0] = (m[2,1] - m[1,2]) * t
        q[1] = (m[0,2] - m[2,0]) * t
        q[2] = (m[1,0] - m[0,1]) * t

    else:
        i = 0
        if (m[1,1] > m[0,0]):
            i = 1
        if (m[2,2] > m[i,i]):
            i = 2
        j = (i+1)%3
        k = (j+1)%3

        t = np.sqrt(m[i,i] - m[j,j] - m[k,k] + 1)
        q[i] = 0.5 * t
        t = 0.5 / t
        q[3] = (m[k,j] - m[j,k]) * t
        q[j] = (m[j,i] + m[i,j]) * t
        q[k] = (m[k,i] + m[i,k]) * t

    return q

def q2R(q):
    q = normalize_vector(q)
    qX = q[0]
    qY = q[1]
    qZ = q[2]
    qW = q[3]

    sqX = qX * qX
    sqY = qY * qY
    sqZ = qZ * qZ
    sqW = qW * qW

    m00 = sqX - sqY - sqZ + sqW
    m11 = -sqX + sqY - sqZ + sqW
    m22 = -sqX - sqY + sqZ + sqW 

    qXqY = qX * qY
    qZqW = qZ * qW
    m10 = 2 * (qXqY + qZqW)
    m01 = 2 * (qXqY - qZqW)

    qXqZ = qX * qZ
    qYqW = qY * qW
    m20 = 2 * (qXqZ - qYqW)
    m02 = 2 * (qXqZ + qYqW)

    qYqZ = qY * qZ
    qXqW = qX * qW
    m21 = 2 * (qYqZ + qXqW)
    m12 = 2 * (qYqZ - qXqW)

    R = np.array([[m00, m01, m02],
                  [m10, m11, m12],
                  [m20, m21, m22]])
    
    return R

def R2EAs(R):
    sy = math.sqrt(R[0,0]*R[0,0] + R[1,0]*R[1,0])
    
    singular = abs(sy) < 1e-6
 
    if not singular:
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else:
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
 
    return np.array([x, y, z])

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

def L_BF(satellite, state):
    # Calculate magnitude of anguler momentum
    L = np.dot(satellite.I, state[10:13])
    return L

def T_BF(satellite, state):
    L = L_BF(satellite, state)
    T = 0.5 * np.dot(state[10:13].T, L)
    return T