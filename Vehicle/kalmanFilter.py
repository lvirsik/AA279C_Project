import numpy as np
import math
from Simulator.simulationConstants import *
from Simulator.util import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def kalman_filter(x, u, y, A, B, dt, P):
    """ Kalman filter"""
    # Calculate Process Noise Matrix
    Q = np.eye(13) *0.01
    # H assumes sensor format [q q w w]
    H = np.array([
        [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],   
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],   
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],    
    ]) 
  
    R = np.eye(14)
    
    # Time Step
    x_next, P_next = predict_step(x, u, A, B, P, Q, dt)
    
    # Measurement Step
    z = np.dot(H, x_next) # Expected Measurement based on state
    x_fit, P_fit = update_step(x_next, y, z, H, R, P_next)

    return x_fit, P_fit
        
def predict_step(x, u, A, B, P_prev, Q, dt):
    """ Prediction step for kalman filter"""

    A_new = np.eye(13) + A*dt
    B_new = B*dt
    x_next = A_new @ x + B_new @ u  #Predicted State Estimate
    P_next = (A_new @ P_prev @ A_new.T) + Q # Predicted Estimate Covariance
    return x_next, P_next

def update_step(x_next, y, z, H, R, P_next):
    """ Update step for kalman filter"""
    
    # Clean if there are no measurements from a sensor
    for i in range(len(y)):
        if math.isnan(y[i]):
            y[i] = z[i]
    S = (H @ P_next @ H.T) + R
    K = P_next @ H.T @ np.linalg.inv(S)
    x_fit = x_next + np.dot(K, (y-z))
    
    chunk = np.eye(13) - (K @ H)
    P_fit = (chunk @ P_next @ chunk.T) + (K @ R @ K.T)
    
    return x_fit, P_fit
    
def get_Y(satellite, state):
    # Sensor Readings
    st1 = satellite.starTracker1.reading(state)
    st2 = satellite.starTracker2.reading(state)
    st3 = satellite.starTracker3.reading(state)
    st4 = satellite.starTracker4.reading(state)
    su1 = satellite.sunSensor1.reading(state)
    gry1 = satellite.imu1.reading(state)
    gry2 = satellite.imu2.reading(state)
    
    # Rotation matrix
    R = q2R(state[6:10])
    
    # Get actual position of objects in sky
    sun_direction = normalize_vector(state[0:3] + JUPITER2SUN)
    star1_direction = normalize_vector(state[0:3] + JUPITER2STAR)
    star2_direction = normalize_vector(state[0:3] + JUPITER2STAR2)
    star3_direction = normalize_vector(state[0:3] + JUPITER2STAR3)
    star4_direction = normalize_vector(state[0:3] + JUPITER2STAR4)
    
    # Set up for conversion
    V1 = np.array([star1_direction, star2_direction, sun_direction])
    V2 = np.array([star4_direction, sun_direction, star2_direction])
    B1 = np.array([st1, st2, su1])
    B2 = np.array([st4, su1, st2])
    
    # Convert Measurements to Quaternions
    V1_centroid = np.mean(V1, axis=0)
    V2_centroid = np.mean(V2, axis=0)
    B1_centroid = np.mean(B1, axis=0)
    B2_centroid = np.mean(B2, axis=0)
    V1_centered = V1 - V1_centroid
    B1_centered = B1 - B1_centroid
    V2_centered = V2 - V2_centroid
    B2_centered = B2 - B2_centroid

    H1 = V1_centered.T @ B1_centered
    H2 = V2_centered.T @ B2_centered

    U1, S1, Vt1 = np.linalg.svd(H1)
    U2, S2, Vt2 = np.linalg.svd(H2)

    R1 = Vt1.T @ U1.T
    R2 = Vt2.T @ U2.T

    # Handle the case where the determinant of R is -1 (reflection case)
    if np.linalg.det(R1) < 0:
        Vt1[2, :] *= -1
        R1 = Vt1.T @ U1.T
    if np.linalg.det(R2) < 0:
        Vt2[2, :] *= -1
        R2 = Vt2.T @ U2.T
  
    q1 = match_quaternion_signs(state[6:10], normalize_vector(R2q(R1)))
    q2 = match_quaternion_signs(state[6:10], normalize_vector(R2q(R2)))
    
    # Stack and Send Back
    Y = np.concatenate((q1, q2, gry1, gry2), axis=None)
    return Y