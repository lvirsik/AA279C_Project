import numpy as np
import math
from Simulator.simulationConstants import *
from Simulator.util import *

def kalman_filter(x, u, y, A, B, dt, P):
    """ Kalman filter"""
    # Calculate Process Noise Matrix
    Q = np.eye(13)
    # H assumes sensor format [q q q q w w]
    H = np.array([
        [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
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
  
    R = np.eye(22)
    
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
    su1 = satellite.sunSensor1.reading(state)
    su2 = satellite.sunSensor2.reading(state)
    gry1 = satellite.imu1.reading(state)
    gry2 = satellite.imu2.reading(state)
    
    # Rotation matrix
    R = q2R(state[6:10])
    
    # Get actual position of objects in sky
    sat2sun = state[0:3] + JUPITER2SUN
    sun_direction = np.dot(np.linalg.inv(R), normalize_vector(sat2sun)) # We need it in body frame
    sat2star1 = state[0:3] + JUPITER2STAR
    star1_direction = np.dot(np.linalg.inv(R), normalize_vector(sat2star1)) # We need it in body frame
    sat2star2 = state[0:3] + JUPITER2STAR2
    star2_direction = np.dot(np.linalg.inv(R), normalize_vector(sat2star2)) # We need it in body frame
    
    # Convert Measurements to Quaternions
    st1_q = match_quaternion_signs(state[6:10], R2q(np.array([st1, star1_direction, np.cross(st1, star1_direction)])))
    st2_q = match_quaternion_signs(state[6:10], R2q(np.array([st2, star2_direction, np.cross(st2, star2_direction)])))
    su1_q = match_quaternion_signs(state[6:10], R2q(np.array([su1, sun_direction, np.cross(su1, sun_direction)])))
    su2_q = match_quaternion_signs(state[6:10], R2q(np.array([su2, sun_direction, np.cross(su2, sun_direction)])))
    
    # Stack and Send Back
    Y = np.concatenate((st1_q, st2_q, su1_q, su2_q, gry1, gry2), axis=None)
    return Y