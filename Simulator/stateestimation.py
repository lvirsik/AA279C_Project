import numpy as np
from Vehicle.satelliteConstants import *
from Simulator.simulationConstants import *
from Simulator.dynamics import *
import scipy

def determ_ad(sat, state):
    # Get observation / measurements
    observations = np.zeros((3,4)) # M 
    observations[:,0] = sat.starTracker1.get_sensor_observation(state).T
    observations[:,1] = sat.starTracker2.get_sensor_observation(state).T
    observations[:,2] = sat.sunSensor1.get_sensor_observation(state).T
    observations[:,3] = sat.sunSensor2.get_sensor_observation(state).T

    # Get reference directions
    current_state = state # The reference or oracle state (The Dynamics / Predicted Motion)
    current_pos = current_state[0:3] # 3D Vector in ECI? (JCI?)
    
    SAT2SUN = current_pos + JUPITER2SUN
    sun_direction = normalize_vector(SAT2SUN)
    SAT2STAR = current_pos + JUPITER2STAR
    star_direction = normalize_vector(SAT2STAR)

    SAT2STAR2 = current_pos + JUPITER2STAR2
    star_direction2 = normalize_vector(SAT2STAR2)

    reference_direction = np.stack((star_direction, star_direction2, sun_direction, sun_direction), axis = 1)

    # Since we have more then 3 measurements, we have a non sqaure matrix and cant take inverse and thus need to make them square
    M = np.matmul(observations, reference_direction.T)
    V = np.matmul(reference_direction, reference_direction.T)

    attitude = np.matmul(M, np.linalg.inv(V)) # Rotation matrix question mark? 
    attitude = R2q(attitude)
    return attitude

def determ_ad_ficttious(sat, traj):
    p1 = sat.starTracker1.get_sensor_observation(traj)
    p2 = sat.starTracker2.get_sensor_observation(traj)

    m1 = p1
    m2 = np.cross(p1,p2) / np.linalg.norm(np.cross(p1, p2))
    m3 = np.cross(m1,m2)

    M = np.stack((m1, m2, m3), axis = 1)
    V = traj[:, 6:10].T # V

    attitude = np.matmul(M, np.linalg.inv(V))
    return attitude

def stat_ad(sat, state):
    observations = np.zeros((3,4))
    observations[:,0] = sat.starTracker1.get_sensor_observation(state).T
    observations[:,1] = sat.starTracker2.get_sensor_observation(state).T
    observations[:,2] = sat.sunSensor1.get_sensor_observation(state).T
    observations[:,3] = sat.sunSensor2.get_sensor_observation(state).T

    # Get reference directions
    current_state = state # The reference or oracle state (The Dynamics / Predicted Motion)
    current_pos = current_state[0:3] # 3D Vector in ECI? (JCI?)
    
    SAT2SUN = current_pos + JUPITER2SUN
    sun_direction = normalize_vector(SAT2SUN)
    SAT2STAR = current_pos + JUPITER2STAR
    star_direction = normalize_vector(SAT2STAR)
    SAT2STAR2 = current_pos + JUPITER2STAR2
    star_direction2 = normalize_vector(SAT2STAR2)
    reference_direction = np.stack((star_direction, star_direction2, sun_direction, sun_direction), axis = 1)
    
    # CORRECT UNTIL HERE
    weights = [1, 1, 1, 1]
    W = np.sqrt(weights) * observations
    U = np.sqrt(weights) * reference_direction
    B = W @ U.T
    S = B + B.T
    Z = np.array([[B[1,2] - B[2,1], B[2, 0] - B[0, 2], B[0, 1] - B[1, 0]]]).T
    sigma = np.trace(B)
    K = np.vstack([np.hstack([S - np.identity(3)*sigma, Z]), np.hstack([Z.T, np.array([[sigma]])])])

    value, vector = np.linalg.eigh(K)
    attitude = vector[:,-1]
    
    ## CORRECT AFTER HERE
    attitude = match_quaternion_signs(state[6:10], attitude)
    return attitude

# Replicate Gyreoscope and the integration of its data
def ang_vel_ad(sat, state):
    observations = np.zeros((2,3))
    observations[0,:] = sat.imu1.get_sensor_observation(state)
    observations[1,:] = sat.imu2.get_sensor_observation(state)
    current_w = np.mean(observations, axis = 0)

    current_q = state[6:10]
    q_rate = qdot(current_q, current_w)
    attitude = current_q + TIMESTEP * q_rate
    return attitude