import numpy as np
from Vehicle.satelliteConstants import *
from Simulator.simulationConstants import *
from Simulator.dynamics import *

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
    current_rot = current_state[6:10] # Quaternions
    
    SAT2SUN = current_pos + JUPITER2SUN
    sun_direction = normalize_vector(SAT2SUN)
    SAT2STAR = current_pos + JUPITER2STAR
    star_direction = normalize_vector(SAT2STAR)

    #SAT2STAR2 = current_pos + JUPITER2STAR2
    # star_direction2 = normalize_vector(SAT2STAR2)

    reference_direction = np.stack((star_direction, star_direction, sun_direction, sun_direction), axis = 0).T

    # Since we have more then 3 measurements, we have a non sqaure matrix and cant take inverse and thus need to make them square
    M = np.matmul(observations, reference_direction.T)
    V = np.matmul(reference_direction, reference_direction.T)
    print(V)

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

def stat_ad(sat, traj):
    observations = np.zeros(4,4)
    observations[0,:] = sat.starTracker1.get_sensor_observation(traj)
    observations[1,:] = sat.starTracker2.get_sensor_observation(traj)
    observations[2,:] = sat.sunSensor1.get_sensor_observation(traj)
    observations[3,:] = sat.sunSensor2.get_sensor_observation(traj)

    weights = np.zeros(4,3)

    attitude = 0
    return attitude

# Replicate Gyreoscope and the integration of its data
def ang_vel_ad(sat, traj):
    observations = np.zeros(4,3)
    observations[0,:] = sat.imu1.get_sensor_observation(traj)
    observations[1,:] = sat.imu2.get_sensor_observation(traj)

    attitude = 0
    return attitude