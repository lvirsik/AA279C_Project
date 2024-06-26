import numpy as np
from Simulator.simulation import Simulation
from Simulator.simulationConstants import *
from Simulator.dynamics import *
from Simulator.trajectoryPlanning import *
from Simulator.stateestimation import *
from Graphics.visualizeOrbits import *
from Graphics.visualizeRotations import *
from Graphics.visualizeEllipsoids import *
from Graphics.visualizePset3 import *
from Graphics.visualizePset4_4 import *
from Graphics.visualizePset6 import *

def runSIM():
    
    # Create Ideal Trajectory
    ideal_trajectory = zeroed_ideal_traj(FINAL_TIME, TIMESTEP)
    
    # Setup Simulation (Orbit)
    initial_cartesian_state = OE_2_ECI(INITIAL_OEs)

    # Setup Simulation (Rotations)
    initial_Q = normalize_vector(INITIAL_Q[0:4])
    initial_w = INITIAL_w[0:3]

    # Combine and set up sim
    initial_state = np.concatenate((initial_cartesian_state, initial_Q, initial_w))
    sim = Simulation(FINAL_TIME, TIMESTEP, initial_state)
    
    # Run Simulation (Rotations)
    trajectory = sim.propogate()
    # Plots
    plot_euler(trajectory, sim)
    plot_error_over_time(trajectory, ideal_trajectory, sim)
    plot_frames_over_time(sim)
    # plot_torques_over_time(sim)

    # plot_orbit(trajectory)
    # plot_torque(trajectory, sim.satellite)
    # plot_attitude_estimation(estimated_rot_trajectory, trajectory)
    # plot_attitude_estimation(estimated_vel_history, trajectory)
    #plot_attitude_estimation(estimated_stat_SE, trajectory)
    #plot_kalman(sim.state_history, sim.kalman_state_history, sim.sensed_state_history)
    plot_three_values_over_time(sim.u_history, sim.satellite)

runSIM()