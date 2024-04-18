import numpy as np
from Simulator.simulation import Simulation
from Simulator.simulationConstants import *
from Simulator.dynamics import *
from Graphics.visualizeOrbits import *
from Graphics.visualizeRotations import *
from Graphics.visualizeEllipsoids import *

# Setup Simulation (Orbit)
initial_cartesian_state = OE_2_ECI(INITIAL_OEs)

# Setup Simulation (Rotations)
initial_EAs = INITIAL_EAs[0:3]
initial_Omega = get_w_from_EulerAngle(INITIAL_EAs).T[0]

# Combine and set up sim
initial_state = np.concatenate((initial_cartesian_state, initial_EAs, initial_Omega))
sim = Simulation(FINAL_TIME, TIMESTEP, initial_state)

# Run Simulation (Rotations)
trajectory = sim.propogate()

# Plots
plot_euler(trajectory)
plot_w_Energy_Momentum(initial_state, sim.satellite.I_principle)
plot_polHode(initial_state, trajectory, sim.satellite.I_principle)