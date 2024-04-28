import numpy as np
from Simulator.simulation import Simulation
from Simulator.simulationConstants import *
from Simulator.dynamics import *
from Graphics.visualizeOrbits import *
from Graphics.visualizeRotations import *
from Graphics.visualizeEllipsoids import *
from Graphics.visualizePset3 import *

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
time = np.linspace(0, FINAL_TIME, int(FINAL_TIME/TIMESTEP)+1)
plot_euler(trajectory)
plotStaticAndDynamicVector(sim.L_inertial*0.0005, sim.w_inertial_history)

orbitalVectorX, orbitalVectorY, orbitalVectorZ = extract_columns(sim.RTN_history)
bodyAxesX, bodyAxesY, bodyAxesZ = extract_columns(sim.R_history)
principalAxesX, principalAxesY, principalAxesZ = extract_columns(sim.R_prin_history)

plotVectorsOverTime(orbitalVectorX, orbitalVectorY, orbitalVectorZ,
                    bodyAxesX, bodyAxesY, bodyAxesZ,
                    principalAxesX, principalAxesY, principalAxesZ, time)
