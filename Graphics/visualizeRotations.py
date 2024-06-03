import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.image import imread
from Simulator.simulationConstants import *
from Simulator.util import *
from Simulator.dynamics import *

def plot_euler(trajectory, sim, frame='pf'):
    rotational_history = trajectory[:, 6:]

    # Create a figure for the rotational dynamics plots
    fig, axs = plt.subplots(2, 3, figsize=(15, 10))

    # Define the names for the rotational dynamics plots
    rotational_dynamics_plot_names = [
        "Pitch (degrees)",
        "Yaw (degrees)",
        "Roll (degrees)",
        "wx",
        "wy",
        "wz",
    ]

    # Define the conversion factor from radians to degrees
    RAD2DEG = 180 / np.pi

    analytical_solution = np.empty((1, 3))
    for i in range(len(rotational_history)):
        if frame=='pf':
            R = np.dot(sim.satellite.R, q2R(rotational_history[i,0:4]))
            EAs = R2EAs(R)
            rotational_history[i, 3:6] = np.dot(sim.satellite.R, rotational_history[i, 4:7])
            rotational_history[i, 0:3] = EAs
            
            analytical_solution = np.vstack((analytical_solution, axially_symmetric_analytical_solution(i*TIMESTEP, I_COM_BF, INITIAL_w)))
        if frame=='bf':
            R = q2R(rotational_history[i,0:4])
            EAs = R2EAs(R)
            rotational_history[i, 3:6] = rotational_history[i, 4:7]
            rotational_history[i, 0:3] = EAs
            
            analytical_solution = np.vstack((analytical_solution, axially_symmetric_analytical_solution(i*TIMESTEP, I_COM_BF, INITIAL_w)))
        
        

    # Plot the rotational dynamics
    for i in range(6):
        row = i // 3
        col = i % 3
        if i < 3:
            axs[row, col].plot(rotational_history[:, i] * RAD2DEG)
        else:
            axs[row, col].plot(rotational_history[:, i])
            #axs[row, col].plot(analytical_solution[:, i-3])
        axs[row, col].set_title(rotational_dynamics_plot_names[i])
    
    # Adjust the layout
    plt.tight_layout()
    plt.show()
    
def plot_kalman(state_history, kalman_state_history, sensed_state_history):

    # Number of time steps
    time_steps = state_history.shape[0]

    # Time array for x-axis
    time = np.arange(time_steps)

    # Create the first figure for the first 6 variables
    fig1, axs1 = plt.subplots(3, 2, figsize=(12, 8))
    fig1.suptitle('Position and Velocity')

    names = ["X Position ",
             "Y Position ",
             "Z Position ",
             "X Velocity ",
             "Y Velocity ",
             "Z Velocity "]
    
    for i in range(6):
        ax = axs1[i // 2, i % 2]
        ax.plot(time, state_history[:, i], label='{}'.format(names[i]))
        ax.plot(time, kalman_state_history[:, i], label='Kalman {}'.format(names[i]))
        ax.set_xlabel('Time')
        ax.set_ylabel('Value')
        ax.legend()
        ax.grid(True)

    # Create the second figure for the last 7 variables
    fig2, axs2 = plt.subplots(4, 2, figsize=(12, 10))
    fig2.suptitle('Quaternions and Angular Velocity Vector Error')

    names = ["Q0 ",
             "Q1 ",
             "Q2 ",
             "Q3 ",
             "Wx ",
             "Wy ",
             "Wz "]

    for i in range(7):
        ax = axs2[i // 2, i % 2]
        # if i <= 3:
        #     ax.plot(time, sensed_state_history[:, i], label='Star 1, 2 {}'.format(names[i]))
        #     ax.plot(time, sensed_state_history[:, i + 4], label='Star 3, 4 {}'.format(names[i]))
        # if i > 3:
        #     ax.plot(time, sensed_state_history[:, i + 4], label='Gyro 1 {}'.format(names[i]))
        #     ax.plot(time, sensed_state_history[:, i + 7], label='Gyro 2 {}'.format(names[i]))
        ax.plot(time, state_history[:, i + 6], label=' {}'.format(names[i]))
        ax.plot(time, kalman_state_history[:, i + 6], label='Kalman {}'.format(names[i]))
        ax.set_xlabel('Time')
        ax.set_ylabel('Value')
        ax.legend()
        ax.grid(True)

    # Hide the empty subplot in the second figure
    fig2.delaxes(axs2[3, 1])

    # Show the plots
    plt.show()
