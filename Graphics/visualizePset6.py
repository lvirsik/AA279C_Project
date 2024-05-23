import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.image import imread
from Simulator.simulationConstants import *
from Simulator.util import *
from Simulator.dynamics import *


def plot_error_over_time(trajectory, ideal_trajectory, sim):
    rotational_history = trajectory[:, 6:]

    # Create a figure for the rotational dynamics plots
    fig, axs = plt.subplots(2, 4, figsize=(15, 10))

    # Define the names for the rotational dynamics plots
    rotational_dynamics_plot_names = [
        "Q1 Error",
        "Q2 Error",
        "Q3 Error",
        "Q4 Error",
        "wx Error",
        "wy Error",
        "wz Error",
    ]

    # Define the conversion factor from radians to degrees
    RAD2DEG = 180 / np.pi

    error = rotational_history - ideal_trajectory[:, 6:13]
        
        

    # Plot the rotational dynamics
    for i in range(4):
        axs[0, i].plot(error[:, i])
        axs[0, i].set_title(rotational_dynamics_plot_names[i])
    for i in range(3):
        axs[1, i].plot(rotational_history[:, 4+i])
        axs[1, i].set_title(rotational_dynamics_plot_names[4+i])
    
    # Adjust the layout
    plt.tight_layout()
    plt.show()

def plot_attitude_estimation(estimated_trajectory, actual_trajectory):
    attitude_history = actual_trajectory[:, 6:10]
    estimated_attitude_history = estimated_trajectory

    # Create a figure for the rotational dynamics plots
    fig, axs = plt.subplots(3, 4, figsize=(15, 10))

    # Define the names for the rotational dynamics plots
    attitude_plot_names = [
        "Q0",
        "Q1",
        "Q2",
        "Q3",
        "Estimated Q0",
        "Estimated Q1",
        "Estimated Q2",
        "Estimated Q3",
        "Q0 Error",
        "Q1 Error",
        "Q2 Error",
        "Q3 Error"
    ]

    # FOR PSET 7 PLOTTING ERROR BETWEEN ESTIMATED AND ACTUAL
    attitude_error_history = attitude_history - estimated_attitude_history[1:len(estimated_attitude_history)]

    # Plot the rotational dynamics
    for i in range(4):
        axs[0, i].plot(attitude_history[:, i])
        axs[0, i].set_title(attitude_plot_names[i])
    for i in range(4):
        axs[1, i].plot(estimated_attitude_history[:, i])
        axs[1, i].set_title(attitude_plot_names[i+4])
    for i in range(4):
        axs[2, i].plot(attitude_error_history[:, i])
        axs[2, i].set_title(attitude_plot_names[i+8])
    
    # Adjust the layout
    plt.tight_layout()
    plt.show()
