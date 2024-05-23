import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.image import imread
from Simulator.simulationConstants import *
from Simulator.util import *
from Simulator.dynamics import *

# Check visualizePset6 too see th e new attitude error.
# Look for attitude_error_history

def plot_small_angle_quaternion_conversion(estimated_trajectory, actual_trajectory):

    attitude_history = actual_trajectory[:, 6:10]
    estimated_attitude_history = estimated_trajectory

    # Create a figure for the rotational dynamics plots
    fig, axs = plt.subplots(2, 4, figsize=(15, 10))

    # Define the names for the rotational dynamics plots
    attitude_plot_names = [
        "Q0 Error",
        "Q1 Error",
        "Q2 Error",
        "Q3 Error",
        "Q0 Error Small Angle",
        "Q1 Error Small Angle",
        "Q2 Error Small Angle",
        "Q3 Error Small Angle"
    ]

    attitude_error_history = attitude_history - estimated_attitude_history[1:len(estimated_attitude_history)]
    quats = np.zeros((len(attitude_error_history), 4))
    for i in range(len(attitude_error_history)):
        Rot_matrix = Q2R_smallAngle(attitude_error_history[i, :]) 
        quats[i, :] = R2q(Rot_matrix)

    # Plot the rotational dynamics
    for i in range(4):
        axs[0, i].plot(attitude_error_history[:, i])
        axs[0, i].set_title(attitude_plot_names[i])
    for i in range(4):
        axs[1, i].plot(quats[:, i])
        axs[1, i].set_title(attitude_plot_names[i+4])

    # Adjust the layout
    plt.tight_layout()
    plt.show()

