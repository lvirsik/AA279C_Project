
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider
import random

def plotStaticAndDynamicVector(static_vector, dynamic_vector_history):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plotting the static vector
    ax.quiver(0, 0, 0, static_vector[0], static_vector[1], static_vector[2], color='r', label='Static Vector')

    # Extracting x, y, z components of the dynamic vector history
    x = [v[0] for v in dynamic_vector_history]
    y = [v[1] for v in dynamic_vector_history]
    z = [v[2] for v in dynamic_vector_history]

    # Plotting the path of the changing vector over time as lines
    ax.plot(x, y, z, color='b', alpha=0.5, label='Dynamic Vector Path')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Static Vector and Dynamic Vector Path')
    ax.legend()
    ax.set_xlim([-1, 1])  
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    ax.set_aspect('equal')

    plt.show()

def plotVectorsOverTime(orbitalVectorX, orbitalVectorY, orbitalVectorZ,
                        bodyAxesX, bodyAxesY, bodyAxesZ,
                        principalAxesX, principalAxesY, principalAxesZ,
                        time):

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot initial vectors
    quiver_orbital_frameX = ax.quiver(0, 0, 0, orbitalVectorX[0, 0], orbitalVectorX[1, 0], orbitalVectorX[2, 0], color='r', label='Orbital Vector X')
    quiver_orbital_frameY = ax.quiver(0, 0, 0, orbitalVectorY[0, 0], orbitalVectorY[1, 0], orbitalVectorY[2, 0], color='g', label='Orbital Vector Y')
    quiver_orbital_frameZ = ax.quiver(0, 0, 0, orbitalVectorZ[0, 0], orbitalVectorZ[1, 0], orbitalVectorZ[2, 0], color='b', label='Orbital Vector Z')

    quiver_body_axesX = ax.quiver(0, 0, 0, bodyAxesX[0, 0], bodyAxesX[1, 0], bodyAxesX[2, 0], color='c', label='Body Axes X')
    quiver_body_axesY = ax.quiver(0, 0, 0, bodyAxesY[0, 0], bodyAxesY[1, 0], bodyAxesY[2, 0], color='m', label='Body Axes Y')
    quiver_body_axesZ = ax.quiver(0, 0, 0, bodyAxesZ[0, 0], bodyAxesZ[1, 0], bodyAxesZ[2, 0], color='y', label='Body Axes Z')

    quiver_principal_axesX = ax.quiver(0, 0, 0, principalAxesX[0, 0], principalAxesX[1, 0], principalAxesX[2, 0], color='k', label='Principal Axes X')
    quiver_principal_axesY = ax.quiver(0, 0, 0, principalAxesY[0, 0], principalAxesY[1, 0], principalAxesY[2, 0], color='gray', label='Principal Axes Y')
    quiver_principal_axesZ = ax.quiver(0, 0, 0, principalAxesZ[0, 0], principalAxesZ[1, 0], principalAxesZ[2, 0], color='orange', label='Principal Axes Z')

    ax.set_xlim([-1, 1])  
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Vectors in 3D Space at Time: {:.2f}'.format(time[0]))
    ax.legend()

    ax_time = plt.axes([0.1, 0.01, 0.8, 0.03], facecolor='lightgoldenrodyellow')
    slider = Slider(ax_time, 'Time', 0, len(time) - 1, valinit=0, valstep=1)

    def update(val):
        index = int(slider.val)

        quiver_orbital_frameX.set_segments([np.array([[0, 0, 0], orbitalVectorX[index]])])
        quiver_orbital_frameY.set_segments([np.array([[0, 0, 0], orbitalVectorY[index]])])
        quiver_orbital_frameZ.set_segments([np.array([[0, 0, 0], orbitalVectorZ[index]])])

        quiver_body_axesX.set_segments([np.array([[0, 0, 0], bodyAxesX[index]])])
        quiver_body_axesY.set_segments([np.array([[0, 0, 0], bodyAxesY[index]])])
        quiver_body_axesZ.set_segments([np.array([[0, 0, 0], bodyAxesZ[index]])])

        quiver_principal_axesX.set_segments([np.array([[0, 0, 0], principalAxesX[index]])])
        quiver_principal_axesY.set_segments([np.array([[0, 0, 0], principalAxesY[index]])])
        quiver_principal_axesZ.set_segments([np.array([[0, 0, 0], principalAxesZ[index]])])

        ax.set_title('Vectors in 3D Space at Time: {:.2f}'.format(time[index]))
        fig.canvas.draw_idle()

    slider.on_changed(update)

    plt.show()


def generateTestVectorsOverTime(time):

    vectorX_start = np.array([1, 0, 0])  
    vectorY_start = np.array([0, 1, 0])
    vectorZ_start = np.array([0, 0, 1])

    vectorX_start = vectorX_start[:, np.newaxis]
    vectorY_start = vectorY_start[:, np.newaxis]
    vectorZ_start = vectorZ_start[:, np.newaxis]

    def rotate_vector(vector, angle, axis):
        angle = angle + random.uniform(0, 1)*10
        axis = axis / np.linalg.norm(axis)
        cos_theta = np.cos(angle)
        sin_theta = np.sin(angle)
        rot_matrix = cos_theta * np.eye(3) + (1 - cos_theta) * np.outer(axis, axis) + sin_theta * np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
        rotated_vector = np.dot(rot_matrix, vector)
        return rotated_vector

    vectorX = np.array([rotate_vector(vectorX_start, t, np.array([0, 0, 1])) for t in time]).squeeze().T
    vectorY = np.array([rotate_vector(vectorY_start, t, np.array([0, 0, 1])) for t in time]).squeeze().T
    vectorZ = np.array([rotate_vector(vectorZ_start, t, np.array([0, 0, 1])) for t in time]).squeeze().T

    return vectorX, vectorY, vectorZ

def extract_columns(matrix):
    column1 = matrix[:,0,:]
    column2 = matrix[:,1,:]
    column3 = matrix[:,2,:]

    column1 = np.expand_dims(column1, axis=0)
    column2 = np.expand_dims(column2, axis=0)
    column3 = np.expand_dims(column3, axis=0)

    return column1[0, 1:len(column1[0])-1], column2[0, 1:len(column2[0])-1], column3[0, 1:len(column3[0])-1]

def plot_torques_over_time(sim):
    time = np.linspace(0, sim.tf, int(sim.tf/sim.ts)+1)
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim([-10**-12, 10**-12])  
    ax.set_ylim([-10**-12, 10**-12])
    ax.set_zlim([-10**-12, 10**-12])
    
    GG = ax.quiver(0, 0, 0, sim.satellite.gg_torque_history[0, 0], sim.satellite.gg_torque_history[1, 0], sim.satellite.gg_torque_history[2, 0], color='r', label='Gravity Gradient Torque')
    MAG = ax.quiver(0, 0, 0, sim.satellite.mag_torque_history[0, 0], sim.satellite.mag_torque_history[1, 0], sim.satellite.mag_torque_history[2, 0], color='g', label='Magnetic Torque')
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Torques in 3D Body axes at Time: {:.2f}'.format(time[0]))
    ax.legend()

    ax_time = plt.axes([0.1, 0.01, 0.8, 0.03], facecolor='lightgoldenrodyellow')
    slider = Slider(ax_time, 'Time', 0, len(time) - 1, valinit=0, valstep=1)

    def update(val):
        index = int(slider.val)

        GG.set_segments([np.array([[0, 0, 0], sim.satellite.gg_torque_history[index]])])
        MAG.set_segments([np.array([[0, 0, 0], sim.satellite.mag_torque_history[index]])])

        ax.set_title('Vectors in 3D Space at Time: {:.2f}'.format(time[index]))
        fig.canvas.draw_idle()

    slider.on_changed(update)

    plt.show()
    

def plot_frames_over_time(sim):
    time = np.linspace(0, sim.tf, int(sim.tf/sim.ts)+1)
    orbitalVectorX, orbitalVectorY, orbitalVectorZ = extract_columns(sim.RTN_history)
    bodyAxesX, bodyAxesY, bodyAxesZ = extract_columns(sim.R_history)
    principalAxesX, principalAxesY, principalAxesZ = extract_columns(sim.R_prin_history)

    plotVectorsOverTime(orbitalVectorX, orbitalVectorY, orbitalVectorZ,
                        bodyAxesX, bodyAxesY, bodyAxesZ,
                        principalAxesX, principalAxesY, principalAxesZ, time)