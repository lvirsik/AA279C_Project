import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.image import imread
from Simulator.simulationConstants import *

def plot_orbit(trajectory):
    # Plot trajectory in 3D
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # Load Earth texture image
    earth_texture = imread("Graphics/earth.jpg") / 255.0
    earth_radius = R_EARTH
    # Plot Earth as a textured sphere
    img = plt.imread("Graphics/earth.jpg")
    u = np.linspace(0, 2 * np.pi, img.shape[0])  # Keep the same number of points as v
    v = np.linspace(0, np.pi, img.shape[1])
    x = earth_radius * np.outer(np.cos(u), np.sin(v))
    y = earth_radius * np.outer(np.sin(u), np.sin(v))
    z = earth_radius * np.outer(np.ones(np.size(u)), np.cos(v))
    ax.plot_surface(x, y, z, rstride=4, cstride=4, facecolors=earth_texture, alpha=0.8)
    
    ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2])
    ax.set_xlabel('X (km)')
    ax.set_ylabel('Y (km)')
    ax.set_zlabel('Z (km)')
    ax.set_title('Orbital Trajectory')
    
    # Set equal axis lengths
    max_range = np.max(trajectory)
    ax.set_xlim([-max_range, max_range])
    ax.set_ylim([-max_range, max_range])
    ax.set_zlim([-max_range, max_range])
    ax.set_box_aspect([1,1,1])
    plt.show()