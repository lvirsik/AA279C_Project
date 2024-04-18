import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.image import imread
from Simulator.simulationConstants import *
from Vehicle.satelliteConstants import *
from Simulator.util import *

# Plot the possible trajectory the angular velocity can trace with the constraint of being compatible with the energy content and magnitude of the anguler momentum
def plot_w_Energy_Momentum(init_c, I_principle):
    w = get_w_from_EulerAngle(init_c)

    I_x = I_principle[0,0] 
    I_y = I_principle[1,1]
    I_z = I_principle[2,2]

    # Calculate magnitude of anguler momentum
    L = np.dot(I_principle, w)
    L_sqr = np.dot(L.transpose(), L)
    L_mag = pow(L_sqr, 0.5)

    # Calculate the kinetic energy times 2
    T = 0.5 * np.dot(w.transpose(), L)

    # Momentum ellipsoid axis coefficients
    a_L = L_mag / I_x
    b_L = L_mag / I_y
    c_L = L_mag / I_z

    # Energy ellipsoid axis coefficients
    a_T = pow((T / I_x), 0.5)
    b_T = pow((T / I_y), 0.5)
    c_T = pow((T / I_z), 0.5)

    # Spherical coordinates 
    phi = np.linspace(0, 2 * np.pi, 256)
    theta = np.linspace(0, np.pi, 256)

    # Momentum ellipsoid surface coordinates in cartesian
    x_L = a_L * np.outer(np.cos(phi), np.sin(theta))
    y_L = b_L * np.outer(np.sin(phi), np.sin(theta))
    z_L = c_L * np.outer(np.ones(np.size(phi)), np.cos(theta)) 
    # energy ellipsoid surface coordinates in cartesian
    x_T = a_T * np.outer(np.cos(phi), np.sin(theta))
    y_T = b_T * np.outer(np.sin(phi), np.sin(theta))
    z_T = c_T * np.outer(np.ones(np.size(phi)), np.cos(theta)) 

    # Plot trajectory in 3D
    fig = plt.figure(1)
    ax = fig.add_subplot(projection='3d')
    ax.plot_surface(x_T, y_T, z_T, color = 'r')
    ax.plot_surface(x_L, y_L, z_L)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.set_title("Momentum ellipsoid")

    plt.show()

# Plot the possible trajectory the angular velocity that is within both the energy ellipsoid and the momentum ellipsoid, a bolhode.
def plot_polHode(init_c, trajectory, I_principle):
    w_x = trajectory[:,9] # if I use all 10000 points the polhode graph is a mess
    w_y = trajectory[:,10]
    w_z = trajectory[:,11]
    
    # Plot trajectory in 3D
    fig = plt.figure(3)
    ax = fig.add_subplot(projection='3d')
    ax.plot(w_x, w_y, w_z)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.set_title("polhode")


    # 2D plots
    #T, L_mag = plot_w_Energy_Momentum(init_c, I_principle)

    # I_x = I_principle[0,0]
    # I_y = I_principle[1,1]
    # I_z = I_principle[2,2]

    # eqn1 = pow(L_mag, 2) - 2 * T * I_x
    # eqn2 = pow(L_mag, 2) - 2 * T * I_y
    # eqn3 = pow(L_mag, 2) - 2 * T * I_z

    # a = (I_y - I_x) * I_y / eqn1    
    # b = (I_z - I_x) * I_z / eqn1

    # c = (I_x - I_y) * I_x / eqn2 
    # d = (I_z - I_y) * I_z / eqn2

    # e = (I_x - I_z) * I_x / eqn3
    # f = (I_y - I_z) * I_y / eqn3  
    
    # theta = np.linspace(0, np.pi, 256)
    # x_1 = a * np.sin(theta) 
    # y_1 = b * np.cos(theta) 

    # x_2 = c * np.sin(theta) 
    # y_2 = d * np.cos(theta) 

    # x_3 = e * np.sin(theta) 
    # y_3 = f * np.cos(theta) 

    # fig = plt.figure(4)
    # plt.plot(x_1.transpose(), y_1.transpose())
    # plt.title('polhode from yz plane')
    # plt.xlabel('y')
    # plt.ylabel('z')

    # fig = plt.figure(5)
    # plt.plot(x_2.transpose(), y_2.transpose())
    # plt.title('polhode from xz plane')
    # plt.xlabel('x')
    # plt.ylabel('z')

    # fig = plt.figure(6)
    # plt.plot(x_3.transpose(), y_3.transpose())
    # plt.title('polhode from xy plane')
    # plt.xlabel('x')
    # plt.ylabel('y')
    
    plt.show()