import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.image import imread
from Simulator.simulationConstants import *
from Vehicle.satelliteConstants import *

def get_w_from_EulerAngle(trajectory):
     # 1 2 3 = phi theta psi = yaw pitch roll
    euler_angles = trajectory[6:9]
    euler_angle_rates = trajectory[9:12]

    phi = euler_angles[0]
    theta = euler_angles[1]
    psi = euler_angles[2]
    
    phiDot = euler_angle_rates[0]
    thetaDot = euler_angle_rates[1]
    psiDot = euler_angle_rates[2]

    # Angular velocity from euler angles and euler angles rates
    w = np.array([[phiDot * np.sin(theta) * np.sin(phiDot) + thetaDot * np.cos(psi)],
                  [phiDot * np.sin(theta) * np.cos(phiDot) + thetaDot * np.sin(psi)],
                  [phiDot * np.cos(theta) + psiDot]])
    return w

# Plot the possible trajectory the angular velocity can trace with the constraint of being compatible with the energy content and magnitude of the anguler momentum
def plot_w_Energy_Momentum(init_c):
    w = init_c[6:9]

    I_x = I_COM_BF[0,0]
    I_y = I_COM_BF[1,1]
    I_z = I_COM_BF[2,2]

    # Get rid of 0.5 on right side and have 2 on left side 2T = 1/2 * I * w^2 
    T = np.dot(w, np.dot(I_COM_BF, w))

    L = np.dot(I_COM_BF, w)
    L_sqr = np.dot(L, np.transpose(L))
    L_mag = pow(L_sqr, 0.5)

    # Ellipsoid axis
    a_T = pow((T / I_x), 0.5)
    b_T = pow((T / I_y), 0.5)
    c_T = pow((T / I_z), 0.5)

    # Ellipsoid axis
    a_L = L_mag / I_x
    b_L = L_mag / I_y
    c_L = L_mag / I_z

    # Plotting ellipsoid using spherical coordinates conversion to cartesian
    phi_T = np.linspace(0, 2 * np.pi, 256)
    theta_T = np.linspace(0, np.pi, 256)
    x_T = a_T * np.outer(np.cos(phi_T), np.sin(theta_T))
    y_T = b_T * np.outer(np.sin(phi_T), np.sin(theta_T))
    z_T = c_T * np.outer(np.ones(np.size(phi_T)), np.cos(theta_T)) 
    print(np.shape(z_T))

    # Plotting ellipsoid using spherical coordinates conversion to cartesian
    phi_L = np.linspace(0, 2 * np.pi, 256)
    theta_L = np.linspace(0, np.pi, 256)
    x_L = a_L * np.outer(np.cos(phi_L), np.sin(theta_L))
    y_L = b_L * np.outer(np.sin(phi_L), np.sin(theta_L))
    z_L = c_L * np.outer(np.ones(np.size(phi_L)), np.cos(theta_L)) 
    print(np.shape(z_T))
    
    """
    phi_L = np.linspace(0, 2 * np.pi, 256).reshape(256, 1)
    theta_L = np.linspace(0, np.pi, 256).reshape(-1, 256)
    x_L = a_L * np.sin(theta_L) * np.cos(phi_L)
    y_L = b_L * np.sin(theta_L) * np.sin(phi_L)
    z_L = c_L * np.cos(theta_L) 
    print(np.shape(z_L))
    """

    # Plot trajectory in 3D
    fig = plt.figure(1)
    ax = fig.add_subplot(111, projection='3d')
    ax.plot_surface(x_T, y_T, z_T)

    # Plot trajectory in 3D
    fig = plt.figure(2)
    ax = fig.add_subplot(111, projection='3d')
    ax.plot_surface(x_L, y_L, z_L)

    # 256 x 256 arrays 
    return [T, L]

# Plot the possible trajectory the angular velocity that is within both the energy ellipsoid and the momentum ellipsoid, a bolhode.
def plot_polHode(init_c, trajectory):
    w = get_w_from_EulerAngle(trajectory)
    w_x = w[0]
    w_y = w[1]
    w_z = w[2]
    print(w_x)

    # Plot trajectory in 3D
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot_surface(w_x, w_y, w_z)

    """
    KE, L = plot_w_Energy_Momentum(init_c)
    
    I_x = I_COM_BF[0,0]
    I_y = I_COM_BF[1,1]
    I_z = I_COM_BF[2,2]

    a = (I_x - pow(L, 2) / (2 * T)) * I_x
    b = (I_y - pow(L, 2) / (2 * T)) * I_y
    c = (I_z - pow(L, 2) / (2 * T)) * I_z

    ellipsoid_phi = np.linspace(0, 2 * np.pi, 256).reshape(256, 1)
    ellipsoid_theta = np.linspace(0, np.pi, 256).reshape(-1, 256)
    x = a * np.sin(ellipsoid_theta) * np.cos(ellipsoid_phi)
    y = b * np.sin(ellipsoid_theta) * np.sin(ellipsoid_phi)
    z = c * np.cos(ellipsoid_theta) # np.ones_like(ellipsoid_theta_T)

    L_sqr = np.dot(L, np.transpose(L))
    L_mag = pow(L_sqr, 0.5)

    
    eqn1 = L_mag - 2 * KE * I_x
    eqn2 = L_mag - 2 * KE * I_y
    eqn3 = L_mag - 2 * KE * I_z

    a = (I_y - I_x) * I_y / eqn1    
    b = (I_z - I_x) * I_z / eqn1

    c = (I_x - I_y) * I_x / eqn2 
    d = (I_z - I_y) * I_z / eqn2

    e = (I_x - I_z) * I_x / eqn3
    f = (I_y - I_z) * I_y / eqn3  
    
    ellipsoid_theta = np.linspace(0, np.pi, 256).reshape(-1, 256)
    x_1 = a * np.sin(ellipsoid_theta) 
    y_1 = b * np.cos(ellipsoid_theta) 

    x_2 = c * np.sin(ellipsoid_theta) 
    y_2 = d * np.cos(ellipsoid_theta) 

    x_3 = e * np.sin(ellipsoid_theta) 
    y_3 = f * np.cos(ellipsoid_theta) 

    """

    """
    output_energy = np.stack((energy[0], energy[1], energy[2]), axis = -1)
    output_momentum = np.stack((momentum[0], momentum[1], momentum[2]), axis = -1)

    size = np.shape(energy)
    intersection = np.zeros(3)

    # This O(n^2) to compare, can be much faster
    for n in range(size[0]):
        for m in range(size[1]):
            coord_e = output_energy[n][m]
            coord_m = output_momentum[n][m]
            if np.all(coord_e == coord_m) == True:
                print("e:" + str(coord_e))
                print("m:" + str(coord_m))
                intersection = np.append(intersection, coord_m, axis = 0)

    print(intersection)

    # Plot trajectory in 3D
    # fig = plt.figure(3)
    # ax = fig.add_subplot(111, projection='3d')
    # ax.plot_surface(intersection[0], intersection[1], intersection[2])
    """