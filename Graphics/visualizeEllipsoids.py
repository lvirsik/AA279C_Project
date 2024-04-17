import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.image import imread
from Simulator.simulationConstants import *
from Vehicle.satelliteConstants import *

def get_w_from_EulerAngle(trajectory):
     # 1 2 3 = phi theta psi = yaw pitch roll
    euler_angles = trajectory [6:9]
    euler_angle_rates = trajectory[9:12]

    phi = euler_angles[0]
    theta = euler_angles[1]
    psi = euler_angles[0]
    
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
    KE = np.dot(w, np.dot(I_COM_BF, w))

    L = np.dot(I_COM_BF, w)
    L_sqr = np.dot(L, np.transpose(L))
    L_mag = pow(L_sqr, 0.5)

    # Ellipsoid axis
    a_T = pow((KE / I_x), 0.5)
    b_T = pow((KE / I_y), 0.5)
    c_T = pow((KE / I_z), 0.5)

    # Ellipsoid axis
    a_L = L_mag / I_x
    b_L = L_mag / I_y
    c_L = L_mag / I_z

    # Plotting ellipsoid using spherical coordinates
    ellipsoid_phi_T = np.linspace(0, 2 * np.pi, 5)
    ellipsoid_theta_T = np.linspace(0, np.pi, 5)
    x_T = a_T * np.outer(np.sin(ellipsoid_theta_T), np.cos(ellipsoid_phi_T))
    y_T = b_T * np.outer(np.sin(ellipsoid_theta_T), np.sin(ellipsoid_phi_T))
    z_T = c_T * np.outer(np.ones_like(ellipsoid_theta_T), np.cos(ellipsoid_theta_T))

    # Plotting ellipsoid using spherical coordinates
    ellipsoid_phi_L = np.linspace(0, 2 * np.pi, 5)
    ellipsoid_theta_L = np.linspace(0, np.pi, 5)
    x_L = a_L * np.outer(np.sin(ellipsoid_theta_L), np.cos(ellipsoid_phi_L))
    y_L = b_L * np.outer(np.sin(ellipsoid_theta_L), np.sin(ellipsoid_phi_L))
    z_L = c_L * np.outer(np.ones_like(ellipsoid_theta_L), np.cos(ellipsoid_theta_L))

    # Plot trajectory in 3D
    fig = plt.figure(1)
    ax = fig.add_subplot(111, projection='3d')
    ax.plot_surface(x_T, y_T, z_T)

    # Plot trajectory in 3D
    fig = plt.figure(2)
    ax = fig.add_subplot(111, projection='3d')
    ax.plot_surface(x_L, y_L, z_L)

    # 256 x 256 arrays 
    return [x_T, y_T, z_T, x_L, y_L, z_L]

# Plot the possible trajectory the angular velocity that is within both the energy ellipsoid and the momentum ellipsoid, a bolhode.
def plot_polHode(init_c):
    ellipsoids = plot_w_Energy_Momentum(init_c)
    energy = ellipsoids[0:3]
    momentum = ellipsoids[3:6]

    size = np.shape(energy)
    output_energy = np.stack((energy[0], energy[1], energy[2]), axis = -1)
    output_momentum = np.stack((momentum[0], momentum[1], momentum[2]), axis = -1)
    print(output_energy)

    intersection = np.zeros((1,3))
    # This O(n^2) to compare, can be much faster
    for n in range(size[0]):
        for m in range(size[1]):
            coord_e = output_energy[n][m]
            print(coord_e)
            #if coord_e.all() == coord_m.all():
                # print("Ener: " + str(coord_e))
             #   intersection = np.append(intersection, coord_m, axis = 0)

    # print(intersection)

    # Plot trajectory in 3D
    fig = plt.figure(3)
    ax = fig.add_subplot(111, projection='3d')
    ax.plot_surface(intersection[0], intersection[1], intersection[2])
