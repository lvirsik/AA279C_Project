import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.image import imread
from Simulator.simulationConstants import *
from Vehicle.satelliteConstants import *

def get_EulerAngle_from_w(w, state):
    phi = state[6]
    theta = state[7]
    psi = state[8]
    
    phidot = (w[0]*np.sin(psi) + w[1]*np.cos(psi))/np.sin(theta)
    thetadot = w[0]*np.cos(psi) - w[1]*np.sin(psi)
    psidot = w[2] - (w[0]*np.sin(psi) + w[1]*np.cos(psi))*(1/np.tan(theta))
    
    return [phidot, thetadot, psidot]

def get_w_from_EulerAngle(trajectory):
     # 1 2 3 = phi theta psi = yaw pitch roll
    if (np.size(trajectory) == (12)): 
        euler_angles = trajectory[6:9]
        euler_angle_rates = trajectory[9:12]

        phi = euler_angles[0]
        theta = euler_angles[1]
        psi = euler_angles[2]
        
        phiDot = euler_angle_rates[0]
        thetaDot = euler_angle_rates[1]
        psiDot = euler_angle_rates[2]

    else:
        euler_angles = trajectory[:, 6:9]
        euler_angle_rates = trajectory[:, 9:12]

        phi = euler_angles[:, 0]
        theta = euler_angles[:, 1]
        psi = euler_angles[:,2]
        
        phiDot = euler_angle_rates[:, 0]
        thetaDot = euler_angle_rates[:,1]
        psiDot = euler_angle_rates[:, 2]

    # Angular velocity from euler angles and euler angles rates
    w = np.array([[phiDot * np.sin(theta) * np.sin(psi) + thetaDot * np.cos(psi)],
                  [phiDot * np.sin(theta) * np.cos(psi) - thetaDot * np.sin(psi)],
                  [phiDot * np.cos(theta) + psiDot]])
    
    return w

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

    print(L_mag**2 / (2*T))

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

    # 256 x 256 arrays 
    return [T, L_mag]

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
    T, L_mag = plot_w_Energy_Momentum(init_c, I_principle)

    I_x = I_principle[0,0]
    I_y = I_principle[1,1]
    I_z = I_principle[2,2]

    eqn1 = pow(L_mag, 2) - 2 * T * I_x
    eqn2 = pow(L_mag, 2) - 2 * T * I_y
    eqn3 = pow(L_mag, 2) - 2 * T * I_z

    a = (I_y - I_x) * I_y / eqn1    
    b = (I_z - I_x) * I_z / eqn1

    c = (I_x - I_y) * I_x / eqn2 
    d = (I_z - I_y) * I_z / eqn2

    e = (I_x - I_z) * I_x / eqn3
    f = (I_y - I_z) * I_y / eqn3  
    
    theta = np.linspace(0, np.pi, 256)
    x_1 = a * np.sin(theta) 
    y_1 = b * np.cos(theta) 

    x_2 = c * np.sin(theta) 
    y_2 = d * np.cos(theta) 

    x_3 = e * np.sin(theta) 
    y_3 = f * np.cos(theta) 

    fig = plt.figure(4)
    plt.plot(x_1.transpose(), y_1.transpose())
    plt.title('polhode from yz plane')
    plt.xlabel('y')
    plt.ylabel('z')

    fig = plt.figure(5)
    plt.plot(x_2.transpose(), y_2.transpose())
    plt.title('polhode from xz plane')
    plt.xlabel('x')
    plt.ylabel('z')

    fig = plt.figure(6)
    plt.plot(x_3.transpose(), y_3.transpose())
    plt.title('polhode from xy plane')
    plt.xlabel('x')
    plt.ylabel('y')

    """
    # This code was a brute force effort, comparing the coordinates of each ellipsoid to see if any were equivalent
    ellipsoids = plot_w_Energy_Momentum(init_c)
    energy = ellipsoids[0:3]
    momentum = ellipsoids[3:6]

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