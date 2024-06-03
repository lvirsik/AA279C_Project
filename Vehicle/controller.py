import numpy as np
import control
from Vehicle.satelliteConstants import *

def compute_K(len_state, A, B):    

    A = A[-7:, -7:]
    B = B[-7:, :]


    # Control
    # C = np.append(np.identity(4), np.zeros((4, 3)), axis=1)
    K,S,E = control.lqr(A, B, Q * 10, R)
    return K

def controller(K, error):
    U = np.dot(-K, error[6:13]) # U is the desired accelerations
    return U