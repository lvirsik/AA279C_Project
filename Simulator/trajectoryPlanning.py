import numpy as np


def zeroed_ideal_traj(tf, ts):
    time = np.linspace(0, int(tf/ts)*ts, num=int(tf/ts)+1)
    traj = np.zeros((len(time), 13))
    return traj 
    