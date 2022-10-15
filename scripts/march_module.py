from globalVariables import GlobalVariables
import numpy as np
def CoM_right_displacement():

    glob = GlobalVariables()
    L1 = glob.L1
    end_CoM = 0.02
    size = int((L1-end_CoM)/0.005)
    CoM_traj = np.zeros((3,size+1))
    CoM_traj_y = np.arange(L1,end_CoM,-0.005)
    CoM_traj[1,:] = CoM_traj_y[:]
    
    end_foot = 0.015
    



if __name__ == '__main__':
    CoM_right_displacement()
