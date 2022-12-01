import numpy as np
import math

# Sampling time in seconds
T_s = 0.1

## Kalman filter matrices
A = np.array([[1.0, 0, T_s, 0],[0, 1.0, 0, T_s],[0, 0, 1.0, 0],[0, 0, 0, 1.0]])
B = np.array([[T_s, 0], [0, T_s], [1.0, 0], [0, 1.0]])
Q = np.diag([2, 2, 3, 3])
H = np.eye(4)

''' Ces matrices sont à vérifier et redéfinir en fonction de nos besoins ''' 

#  @param   x_meas          Measured x-coordinate.
#  @param   y_meas          Measured y-coordinate.
#  @param   vx_meas         Measured x-velocity.
#  @param   vy_meas         Measured y-velocity.
#  @param   dvx             y-velocity control input.
#  @param   dvy             x-velocity control input.


def kalman_filter(x_meas, y_meas, vx_meas, vy_meas, x_est_prev, P_est_prev, dvx = 0, dvy = 0, detection = True):
    
    Input = np.array([dvx, dvy])
    x_est_a_priori = np.dot(A, x_est_prev) + np.dot(B, Input)
    
    P_est_a_priori = np.dot(A, np.dot(P_est_prev, A.T));
    P_est_a_priori = P_est_a_priori + Q if type(Q) != type(None) else P_est_a_priori
    
    if detection:
        R = np.diag([0.25, 0.25, 0.30, 0.30])
    else:
        R = np.diag([math.inf, math.inf, math.inf, math.inf])
        
    y = np.array([x_meas, y_meas, vx_meas, vy_meas])

    i = y - np.dot(H, x_est_a_priori)

    S = np.dot(H, np.dot(P_est_a_priori, H.T)) + R
    
    x_est = x_est_a_priori + np.dot(K, i)
    P_est = P_est_a_priori - np.dot(K, np.dot(H, P_est_a_priori))
    
    return x_est, P_est
                                    
    
