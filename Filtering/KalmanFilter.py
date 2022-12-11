import math
import numpy as np

# Sampling time in seconds
T_s = 0.1

## Kalman filter matrices
A = np.array([[1.0, 0, T_s, 0],[0, 1.0, 0, T_s],[0, 0, 1.0, 0],[0, 0, 0, 1.0]])
B = np.array([[T_s, 0], [0, T_s], [1.0, 0], [0, 1.0]])
Q = np.diag([2, 2, 2, 2])
H = np.eye(4)

#     x, y, vx, vy         Measured state.
#     dvx, dvy             Velocity input.

#Adapted function from the course exercises session 8
def kalman_filter(x, y, vx, vy, x_est_prev, p_est_prev, dvx=0, dvy=0, detection=True):
    
    Input = np.array([dvx, dvy])
    x_est_a_priori = np.dot(A, x_est_prev) + np.dot(B, Input)
    
    p_est_a_priori = np.dot(A, np.dot(p_est_prev, A.T))
    p_est_a_priori = p_est_a_priori + Q if type(Q) != type(None) else p_est_a_priori
    
    if detection:
        R = np.diag([0.48, 0.48, 0.64, 0.49])
    else:
        R = np.diag([math.inf, math.inf, math.inf, math.inf])
        
    y = np.array([x, y, vx, vy])
    i = y - np.dot(H, x_est_a_priori)
    S = np.dot(H, np.dot(p_est_a_priori, H.T)) + R
    K = np.dot(p_est_a_priori, np.dot(H.T, np.linalg.inv(S)))
    
    x_est = x_est_a_priori + np.dot(K, i)
    p_est = p_est_a_priori - np.dot(K, np.dot(H, p_est_a_priori))
    
    return x_est, p_est
