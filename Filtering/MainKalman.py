from KalmanFilter import kalman_filter
import RobotKalman as RK 
import vision as vs
from tdmclient import ClientAsync, aw


def main():
      
    #lancer la communication asynchrone avec thymio
    client = ClientAsync()
    node = await client.wait_for_node()
    await node.lock()
    
    Thymio = RK.RobotNav(node, client)
    
    thymio_start = vs.detect_start(img, show=False, begin=True)
    img_taken = False
    
    # Initialize a posteriori covariance matrix of predicted state
    # P_est is a list of covariance matrices
    P_est_front = [1000 * np.eye(4)]
    P_est_back = [1000 * np.eye(4)]

    # Initialize a posteriori pose estimate
    # The states are [x, y, vx, vy]
    # x_est is a list of a posteriori estimates 
    x_est_front = [np.array([thymio_start[4][0], thymio_start[4][1], 0, 0])]
    x_est_back = [np.array([thymio_start[5][0], thymio_start[5][1], 0, 0])]
    
    #Inputs
    dvx = 0
    dvy = 0
    
    
    while True:
        Detection = True 
    
        #Si une image a été prise
        if img_taken:
            x_front, y_front, vx, vy, Detection = Thymio.GetThymioValues(img, front=True)
            x_back, y_back = Thymio.GetThymioValues(img, front=False)
        
        else:
            Detection = False
            
        new_x_est_front, new_P_est_front = kalman_filter(x_front, y_front, vx, vy, x_est_front[-1]
                                                         , P_est_front[-1], dvx, dvy, detection= Detection)
        
        new_x_est_back, new_P_est_back = kalman_filter(x_back, y_back, vx, vy, x_est_back[-1]
                                                         , P_est_back[-1], dvx, dvy, detection= Detection)
        
        x_est_front.append(new_x_est_front)
        x_est_back.append(new_x_est_back)
        
        P_est_front.append(new_P_est_front)
        P_est_back.append(new_P_est_back)
        
        Thymio.set_last_position(new_x_est_front[0:2], new_x_est_back[0:2]) ##A faire fonction qui set la position du thymio avec ces 2 
                                                                            ##estimations de centres 
        dvx = 0
        dvy = 0
