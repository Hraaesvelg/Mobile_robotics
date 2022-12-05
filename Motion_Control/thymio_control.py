from tdmclient import ClientAsync, aw
import numpy as np

'''thymio dimensions'''
r = 22  # mm
l = 48  # mm

'''controller constants to tune accordingly'''
kp = 15  # >0
ka = 20  # > kp
kb = -0.0001  # <0

'''speed limits and sensors thresholds to tune accordingly'''
<<<<<<< HEAD
v_max = 100
v_min = 50
thres_arrived = 50
=======
v_max = 1000
v_min = 100
thres_arrived = 10
>>>>>>> origin/main
alpha_thres = 2


def thym_motors(right, left):
    """
    Create the speed structure to control the Thymio's motors (low level)
    :param right: speed of the right motor
    :param left: speed of the left motor
    :return: structure needed by the Thymio to access peripherals
    """
    return {"motor.left.target": [left], "motor.right.target": [right]}


def set_motor_speed(sr, sl, node):
    """
    Set the speed of our thymio
    :param sr: speed of the right motor
    :param sl: speed of the left motor
    :param node: link to our robot
    """
    aw(node.set_variables(thym_motors(sr, sl)))


def stop_motors(node):
    """
    Stop motor motion
    :param node: link to our robot
    :return:
    """
    set_motor_speed(0, 0, node)


def get_motors_speed(node, client):
    """
    Function to get access to current motors' speed
    :param node:
    :param node, client: link to our robot
    :return: structure containing motors' speed ([speed_right, speed_left])
    """
    aw(node.wait_for_variables({"motor.left.speed", "motor.right.speed"}))
    aw(client.sleep(0.05))
    return [node.v.motor.right.speed, node.v.motor.left.speed]


def get_prox_sensors(node, client):
    """
    Function to get access to current motors' speed
    :param node:
    :param node, client: link to our robot
    :return: structure containing proximity sensors' values ([prox1,..., prox8])
    """
    aw(node.wait_for_variables({"prox.horizontal"}))
    aw(client.sleep(0.05))
    return node.v.prox.horizontal

def mov_simplified(pos, theta, target, node):
    state=0
    delta_pos = (int(target.x) - pos[0], int(target.y) - pos[1])
    alpha = -theta + np.arctan2(delta_pos[1], delta_pos[0])
    rho = np.linalg.norm(delta_pos)
    if alpha>alpha_thres:
        left_speed = int(-l*alpha)
        right_speed = int(l*alpha)
        set_motor_speed(right_speed, left_speed, node)
        return 0
<<<<<<< HEAD
    elif rho>thres_arrived :
=======
    elif rho>thresh_arrived :
>>>>>>> origin/main
        set_motor_speed(v_min, v_min, node)
        return 0
    else:
        stop_motors(node)
        return 1

def astolfi(pos, theta, target, node):
    state = 0  # this functions is called recursivly untill state=1 i.e. the thymio has arrived
    delta_pos = (int(target.x) - pos[0], int(target.y) - pos[1])
    rho = np.linalg.norm(delta_pos)
    alpha = -theta + np.arctan2(delta_pos[1], delta_pos[0])
    beta = -theta - alpha
    omega = ka * alpha + kb * beta
    if rho > thres_arrived:
        v = kp * rho
        if v > v_max: v = v_max
        if v < v_min: v = v_min
    else:
        v = 0
        state = 1
    left_speed = int(v - l * omega)
    right_speed = int(v + l * omega)
    set_motor_speed(right_speed, left_speed, node)
    return state


def leds_blink(node):
    return 0
