from tdmclient import ClientAsync, aw
import numpy as np
import math

'''controller constants to tune accordingly'''
kp = 0.5  # >0
ka = 25  # > kp
kb = -0.01  # <0

'''speed limits and sensors thresholds to tune accordingly'''
v_max = 200
v_min = 20
thres_arrived = 50
angle_thres = 0.17
obstSpeedGain = np.array([6,4,-2,-0,-0])/100


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
    Function to survey the thymio's sensors
    :param node:
    :param node, client: link to our robot
    :return: structure containing proximity sensors' values ([prox1,..., prox8])
    """
    aw(node.wait_for_variables({"prox.horizontal"}))
    aw(client.sleep(0.05))
    return node.v.prox.horizontal


def astolfi(pos, theta, target, node, client):
    state = 0  # this functions is called recursivly untill state=1 i.e. the thymio has arrived
    delta_pos = [target[0] - pos[0], -(target[1] - pos[1])]
    rho = np.linalg.norm(delta_pos)
    alpha = -theta - np.arctan2(delta_pos[1], delta_pos[0])
    if alpha>np.pi:
        alpha-=2*np.pi
    elif alpha<-np.pi:
        alpha+=2*np.pi
    beta = theta - alpha
    if beta>np.pi:
        beta-=2*np.pi
    elif beta<-np.pi:
        beta+=2*np.pi
    sensors= np.array(get_prox_sensors(node, client)[0:5])
    vit_obst_left=np.sum(np.multiply(sensors,obstSpeedGain))
    vit_obst_right=np.sum(np.multiply(sensors,np.flip(obstSpeedGain)))
    #print("teta=",theta,"  alpa=",alpha,"  beta=",beta,"  rho=",rho) #prints to debug
    
    if (alpha>angle_thres):
        omega = ka * alpha + kb * beta
    else: 
        omega=kb*beta
    
    omega = ka * alpha + kb * beta
    if rho > thres_arrived:
        v = kp * rho
        if v > v_max: v = v_max
        if v < v_min: v = v_min
    else:
        v = 0
        state = 1
    left_speed = v - omega + vit_obst_left
    right_speed = v + omega + vit_obst_right
    set_motor_speed(int(right_speed), int(left_speed), node)
    return state


def leds_blink(node):
    return 0
