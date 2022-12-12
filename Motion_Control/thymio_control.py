from tdmclient import ClientAsync, aw
import numpy as np
import math

'''controller constants to tune accordingly'''
kp = 1  # >0
ka = 100  # > kp
kb = -0.01  # <0

'''speed limits and sensors thresholds to tune accordingly'''
v_max = 200
v_min = 70
thres_arrived = 40
obstSpeedGain = np.array([6, 4, -2, -0, -0]) / 100


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
    :param node, client: link to our robot
    :return: structure containing proximity sensors' values ([prox1,..., prox8])
    """
    aw(node.wait_for_variables({"prox.horizontal"}))
    aw(client.sleep(0.05))
    return node.v.prox.horizontal

def get_angle2goal(geometry, goal):
    """

    Compute the angle between thecurent orientation of the tymio and its next goal
    :return: beta the angle btw goal/crt orientation
    """
    beta = math.atan2((goal[1] - geometry[0][1]), goal[0] - geometry[0][0])- geometry[1] - np.pi
    if beta < -np.pi:
        beta = beta + 2*np.pi
    elif beta > np.pi:
        beta = beta- 2*np.pi
    return -beta

def get_correct_orientation(beta, node, speed, tol):
    if abs(beta) < tol:
        stop_motors(node)
    if beta < 0 :
        set_motor_speed(-speed, speed, node)
    elif beta > 0:
        set_motor_speed(speed, -speed, node)
    

def astolfi(pos, theta, target, node, client):
    """
    Astolfi controller used to control the robot. We have added obstacles' avoidance procedure
    :param pos: Actual Position of the thymio: [x,y]
    :param theta: Actual angle
    :param target: Target's position: [x,y]
    :param node, client: link to our robot
    :return: True if the target is reached, false otherwise
    """
    state = 0
    delta_pos = [target[0] - pos[0], -(target[1] - pos[1])]
    rho = np.linalg.norm(delta_pos)
    alpha = theta + np.arctan2(delta_pos[1], delta_pos[0]) - np.pi
    if alpha > np.pi:
        alpha -= 2 * np.pi
    elif alpha < -np.pi:
        alpha += 2 * np.pi
    beta = theta - alpha - np.pi
    if beta > np.pi:
        beta -= 2 * np.pi
    elif beta < -np.pi:
        beta += 2 * np.pi
    sensors = np.array(get_prox_sensors(node, client)[0:5])
    vit_obst_left = np.sum(np.multiply(sensors, obstSpeedGain))
    vit_obst_right = np.sum(np.multiply(sensors, np.flip(obstSpeedGain)))
    omega = ka * alpha + kb * beta
    if rho > thres_arrived:
        v = kp * rho
        if v > v_max: v = v_max
        if v < v_min: v = v_min
    else:
        v = 0
        state = 1
    left_speed = v - omega + vit_obst_left
    right_speed = v + + omega + vit_obst_right
    set_motor_speed(int(right_speed), int(left_speed), node)
    return state


def leds_blink(node):
    """
    Led show to celebrate the thymio's arrival
    :param node: Needed to control the thymio
    :return:
    """
    program = """
    var on = 0  # 0=off, 1=on
    timer.period[0] = 500

    onevent timer0
    on = 1 - on  # "on = not on" with a syntax Aseba accepts
    leds.top = [32 * on, 32 * on, 0]
    """
    aw(node.compile(program))
    aw(node.run())
