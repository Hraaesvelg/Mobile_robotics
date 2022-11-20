from tdmclient import ClientAsync, aw
import numpy as np


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
