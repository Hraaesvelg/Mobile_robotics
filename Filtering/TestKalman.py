import time
import math
import numpy as np

from vision import detect_start

'''A calculer sur le robot qu'on va utiliser''' 
SPEED_COEFF = 0.3 
MAX_MOTOR_SPEED = 500 #Thymio unit
MAX_RAW_MOTOR_SPEED = 2**16

class RobotThymio():
    
    def __init__(self):
        self.ser = connect_to_thymio(verbose = verbose) '''Check comment elle s'appelle dans notre projet'''
        self.last_orientation = 0
        self.last_thymioCoordinates = np.zeros(2)
        
    def GetThymioValues(self, img):

        x = 0
        y = 0
        vx = 0
        vy = 0
        
        thymioCoordinates = detect_start(img, show=False)[0]
        speed = (self.get_motor_right_speed + self.get_motor_left_speed)/2 * SPEED_COEFF
        
        '''Il nous faut une fonction qui vérifie que thymio a été détecté sur l'image / la vidéo'''
        detect_thymio = check_if_thymio()
        
        if detect_thymio:
            Detection = True
            x = thymioCoordinates[0]
            y = thymioCoordinates[1]
            
            orientation = compute_orientation(self, img)
            
            vx = speed * math.cos(orientation)
            vy = speed * math.sin(orientation)
            
        else: 
            Detection = False
        
        return x, y, vx, vy, Detection
     
        
    #Return orientation compared to x-axis between pi and -pi    
    def compute_orientation(self, img):
        center1, center2 = detect_start(img, show=False)[4], detect_start(img, show=False)[5]
        x = center1[0] - center2[0]
        y = center1[1] - center2[1]
        
        orientation = math.atan2(y, x)
        set_last_orientation(orientation)
        
        return orientation
        
    def get_motor_right_speed(self):
        try:
            speed = self.ser.get_var("motor.right.speed")
            if speed <= MAX_MOTOR_SPEED:
                return speed
            else:
                return speed - MAX_RAW_MOTOR_SPEED
        except (IndexError, KeyError):
            return 0
        
    def get_motor_left_speed(self):
        try:
            speed = self.ser.get_var("motor.left.speed")
            if speed <= MAX_MOTOR_SPEED:
                return speed
            else:
                return speed - MAX_RAW_MOTOR_SPEED
        except (IndexError, KeyError):
            return 0
        
    def set_last_orientation(self, orientation):
        self.last_orientation = orientation
        
    def set_motor_right_speed(self, speed):
        try:
            if speed >= 0:
                self.ser.set_var("motor.right.target", speed)
            else:
                self.ser.set_var("motor.right.target", MAX_RAW_MOTOR_SPEED + speed)
        except (IndexError, KeyError): # Raised if forgot to wait after connecting to Thymio.
            pass
    
    def set_motor_left_speed(self, speed):
        try:
            if speed >= 0:
                self.ser.set_var("motor.left.target", speed)
            else:
                self.ser.set_var("motor.left.target", MAX_RAW_MOTOR_SPEED + speed)
        except (IndexError, KeyError): # Raised if forgot to wait after connecting to Thymio.
            pass