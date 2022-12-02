

def rotate(direction, speed, time):
    if direction == 'left':
        print('Rotation gauche')
    elif direction == 'right':
        print('Rotation droite')


def advance(speed, time):
    print('Avance le robot')


def detect_obstacle():
    return False

def stop_thymio():
    print( 'robot stoped' )