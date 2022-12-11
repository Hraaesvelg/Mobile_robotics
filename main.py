#lancer la communication asynchrone avec thymio
from tdmclient import ClientAsync, aw
client = ClientAsync()
node = await client.wait_for_node()
await node.lock()

#lancer la communication asynchrone avec thymio
from tdmclient import ClientAsync, aw
client = ClientAsync()
node = await client.wait_for_node()
await node.lock()

cap = cv2.VideoCapture(1)
img = vs.get_image(cap)
margin = 60
tol = 0.1
thymio = rbt.RobotNav()

i = 0 #step vis graph
path = 0
while cap.isOpened():
    ret, frame = cap.read()
    
    if thymio.get_state() == 0:
        thymio.initialisation_step(frame, margin, False)
        path = thymio.get_path('real')
        
        pos_thy, two_centres, test_detect = vs.detect_start1(frame, False)  #recupere pos thymio, #test_detect bool a false si thymio pas detect
        thymio.update_position_cam(two_centres, node, client)
        beta = ctrl.get_angle2goal(thymio.get_geometry(),[path[1].x, path[1].y])
        ctrl.get_correct_orientation(beta, node, 50, 0.2)
        if beta < tol:
            thymio.set_state(1)
            ctrl.stop_motors(node)
        print("fin state")
    
    if thymio.get_state() == 1:  #motion control commence
        pos_thy, two_centres, test_detect = vs.detect_start1(frame, False)  #recupere pos thymio, #test_detect bool a false si thymio pas detetc
        frame = glb.draw_path(frame, thymio.get_path('real')) #dessine chemin du thymio new fenetre
        path_img = thymio.get_path('img')
        if test_detect:
            frame = glb.draw_thymio(frame, path_img,"")   # dessine point thymio new fenetre
        #frame = glb.draw_thymio(frame, path_kalman)   # dessine point thymio new fenetre
        if test_detect:
            thymio.update_position_cam(two_centres, node, client)  #update pos thymio et angle dans la classe robot
        #else:
            #thymio.update_position_kalman(node, client, False)
    
        if len(path) > (i+1):
            next_goal = (path[i+1].x,path[i+1].y) 
            pos, teta = thymio.get_geometry()
            temp=ctrl.astolfi(pos, teta, next_goal, node, client) #astolfi controller
            if temp==1:
                i=i+1
        else : 
            ctrl.stop_motors(node)
            ctrl.leds_blink(node)
            break
    cv2.imshow('frame',frame)     
    if cv2.waitKey(1) == ord('q'):
        ctrl.stop_motors(node)
        break
        
cap.release()
cv2.destroyAllWindows()