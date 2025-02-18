from ast import Try
import time, sys, pygame
from threading import Thread, Lock
from pygame import joystick
import math
import os
import inputs
os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"
from fairino import Robot


###############################################################
#                   CONTROLLER MAPPING                        #   
# X & Y axis movement: Left stick                             #
# Z movement: right stick (up and down)                       #
# Rotate robot wrist (up/down): Right Stick (left to right)   #
# Rotate robot wrist: left/right trigger                      #
# A: Opens/Closes gripper (if one exists)                     #
# X: Move to Home Pose                                        #
# Y: Clears any robot errors                                  #
# B: Stops robot movement                                     #
###############################################################

# CONSTANTS
ROBOT_IP = '192.168.57.2' # CHANGE TO MATCH YOUR DEVICE
SOFT_LIMIT = [-175.0, 175.0, -265.0, 85.0, -160.0
              , 160.0, -265.0, 85.0, -175.0, 175.0, -175.0, 175.0]

HOME_POS = [-80, -65, -80, -130, 90, 0]

PACKING_POS = [-125, 10, -158, -122, 0.012, 0] #Specific for FR16
L_LEFT = 'll'
L_RIGHT = 'lr'
L_UP = 'lu'
L_DOWN = 'ld'
R_LEFT = 'rl'
R_RIGHT = 'rr'
R_UP = "ru"
R_DOWN = "rd"
LT = 'lt'
RT = 'rt'

global is_moving

def truncate(number):
    factor = 10.0 ** 2
    return math.trunc(number * factor) / factor

def checkNeutralPos(joystick):
    axes = joystick.get_numaxes()
    for m in range(axes - 2):
        axis = truncate(float(joystick.get_axis(m) * 1))
        if(abs(axis) > 0.1):
            return False
    for m in range(4, 6):
        axis = truncate(float(joystick.get_axis(m) * 1))
        if(abs(axis) < 0.9):
            return False
    return True

def toggleGripper(robot, gripperOpen):
    # Opens/closes gripper
    if(gripperOpen):
        try:
            print(robot.MoveGripper(1, 0, 80, 80, 5000,0))
        except:
            print("Error opening gripper")
    else:
        try:
            print(robot.MoveGripper(1, 100, 80, 80, 5000,0))
        except:
            print("Error closing gripper")
    return not gripperOpen
        
def moveToSafety(robot, index, joint_pos, lock):
    # After joint out of bounds, reset to safe pose
    if joint_pos[index] < 0:
        joint_pos[index] += 7
    else:
        joint_pos[index] -= 7
    if(lock.locked):
        lock.release()
    with lock:
        time.sleep(0.5)
        robot.MoveJ(joint_pos, 0, 0)
        time.sleep(0.5)


def checkJointLimits(robot, lock):
    try:
        err, j_pos = robot.GetActualJointPosDegree()
        print(f"Actual Position: msg: {err}, value: {j_pos}")
    except:
        print("error when getting actual joint positions")
        return
    for i in range(len(j_pos) - 1):
        if (j_pos[i] - 5 <= SOFT_LIMIT[i*2]) or j_pos[i] + 5 >= SOFT_LIMIT[i*2 + 1]:
            robot.ImmStopJOG()
            print(f"Joint {i}: Too close to soft limit")
            time.sleep(.5)
            moveToSafety(robot, i, j_pos, lock)
            return True
    return False

def moveAxisThread(robot, lock, joystick, mv_input):
    if lock.locked():
        print("Move axis call rejected: mutex populated")
        return
    # Move according to user input
    with lock:
        index = -1
        print("Move axis called")
        if(mv_input == L_LEFT):
            err = robot.StartJOG(2, 1, 1, 200)
            print(f"Move axis returned: {err}")
            if(err == 0):
                index = 0

        elif(mv_input == L_RIGHT):
            try:
                err = robot.StartJOG(2, 1, 0, 200)
                print(f"Move axis returned: {err}")
                if(err == 0):
                    index = 0
            except:
                print("Error sending move commands")
        elif(mv_input == L_DOWN):
            err = robot.StartJOG(2, 2, 1, 200)
            print(f"Move axis returned: {err}")
            if(err == 0):
                index = 1

        elif(mv_input == L_UP):
            err = robot.StartJOG(2, 2, 0, 200)
            print(f"Move axis returned: {err}")
            if err == 0:
                index = 1

        elif(mv_input == R_UP):
            try:
                err = robot.StartJOG(2, 3, 0, 200)
                print(f"Move axis returned: {err}")
                if err == 0:
                    index = 3
            except:
                print("Error in vertical jog")

        elif(mv_input == R_DOWN):
            err = robot.StartJOG(2, 3, 1, 200)
            print(f"Move axis returned: {err}")
            if err == 0:
                index = 3

        elif(mv_input == R_RIGHT):
            try:
                err = robot.StartJOG(0, 5, 0, 180)
                if err == 0:
                    index = 2
            except:
                print("Error rotating end tool")
        elif(mv_input == R_LEFT):
            try:
                err = robot.StartJOG(0, 5, 1, 180)
                if err == 0:
                    index = 2
            except:
                print("Error rotating end tool")

        elif(mv_input == LT):
            try:
                err = robot.StartJOG(0, 4, 1, 180)
                if err == 0:
                    index = 4
            except:
                print("Error in rotating joint 4")

        elif(mv_input == RT):
            try:
                err = robot.StartJOG(0, 4, 0, 180)
                if err == 0:
                    index = 5
            except:
                print("Error in rotating joint 4")
        if(index == -1):
            return
        
        stop_thread = Thread(target=stopAxis, args=[robot, joystick, index, lock])
        # stop_thread.daemon = True
        stop_thread.start()
        time.sleep(.25)
        # robot.ImmStopJOG()

    
def stopAxis(robot, joystick, JSAxel_index, lock):
    if(JSAxel_index == -1):
        print("No axis recieved by stopAxis()")
        return
    print("Stop axis called")
    # While the user holds the joystick, let it jog. When stick is released, stop the movement
    while True:
        axel = float(joystick.get_axis(JSAxel_index))
        # print(axel)
        if(JSAxel_index < 4):
            if abs(axel) < 0.1:
                err = robot.ImmStopJOG()
                print(f"Stop jog err: {err}")
                while(err != 0):
                    time.sleep(0.1)
                    print("ERROR in stopping robot!!!")
                    err = robot.ImmStopJOG()
                return
            
        else:
             if abs(axel) >= 1:
                err = robot.ImmStopJOG()
                while(err != 0):
                    time.sleep(0.1)
                    print("ERROR in stopping robot!!!")
                    err = robot.ImmStopJOG()
                return

        if checkJointLimits(robot, lock):
            err = robot.ImmStopJOG()
            while(err != 0):
                time.sleep(0.1)
                print("ERROR in stopping robot!!!")
                err = robot.ImmStopJOG()
            return
        time.sleep(0.1)

def ResetErrors(robot):
    print("Error reset activated")
    err = robot.ResetAllError()
    print(f"ResetAllError() response: {err}")
    if(err !=  0):
        print("Robot broke safety conditions. Please check WebApp for error msg")

def run(robot, robot_speed, gripperConnected):
    # Initialization, PyGame window creation, etc
    gripperOpen = True
    is_moving = False
    pygame.init()
    screen = pygame.display.set_mode((1280, 720))
    screen.fill("purple")
    pygame.display.flip()
    lock = Lock()
    clock = pygame.time.Clock()
    joy_stat = 0
    running = True
    axis = [0, 0, 0, 0, 0, 0]
    straxis = "0,0,0,0,"
    while running:
        pygame.joystick.init()
        joystick_count = pygame.joystick.get_count()
        # Connects controller
        while pygame.joystick.get_count() != 0 and joy_stat == 0:
            for i in range(joystick_count):
                joystick = pygame.joystick.Joystick(i)
                joystick.init()
                print("Controller connected")
                time.sleep(1)
                joy_stat = 1
            break
        # Inits controller
        while pygame.joystick.get_count() == 0:
            pygame.joystick.quit()
            joy_stat = 0
            print("Waiting for controller to connect...")
            time.sleep(1)
            pygame.joystick.init()
            continue	
        
        # Refresh reads 
        pygame.joystick.Joystick(0).init
        clock.tick(60)

        # Scans each action from the joystick
        for event in pygame.event.get():
            # If you press one of the four buttons on the right...
            if event.type == pygame.JOYBUTTONDOWN:
                # Pressing A will open/close the gripper (If one is connected)
                if event.button == 0:
                    print("A Has Been Pressed")
                    if(gripperConnected):
                        gripperOpen = toggleGripper(robot, gripperOpen)
                    else:
                        print("No gripper connected")
                # Pressing the B button is meant to act as an extra way to stop robot motion
                elif event.button == 1:
                    print("B Has Been Pressed")
                    robot.ImmStopJOG()
                    time.sleep(1)
                # Pressing the Y button will clear any errors that might be in the web app (joint limits etc)
                elif event.button == 3:
                    print("Y Has Been Pressed")
                    ResetErrors(robot)
                # If X is pressed, the robot will return to its "Home position" (neutral, standing pose)
                elif event.button == 2:
                    print("X Has Been Pressed")
                    robot.MoveJ(HOME_POS, 0, 0)
                    time.sleep(1)

            # These are for functionality when buttons are released. They currently have no use
            elif event.type == pygame.JOYBUTTONUP:
                if event.button == 0:
                    print("A Has Been Uped")
                elif event.button == 1:
                    print("B Has Been Uped")
                elif event.button == 3:
                    print("Y Has Been Uped")
                elif event.button == 2:
                    print("X Has Been Uped")
            
            # If you're moving the joysticks...
            elif event.type == pygame.JOYAXISMOTION:
                # If movement is done and joystick is neutral, make sure that is_moving is False
                if(is_moving and checkNeutralPos(joystick)):
                    is_moving = False
                # If is_moving is True following normal behavior, skip over the reads to repvent thread spamming
                elif(is_moving):
                    continue
                # Get info on controller axis reads
                axes = joystick.get_numaxes()
                straxis = ""
                fl_axis = []
                # Scan each axis
                for m in range(axes):
                    axis[m] = truncate(float(joystick.get_axis(m) * 1))
                    fl_axis.append(axis[m])
                    straxis = straxis + str(float(axis[m]) * 1) + ","
                print("Controller axices: ", straxis)

                # If left stick is pressed right...
                if(fl_axis[0] > .7):
                    print("L-Right")
                    mv_thread = Thread(target=moveAxisThread, args=[robot, lock, joystick, L_RIGHT])
                    mv_thread.start()
                    is_moving = True

                # If left stick is pressed Left...
                if(fl_axis[0] < -.7):
                    print("L-Left")
                    mv_thread = Thread(target=moveAxisThread, args=[robot, lock, joystick, L_LEFT])
                    mv_thread.start()
                    is_moving = True
                
                # If left stick is pressed down...
                elif(fl_axis[1] > .7):
                    print('L back')
                    mv_thread = Thread(target=moveAxisThread, args=[robot, lock, joystick, L_DOWN])
                    mv_thread.start()
                    is_moving = True
                
                # If left stick is pressed up...
                elif(fl_axis[1] < -.7):
                    print('L Forward')
                    mv_thread = Thread(target=moveAxisThread, args=[robot, lock, joystick, L_UP])
                    mv_thread.start()
                    is_moving = True
                
                # If right stick is pressed right...
                elif(fl_axis[2] > 0.7):
                    print('R Right')
                    mv_thread = Thread(target=moveAxisThread, args=[robot, lock, joystick, R_RIGHT])
                    mv_thread.start()
                    is_moving = True
                
                # If left stick is pressed left...
                elif(fl_axis[2] < -0.7):
                    print('R left')
                    mv_thread = Thread(target=moveAxisThread, args=[robot, lock, joystick, R_LEFT])
                    mv_thread.start()
                    is_moving = True
                
                # If left stick is pressed up...
                elif(fl_axis[3] > 0.7):
                    print('R Back')
                    mv_thread = Thread(target=moveAxisThread, args=[robot, lock, joystick, R_UP])
                    mv_thread.start()
                    is_moving = True
                
                # If left stick is pressed down...
                elif(fl_axis[3] < -0.7):
                    print('R Forward')
                    mv_thread = Thread(target=moveAxisThread, args=[robot, lock, joystick, R_DOWN])
                    mv_thread.start()
                    is_moving = True
                
                # If left trigger is pressed...
                elif(fl_axis[4] > 0):
                    print('lt')
                    mv_thread = Thread(target=moveAxisThread, args=[robot, lock, joystick, LT])
                    mv_thread.start()
                    is_moving = True
                
                # If right trigger is pressed...
                elif(fl_axis[5] > 0):
                    print('rt')
                    mv_thread = Thread(target=moveAxisThread, args=[robot, lock, joystick, RT])
                    mv_thread.start()
                    is_moving = True
                
                                
                # If no motion...
                elif(is_moving and checkNeutralPos(joystick)):
                    err = robot.ImmStopJOG()
                    while(err != 0):
                        time.sleep(0.1)
                        print("ERROR in stopping robot!!!")
                        err = robot.ImmStopJOG()
                    is_moving = False
                time.sleep(0.008)
            if event.type == pygame.QUIT:
                running = False
                pygame.quit()
                sys.exit()
            time.sleep(0.008)


def main():
    gripperConnected = False    # Change this to True if you wish to connect a gripper to this code
    controller_connected = False
    robot_connected = False
    # Waits for controller to appear on the list of devices
    while not controller_connected:
        for device in inputs.devices:
            if 'X-Box' in device.name:
                controller_connected = True
                print("Controller Port Connected")
                break
        if not controller_connected:
            print("Please connect controller...")
            time.sleep(0.5)
    # Waits until robot connection is ebstablished
    while not robot_connected:
        try:
            robot = Robot.RPC(ROBOT_IP)
            robot_connected = True
            break
        except:
            time.sleep(0.5)
            robot_connected = False
    time.sleep(3)
    # If gripper is configured, set above value to True and this will connect it (currently configured for DH)
    if(gripperConnected):
        print(robot.GetGripperConfig())
        print(robot.SetGripperConfig(4,0,0,0))
        time.sleep(.5)
        robot.ActGripper(1,1)
    robot_speed = 40 # Set the Point-To-Point speed of your robot
    robot.SetSpeed(robot_speed)
    run_thread = Thread(target = run, args=[robot, robot_speed])
    run_thread.run()

if __name__ == "__main__":
    main()
    pygame.quit()

