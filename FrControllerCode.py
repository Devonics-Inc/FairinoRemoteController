from ast import Try
import time, sys, pygame
from threading import Thread, Lock
from pygame import joystick
import math
import inputs
import os
os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"
sys.path.insert(0, '/Users/FrTest/Desktop/FrController')
from fairino import Robot


# CONSTANTS
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

# raspi functionality
# PS controller
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


def straightenGripper(robot):
    try:
        e, j_pos = robot.GetActualJointPosDegree()
        j_pos[5] = 0
        j_pos[4] = 90
        robot.MoveJ(j_pos, 0, 0)
    except:
        print("Unexpected error when straighening gripper")
        
def moveToSafety(robot, index, joint_pos, lock):
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
    # print("check Joint limits called")
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
        stop_thread.daemon = True
        stop_thread.start()
        time.sleep(.25)
        # robot.ImmStopJOG()

    
def stopAxis(robot, joystick, JSAxel_index, lock):
    if(JSAxel_index == -1):
        print("No axis recieved by stopAxis()")
        return
    print("Stop axis called")
    while True:
        axel = float(joystick.get_axis(JSAxel_index))
        # print(axel)
        if(JSAxel_index < 4):
            if abs(axel) < 0.1:
                try:
                    robot.ImmStopJOG()
                    print("Joystick released")
                except:
                    print("ERROR in stopping robot!!!")
                break
        else:
             if abs(axel) >= 1:
                try:
                    robot.ImmStopJOG()
                    print("Joystick released")
                except:
                    print("ERROR in stopping robot!!!")
                break
        if checkJointLimits(robot, lock):
            try:
                robot.ImmStopJOG()
                print("Too close to soft limit")
            except:
                print("ERROR in stopping robot!!!")
            break
        if(robot.GetRobotMotionDone() == 1):
            print("Robot done moving. Ending stopThread()")
            break
        time.sleep(0.008)

def ResetErrors(robot):
    print("Error reset activated")
    err = robot.ResetAllError()
    print(f"ResetAllError() response: {err}")
    if(err !=  0):
        print("Robot broke safety conditions. Please check WebApp for error msg")

def run(robot, robot_speed):
    gripperOpen = True
    pygame.init()
    screen = pygame.display.set_mode((1280, 720))
    screen.fill("purple")
    pygame.display.flip()
    lock = Lock()
    clock = pygame.time.Clock()
    joy_stat = 0
    running = True
    # joysticks = []
    axis = [0, 0, 0, 0, 0, 0]
    straxis = "0,0,0,0,"
    while running:
        pygame.joystick.init()
        joystick_count = pygame.joystick.get_count()
        while pygame.joystick.get_count() != 0 and joy_stat == 0:
            for i in range(joystick_count):
                joystick = pygame.joystick.Joystick(i)
                joystick.init()
                print("Controller connected")
                time.sleep(1)
                joy_stat = 1
            break
        while pygame.joystick.get_count() == 0:
            pygame.joystick.quit()
            joy_stat = 0
            print("Waiting for controller to connect...")
            time.sleep(1)
            pygame.joystick.init()
            continue
            break	

        
        pygame.joystick.Joystick(0).init
        clock.tick(60)
        
        # print(i for i in pygame.event.get() if i.type == pygame.JOYAXISMOTION)
        # print(joystick_count)
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                if event.button == 0:
                    print("A Has Been Pressed")
                    gripperOpen = toggleGripper(robot, gripperOpen)
                elif event.button == 1:
                    print("B Has Been Pressed")
                    robot.MoveJ(HOME_POS, 0, 0)
                    time.sleep(3)
                elif event.button == 3:
                    print("Y Has Been Pressed")
                    ResetErrors(robot)
                elif event.button == 2:
                    print("X Has Been Pressed")
                    robot.MoveJ(PACKING_POS, 0, 0)
                    time.sleep(3)
            elif event.type == pygame.JOYBUTTONUP:
                if event.button == 0:
                    print("A Has Been Uped")
                elif event.button == 1:
                    print("B Has Been Uped")
                elif event.button == 3:
                    print("Y Has Been Uped")
                elif event.button == 2:
                    print("X Has Been Uped")
            
            elif event.type == pygame.JOYAXISMOTION:
                # joystick_s = pygame.joystick.Joystick(0)
                axes = joystick.get_numaxes()
                # up, left, down, right
                straxis = ""
                fl_axis = []
                for m in range(axes):
                    axis[m] = truncate(float(joystick.get_axis(m) * 1))
                    fl_axis.append(axis[m])
                    straxis = straxis + str(float(axis[m]) * 1) + ","
                print("Straxis: ", straxis)
                if(fl_axis[0] > .7):
                    print("L-Right")
                    mv_thread = Thread(target=moveAxisThread, args=[robot, lock, joystick, L_RIGHT])
                    #mv_thread.daemon = True
                    mv_thread.start()
                if(fl_axis[0] < -.7):
                    print("L-Left")
                    mv_thread = Thread(target=moveAxisThread, args=[robot, lock, joystick, L_LEFT])
                    #mv_thread.daemon = True
                    mv_thread.start()
                elif(fl_axis[1] > .7):
                    print('L back')
                    mv_thread = Thread(target=moveAxisThread, args=[robot, lock, joystick, L_DOWN])
                    #mv_thread.daemon = True
                    mv_thread.start()
                elif(fl_axis[1] < -.7):
                    print('L Forward')
                    mv_thread = Thread(target=moveAxisThread, args=[robot, lock, joystick, L_UP])
                    #mv_thread.daemon = True
                    mv_thread.start()
                elif(fl_axis[2] > 0.7):
                    print('R Right')
                    mv_thread = Thread(target=moveAxisThread, args=[robot, lock, joystick, R_RIGHT])
                    #mv_thread.daemon = True
                    mv_thread.start()
                elif(fl_axis[2] < -0.7):
                    print('R left')
                    mv_thread = Thread(target=moveAxisThread, args=[robot, lock, joystick, R_LEFT])
                    #mv_thread.daemon = True
                    mv_thread.start()
                elif(fl_axis[3] > 0.7):
                    print('R Back')
                    mv_thread = Thread(target=moveAxisThread, args=[robot, lock, joystick, R_UP])
                    #mv_thread.daemon = True
                    mv_thread.start()
                elif(fl_axis[3] < -0.7):
                    print('R Forward')
                    mv_thread = Thread(target=moveAxisThread, args=[robot, lock, joystick, R_DOWN])
                    #mv_thread.daemon = True
                    mv_thread.start()
                elif(fl_axis[4] > 0):
                    print('lt')
                    mv_thread = Thread(target=moveAxisThread, args=[robot, lock, joystick, LT])
                    #mv_thread.daemon = True
                    mv_thread.start()
                elif(fl_axis[5] > 0):
                    print('rt')
                    # moveAxisThread(robot, lock, joystick, 'rt')
                    mv_thread = Thread(target=moveAxisThread, args=[robot, lock, joystick, RT])
                    #mv_thread.daemon = True
                    mv_thread.start()
                
                elif(checkNeutralPos(joystick)):
                    try:
                        robot.ImmStopJOG()
                        print("Neutral stick detected when robot is in motion")
                    except:
                        print("Error stopping robot from main loop")
                time.sleep(0.006)
                # print(straxis)
            if event.type == pygame.QUIT:
                running = False
                pygame.quit()
                sys.exit()
            time.sleep(0.008)


def main():
    # flip() the display to put your work on screen
    controller_connected = False
    robot_connected = False
    #for device in inputs.devices:
    #    print(device.name)
    while not controller_connected:
        for device in inputs.devices:
            if 'X-Box' in device.name:
                controller_connected = True
                print("Controller Port Connected")
                break
        if not controller_connected:
            print("Please connect controller...")
            time.sleep(0.5)
    while not robot_connected:
        try:
            robot = Robot.RPC('192.168.58.32')
            robot_connected = True
            break
        except:
            time.sleep(0.5)
            robot_connected = robot.connected
    time.sleep(3)
    print(robot.GetGripperConfig())
    print(robot.SetGripperConfig(4,0,0,0))
    time.sleep(.5)
    robot.ActGripper(1,1)
    robot_speed = 55
    robot.SetSpeed(robot_speed)
    run_thread = Thread(target = run, args=[robot, robot_speed])
    # run_thread.daemon = True
    run_thread.run()

if __name__ == "__main__":
    main()
    pygame.quit()

