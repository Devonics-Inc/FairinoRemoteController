import time, sys, pygame
import math
import os
import csv
import msvcrt
import numpy as np
from threading import Thread, Lock, Event

sys.path.insert(0, '/Users/FrTest/Desktop/FrController')
from fairino import Robot

os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"

csv_file = "coordinates.csv"

gripper_pos = 100

pick_dict = {
    1 : [-60.58,-86.81,104.51,-107.38,-92.29,-60.96],
    2 : [-79.15,-99.21,115.95,-105.3,-92.3,-80.08],
    3 : [-105.74,-104.74,121.23,-103.26,-91.19,-105.64],
    4 : [-128.65,-101.9,119.44,-104.68,-91.46,-125.14]
}

place_dict = {
    'P' : [82.65,-103.32,118.96,-103.44,-87.52,81.23],
    'G' : [66.38,-98.67,114.01,-98.0,-92.76,62.26],
    'S' : [48.31,-92.6,112.4,-105.1,-94.33,44.22],
    'H' : [36.08,-80.17,97.94,-103.81,-94.88,31.96]
}

MIDPOINT = [-14.05,-82.72,99.93,-104.53,-88.67,-20.8]
POINT1 = [-38.2,-76.35,97.28,-109.57,-87.87,-49.99]
POINT2 = [12.67,-75.5,96.81,-111.71,-93.84,-0.01]
HOME = [-9.54,-80.59,74.67,-81.7,-92.29,-10.86]

MIDPOINT_UP = []
POINT1_UP = []
POINT2_UP = []

TARGET1 = POINT1
TARGET2 = POINT2


JOG_ACCL = 100
JOG_SPEED = 100
incr = 1.0
THRESHOLD = 0.3

SOFT_LIMIT = [-160.0, 160.0, -250.0, 70.0, -145.0,
              145.0, -250.0, 70.0, -160.0, 160.0, -160.0, 160.0]

L_LEFT = 'll'; L_RIGHT = 'lr'; L_UP = 'lu'; L_DOWN = 'ld'
R_LEFT = 'rl'; R_RIGHT = 'rr'; R_UP = "ru"; R_DOWN = "rd"
LT = 'lt'; RT = 'rt'; NO = 'no'

A = 'A'; B = 'B'; X = 'X'; Y = 'Y'; LB = 'LB'; RB = 'RB'; JR = 'JR'; JL = 'JL'

joystick_cmd = None
cmd_lock = Lock()
stop_flag = Event()
motion_thread = None
motion_stop_event = Event()

pick_running = False
pick_thread = None
stop_pick = None

gripper_open = False
stop_db_event = None

db_running = False
db_thread = None

def runLuaScript(robot):
    robot.Mode(0) #Robot cuts to autorun mode
    ret = robot.ProgramLoad('/fruser/testinglua.lua') #Load the robot program to be executed, the testPTP.lua program needs to be written first on the webapp
    print("Error code loading robot program to be executed", ret)
    ret = robot.ProgramRun() #Execute the robot program
    print("Execute robot program error code", ret)
    time.sleep(2)
    error, val = robot.GetSysVarValue(3)
    print("System variable number:", 3, "value", val)
    return val

def writetocsv(coordinate):
    try:
            with open(csv_file, 'a', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow(['J1', 'J2', 'J3', 'J4', 'J5', 'J6'])  # Write header row
                csv_writer.writerow(round(num, 2) for num in coordinate)  # Write coordinate data
            print(f"Coordinates saved to '{csv_file}' successfully.")
    except Exception as e:
            print(f"An error occurred: {e}")


def truncate(number):
    factor = 10.0 ** 2
    return math.trunc(number * factor) / factor

def checkJointLimits(j_pos):
    for i in range(len(j_pos) - 1):
        if (j_pos[i] - SOFT_LIMIT[i * 2] <= 1) or (SOFT_LIMIT[i * 2 + 1] - j_pos[i] <= 1):
            print("Error - Approaching joint limit!")
            return False
    return True

def ResetErrors(robot):
    print("Error reset activated")
    err = robot.ResetAllError()
    print(f"ResetAllError() response: {err}")

def getServoCommand(mv_input):
    delta = [0.0] * 6
    if mv_input == L_LEFT: delta[0] = -incr
    elif mv_input == L_RIGHT: delta[0] = incr
    elif mv_input == L_UP: delta[1] = -incr
    elif mv_input == L_DOWN: delta[1] = incr
    elif mv_input == R_UP: delta[2] = incr
    elif mv_input == R_DOWN: delta[2] = -incr
    elif mv_input == R_LEFT: delta[4] = -incr
    elif mv_input == R_RIGHT: delta[4] = incr
    elif mv_input == LT: delta[3] = 0
    elif mv_input == RT: delta[3] = 0
    return delta

def getDelta(mv_input):
    DELTA = [0,0,0,0,0,0,0] #DELTA SERVO CART
    # targetC = robot.getForwardKin(targetJ)[1] #CARTESIAN COORDINATES
    thresh = 0.3
    #calculate delta in cartesian coordinates
    #d = incr * (mv_input[] - thresh)/thresh

    DELTA[ 0 ]  = ( mv_input[ 0 ] * -incr ) if abs( mv_input[ 0 ]) > 0.3 else 0.0
    DELTA[ 1 ]  = ( mv_input[ 1 ] * incr ) if abs( mv_input[ 1 ]) > 0.3 else 0.0
    DELTA[ 2 ]  = ( mv_input[ 3 ] * incr ) if abs( mv_input[ 3 ]) > 0.3 else 0.0
    DELTA[ 3 ]  = ( mv_input[ 4 ] * 0.00 ) if abs( mv_input[ 4 ]) > 0.3 else 0.0
    DELTA[ 4 ]  = ( mv_input[ 2 ] * -incr ) if abs( mv_input[ 2 ]) > 0.65 else 0.0
    return DELTA
    

def getJoystickCommand(axes):
    DEAD_ZONE = 0.15  # Ignore small axis values
    if abs(axes[0]) > THRESHOLD: return L_RIGHT if axes[0] > 0 else L_LEFT
    if abs(axes[1]) > THRESHOLD: return L_DOWN if axes[1] > 0 else L_UP
    if abs(axes[2]) > THRESHOLD: return R_RIGHT if axes[2] > 0 else R_LEFT
    if abs(axes[3]) > THRESHOLD: return R_UP if axes[3] > 0 else R_DOWN
    if all(abs(axis) < DEAD_ZONE for axis in axes):
        return None
    return None  # Default to None to stop motion

def toggleGripper(robot, state):
    global gripper_open, gripper_pos
    if not state:
        gripper_pos = 100
        print("Opening gripper")
        robot.MoveGripper(1, 100, 100, 30, 30000, 0)
    else:
        gripper_pos = 50
        print("Closing gripper")
        robot.MoveGripper(1, 0, 100, 30, 30000, 0)
    time.sleep(.8)
    gripper_open = not state
    return not state

def openGripper(robot):
    global gripper_pos, gripper_open
    gripper_open = True
    gripper_pos = 100
    robot.MoveGripper(1, 100, 100, 30, 30000, 0)

def closeGripper(robot):
    global gripper_pos, gripper_open
    gripper_open = False
    print("Closing gripper")
    gripper_pos -= 10
    if gripper_pos <= 40: gripper_pos = 40
    robot.MoveGripper(1, gripper_pos, 100, 30, 30000, 0)

def SaveTarget1(robot):
    global TARGET1
    err, TARGET1 = robot.GetActualJointPosDegree()
    print("Saved Target 1:", TARGET1)

def SaveTarget2(robot):
    global TARGET2
    err, TARGET2 = robot.GetActualJointPosDegree()
    print("Saved Target 2:", TARGET2)

def ResetTarget1():
    global TARGET1
    TARGET1 = POINT1
    print("Reset to default Target 1")

def ResetTarget2():
    global TARGET2
    TARGET2 = POINT2
    print("Reset to default Target 2")

def CalculateMid(robot):
    global MID_POS
    tcp1 = robot.GetForwardKin(TARGET1)[1]
    tcp2 = robot.GetForwardKin(TARGET2)[1]
    midpos = [ (tcp1[i] + tcp2[i]) / 2 for i in range(3) ]
    midpos += tcp1[3:]
    midpos[2] += 80
    MID_POS = robot.GetInverseKin(type=0, desc_pos=midpos, config=-1)[1]

def CalculateZoffset(robot, Jpose, up = 1):
    err, tcp = robot.GetForwardKin(Jpose)
    tcp_new = tcp
    if up :
        tcp_new[2] += 60
    else:
        tcp_new[2] -= 60
    
    #print(tcp_new)
    err, x = robot.GetInverseKin(type=0, desc_pos=tcp_new, config=-1)
    #print(err, x, tcp_new)
    return(x)

def Pick(robot):
    global POINT1, POINT2, MIDPOINT, POINT1_UP, POINT2_UP, MIDPOINT_UP, gripper_open, stop_pick
    if stop_pick.is_set():
        print("Pick Loop interrupted!")
        return
    try:
        if stop_pick.is_set():
            print("Pick Loop interrupted before POINT1_UP!")
            return
        robot.MoveJ(POINT1_UP, 0, 0)

        if stop_pick.is_set():
            print("Pick Loop interrupted before POINT1!")
            return
        robot.MoveJ(POINT1, 0, 0)

        if stop_pick.is_set():
            print("Pick Loop interrupted before first toggleGripper!")
            return
        gripper_open = toggleGripper(robot, gripper_open)

        if stop_pick.is_set():
            print("Pick Loop interrupted before second toggleGripper!")
            return
        gripper_open = toggleGripper(robot, gripper_open)

        if stop_pick.is_set():
            print("Pick Loop interrupted before POINT1_UP return!")
            return
        robot.MoveJ(POINT1_UP, 0, 0)

        if stop_pick.is_set():
            print("Pick Loop interrupted before POINT2_UP!")
            return
        robot.MoveJ(POINT2_UP, 0, 0)

        if stop_pick.is_set():
            print("Pick Loop interrupted before POINT2!")
            return
        robot.MoveJ(POINT2, 0, 0)

        if stop_pick.is_set():
            print("Pick Loop interrupted before third toggleGripper!")
            return
        gripper_open = toggleGripper(robot, gripper_open)

        if stop_pick.is_set():
            print("Pick Loop interrupted before fourth toggleGripper!")
            return
        gripper_open = toggleGripper(robot, gripper_open)

        if stop_pick.is_set():
            print("Pick Loop interrupted before final POINT2_UP!")
            return
        robot.MoveJ(POINT2_UP, 0, 0)
    except Exception as e:
        print(f"Error in Pick Loop: {e}")
        return

def runPickLoop(robot, event):
    global pick_running, gripper_open
    try:
        while not event.is_set():
            Pick(robot)
            time.sleep(0.5)  # Non-interruptible sleep
    except Exception as e:
        print(f"Error in Pick Loop: {e}")
    finally:
        pick_running = False

def togglePick(robot):
    global pick_running, pick_thread, gripper_open, stop_pick

    pick_running = not pick_running

    if pick_running:
        print("Going MIDPOINT")
        try:
            if not gripper_open:
                gripper_open = toggleGripper(robot, gripper_open)
                time.sleep(0.8)  # Non-interruptible sleep
            GoTO(robot, MIDPOINT)
            print("Starting Pick loop")
            stop_pick = Event()
            pick_thread = Thread(target=runPickLoop, args=(robot, stop_pick,))
            pick_thread.start()
        except Exception as e:
            print(f"Failed to start Pick loop: {e}")
            pick_running = False
            stop_pick = None
    else:
        print("Stopping Pick loop")
        try:
            if stop_pick:
                stop_pick.set()
            if pick_thread and pick_thread.is_alive():
                pick_thread.join()
        except Exception as e:
            print(f"Error stopping Pick loop: {e}")
        finally:
            stop_pick = None
            pick_thread = None

def GoTO(robot, P1):
    global gripper_open
    P1_UP = CalculateZoffset(robot, P1, up=1)

    robot.MoveJ(P1_UP, 0, 0)
    robot.MoveJ(P1, 0, 0)
    gripper_open = toggleGripper(robot, gripper_open)
    time.sleep(0.8)
    robot.MoveJ(P1_UP, 0, 0)

def Assess(value):
    if abs(value - 32) < 2:
        return 'P'
    elif abs(value - 34) < 2:
        return 'G'
    elif abs(value - 54) < 2:
        return 'S'
    elif abs(value - 69) < 2:
        return 'H'

def PickDiffBalls(robot):
    global gripper_open
    place_seq = ['P', 'G', 'S', 'H']
    pick_seq = [1, 2, 3, 4]
    if not gripper_open:
            gripper_open = toggleGripper(robot, gripper_open)
            time.sleep(.8)
    for index in range(4):
        if stop_db_event.is_set():  # Check if stop was requested
            print("PickDiffBalls interrupted!")
            return  # Exit gracefully
        try:
            GoTO(robot, pick_dict[pick_seq[index]])
            robot.MoveJ(HOME, 0, 0)
            time.sleep(2)
            val = runLuaScript(robot)
            ball = Assess(val)
            print(f"Assessed ball: {ball}")
            GoTO(robot, place_dict[ball])
            robot.MoveJ(HOME, 0, 0)
        except Exception as e:
            print(f"Error in PickDiffBalls: {e}")
            return 

def runDiffBallsLoop(robot):
    global db_running
    try:
        PickDiffBalls(robot)
    except Exception as e:
        print(f"Error in runDiffBallsLoop: {e}")
    finally:
        db_running = False  

def toggleDiffBalls(robot):
    global db_running, db_thread, stop_db_event

    if not db_running:
        print("Starting All Diff Balls loop")
        try:
            stop_db_event = Event()  
            db_running = True
            db_thread = Thread(target=runDiffBallsLoop, args=(robot,))
            db_thread.start()
        except Exception as e:
            print(f"Failed to start thread: {e}")
            db_running = False
            stop_db_event = None
    else:
        print("Stopping All Diff Balls loop")
        try:
            db_running = False
            if stop_db_event:
                stop_db_event.set()  # Signal thread to stop
            if db_thread and db_thread.is_alive():
                db_thread.join()  # Wait for thread to terminate
        except Exception as e:
            print(f"Error stopping thread: {e}")
        finally:
            # Clean up global state
            stop_db_event = None
            db_thread = None

def motion_execution(robot, joy, initial_axes, stop_event):
    initial_cmd = getJoystickCommand(initial_axes)
    if not initial_cmd:
        return
    print(f"Starting motion with command: {initial_cmd}")

    robot.ServoMoveStart()
    try:
        current_cmd = initial_cmd
        while not stop_event.is_set():
            # Poll joystick axes
            axes = [truncate(joy.get_axis(i)) for i in range(joy.get_numaxes())]
            print(axes)
            current_cmd = getJoystickCommand(axes)

            # Stop if joystick is neutral
            if current_cmd is None:
                print("Joystick neutral, stopping motion")
                break

            # Apply motion for the current command
            delta = getServoCommand(current_cmd)
            #print(f"Delta basic : {delta}")

            diag = getDelta(axes)
            #print(f"Delta Diagonal : {diag}")

            currentTCP = robot.GetActualTCPPose()[1]
            futureTCP = [currentTCP[i] + delta[i] for i in range(6)]
            futureJ = robot.GetInverseKin(type=0, desc_pos=futureTCP, config=-1)[1]

            if checkJointLimits(futureJ):
                pass
                #robot.ServoCart(2, diag, vel=JOG_SPEED, acc=JOG_ACCL)
            else:
                print("Joint limit reached, stopping motion")
                break

            time.sleep(0.008)  # Tight loop for responsive control
    finally:
        robot.ServoMoveEnd()
        print("Motion execution ended")

def joystick_input_thread(robot, joy):
    print("Accepting Joystick Input (Buttons and Initial Motion)")
    global joystick_cmd, motion_thread, motion_stop_event, gripper_open
    button_states = [False] * 10

    while not stop_flag.is_set():
        # Poll buttons only
        buttons = [joy.get_button(i) for i in range(10)]

        # Check for new motion only if no motion thread is running
        if not (motion_thread and motion_thread.is_alive()):

            axes = [truncate(joy.get_axis(i)) for i in range(joy.get_numaxes())]
            with cmd_lock:
                AXES = axes
                joystick_cmd = getJoystickCommand(AXES)
            if joystick_cmd:
                print(f"Starting new motion thread with command: {joystick_cmd}")
                motion_stop_event = Event()
                motion_thread = Thread(target=motion_execution, args=(robot, joy, AXES, motion_stop_event))
                motion_thread.start()

        # Handle button presses
        for i in range(len(buttons)):
            if buttons[i] and not button_states[i]:
                print(f"Button {i} pressed")
                if i == 0:  # A
                    gripper_open = toggleGripper(robot, gripper_open)
                elif i == 1:  # B
                    togglePick(robot)
                elif i == 2:  # X
                    toggleDiffBalls(robot)
                elif i == 3:  # Y
                    robot.MoveJ(HOME, 0, 0)
                elif i == 4:  # LB
                    closeGripper(robot)
                elif i == 5:  # RB
                    openGripper(robot)
                elif i == 8:
                    ResetErrors(robot)
                elif i == 9:
                    err, Jpose = robot.GetActualJointPosDegree()
                    writetocsv(Jpose)
            button_states[i] = buttons[i]

        time.sleep(0.01)  # Slower polling for buttons


def run(robot, robot_speed):
    global stop_flag
    pygame.init()
    screen = pygame.display.set_mode((640, 480))
    pygame.display.set_caption("Joystick Control")
    screen.fill("purple")
    pygame.display.flip()

    pygame.joystick.init()

    while pygame.joystick.get_count() == 0:
        print("Waiting for controller...")
        pygame.joystick.quit()
        pygame.joystick.init()
        time.sleep(1)

    joy = pygame.joystick.Joystick(0)
    joy.init()
    print("Controller connected")

    robot.SetSpeed(robot_speed)
    poll_thread = Thread(target=joystick_input_thread, args=(robot, joy))
    poll_thread.start()

    try:
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
            time.sleep(0.01)
    finally:
        print("Shutting down...")
        stop_flag.set()
        poll_thread.join()

        if motion_thread and motion_thread.is_alive():
            motion_stop_event.set()
            motion_thread.join()

        if pick_thread and pick_thread.is_alive():
            pick_running = False
            pick_thread.join()

        pygame.display.quit()
        pygame.quit()


def main():
    global POINT1, POINT2, MIDPOINT, POINT1_UP, POINT2_UP, MIDPOINT_UP
    robot = None
    while not robot:
        try:
            robot = Robot.RPC('192.168.57.2')
        except Exception as e:
            print("Waiting for robot connection...", e)
            time.sleep(0.5)

    robot_speed = 100
    ResetErrors(robot)
    CalculateMid(robot)

    POINT1_UP   = CalculateZoffset(robot, POINT1, up=1)
    POINT2_UP   = CalculateZoffset(robot, POINT2, up=1)
    MIDPOINT_UP   = CalculateZoffset(robot, MIDPOINT, up=1)

    robot.SetGripperConfig(4, 0)
    robot.ActGripper(1, 0)
    robot.ActGripper(1, 1)
    global gripper_open
    if not gripper_open:
        gripper_open = toggleGripper(robot, gripper_open)
        
    robot.SetSpeed(robot_speed)
    robot.SetOaccScale(100)

    try:
        run(robot, robot_speed)
    except KeyboardInterrupt:
        print("\n[!] KeyboardInterrupt received. Cleaning up...")
        stop_flag.set()
        if motion_thread and motion_thread.is_alive():
            motion_stop_event.set()
            motion_thread.join()
        if pick_thread and pick_thread.is_alive():
            pick_running = False
            pick_thread.join()
        pygame.quit()
        sys.exit(0)

if __name__ == "__main__":
    main()
