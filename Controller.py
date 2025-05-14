import time, sys, pygame
import math
import os
import csv
import msvcrt
import numpy as np
import threading
from threading import Thread, Lock, Event

sys.path.insert(0, '/Users/FrTest/Desktop/FrController')
from fairino import Robot

os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"


csv_file = "coordinates.csv"

gripper_pos = 100

pick_dict = {
    1 : [-63.16,-88.4,107.05,-108.65,-90.0,-63.16],
    2 : [-82.42,-102.49,119.18,-106.69,-90.0,-82.42],
    3 : [-107.69,-110.2,126.51,-106.3,-90.0,-107.69],
    4 : [-130.48,-107.0,124.62,-107.61,-90.0,-130.48]
}

place_dict = {
    'P' : [86.34,-103.66,121.54,-107.88,-90.0,86.34],
    'G' : [63.45,-107.31,122.09,-104.78,-90.0,63.45],
    'S' : [44.42,-98.82,118.52,-109.7,-90.0,44.42],
    'H' : [31.86,-85.87,104.64,-108.78,-90.0,31.86]
}

MIDPOINT = [-13.52,-85.44,103.75,-108.31,-90.0,-13.52]
POINT1 = [-37.65,-76.45,98.0,-111.55,-90.0,-37.65]
POINT2 = [10.44,-76.51,97.89,-111.38,-90.0,10.44]
HOME = [-10.14,-81.78,77.05,-85.27,-90.0,-10.14]

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

SAFE_CART_LIMITS = [-474, 111, -365, 302, 26, 225, 150, 184, -74, 74, 45, 135]

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

drop_running = False
drop_thread = None
stop_drop_event = None

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
                csv_writer.writerow(['X', 'Y', 'Z', 'RX', 'RY', 'RZ'])  # Write header row
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

def CheckSafetyPlanes(tcpPose):
    violations = [None] * 6  # None (no violation), 'min' (at min limit), or 'max' (at max limit)
    for i in range(len(tcpPose)):
        if i == 3:
            tcpPose[i] = abs(tcpPose[i])
        if tcpPose[i] <= SAFE_CART_LIMITS[i * 2] + 1:
            print(f"Approaching or beyond min Cartesian limit for axis {i}: {tcpPose[i]} <= {SAFE_CART_LIMITS[i * 2]}")
            violations[i] = 'min'
        elif tcpPose[i] >= SAFE_CART_LIMITS[i * 2 + 1] - 1:
            print(f"Approaching or beyond max Cartesian limit for axis {i}: {tcpPose[i]} >= {SAFE_CART_LIMITS[i * 2 + 1]}")
            violations[i] = 'max'
    return violations

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
    DELTA[ 1 ]  = ( mv_input[ 1 ] * -incr ) if abs( mv_input[ 1 ]) > 0.3 else 0.0
    DELTA[ 2 ]  = ( mv_input[ 3 ] * incr ) if abs( mv_input[ 3 ]) > 0.3 else 0.0
    DELTA[ 3 ]  = ( mv_input[ 4 ] * 0.00 ) if abs( mv_input[ 4 ]) > 0.3 else 0.0
    DELTA[ 4 ]  = ( mv_input[ 2 ] * 0.00 ) if abs( mv_input[ 2 ]) > 0.65 else 0.0
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
        robot.MoveGripper(1, 100, 100, 30, 30000,0,0,0,0,0)
    else:
        gripper_pos = 50
        print("Closing gripper")
        robot.MoveGripper(1, 0, 100, 30, 30000,0,0,0,0,0)
    time.sleep(.8)
    gripper_open = not state
    return not state

def openGripper(robot):
    global gripper_pos, gripper_open
    gripper_open = True
    gripper_pos = 100
    robot.MoveGripper(1, 100, 100, 30, 30000,0,0,0,0,0)

def closeGripper(robot):
    global gripper_pos, gripper_open
    gripper_open = False
    print("Closing gripper")
    gripper_pos -= 10
    if gripper_pos <= 40: gripper_pos = 40
    robot.MoveGripper(1, gripper_pos, 100, 30, 30000,0,0,0,0,0,0)

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
    print(robot.GetForwardKin(Jpose))
    err, tcp = robot.GetForwardKin(Jpose)
    print(tcp)
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
        robot.MoveJ(POINT1_UP, 1, 0, vel = JOG_SPEED, acc = JOG_ACCL)

        if stop_pick.is_set():
            print("Pick Loop interrupted before POINT1!")
            return
        robot.MoveJ(POINT1, 1, 0, vel = JOG_SPEED, acc = JOG_ACCL)

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
        robot.MoveJ(POINT1_UP, 1, 0, vel = JOG_SPEED, acc = JOG_ACCL)

        if stop_pick.is_set():
            print("Pick Loop interrupted before POINT2_UP!")
            return
        robot.MoveJ(POINT2_UP, 1, 0, vel = JOG_SPEED, acc = JOG_ACCL)

        if stop_pick.is_set():
            print("Pick Loop interrupted before POINT2!")
            return
        robot.MoveJ(POINT2, 1, 0, vel = JOG_SPEED, acc = JOG_ACCL)

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
        robot.MoveJ(POINT2_UP, 1, 0, vel = JOG_SPEED, acc = JOG_ACCL)
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

    robot.MoveJ(P1_UP, 1, 0, vel = JOG_SPEED, acc = JOG_ACCL)
    robot.MoveJ(P1, 1, 0, vel = JOG_SPEED, acc = JOG_ACCL)
    gripper_open = toggleGripper(robot, gripper_open)
    time.sleep(0.8)
    robot.MoveJ(P1_UP, 1, 0, vel = JOG_SPEED, acc = JOG_ACCL)

def Assess(value):
    if abs(value - 32) < 2:
        return 'P'
    elif abs(value - 34) < 2:
        return 'G'
    elif abs(value - 54) <= 2:
        return 'S'
    elif abs(value - 69) <= 2:
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
            robot.MoveJ(HOME, 1, 0, vel = JOG_SPEED, acc = JOG_ACCL)
            time.sleep(2)
            val = runLuaScript(robot)
            ball = Assess(val)
            print(f"Assessed ball: {ball}")
            GoTO(robot, place_dict[ball])
            robot.MoveJ(HOME, 1, 0, vel = JOG_SPEED, acc = JOG_ACCL)
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


def DropatLoc(robot):
    global gripper_open
    if stop_drop_event and stop_drop_event.is_set():
        print("DropatLoc interrupted!")
        return
    try:
        currPos = robot.GetActualJointPosDegree()[1]
        up = CalculateZoffset(robot, currPos, up=1)
        robot.MoveJ(up, 1, 0, vel = JOG_SPEED, acc = JOG_ACCL)
        robot.MoveJ(HOME, 1, 0, vel = JOG_SPEED, acc = JOG_ACCL)
        time.sleep(2)
        val = runLuaScript(robot)
        ball = Assess(val)
        print(f"Assessed ball: {ball}")
        GoTO(robot, place_dict[ball])
    except Exception as e:
        print(f"Error in DropatLoc: {e}")
        return

def runDropatLoc(robot):
    global drop_running
    try:
        DropatLoc(robot)
    except Exception as e:
        print(f"Error in runDropatLoc: {e}")
    finally:
        drop_running = False


def reconfigure_gripper(robot):
    global gripper_open
    robot.SetGripperConfig(4, 0)
    robot.ActGripper(1, 0)
    robot.ActGripper(1, 1)
    global gripper_open
    if not gripper_open:
        gripper_open = toggleGripper(robot, gripper_open)


def motion_execution(robot, joy, stop_event):
    """
    Execute continuous motion based on joystick input using getDelta.
    """
    print("Starting motion execution")
    try:
        robot.ServoMoveStart()
        while not stop_event.is_set():
            axes = [truncate(joy.get_axis(i)) for i in range(joy.get_numaxes())]
            delta = getDelta(axes)

            if all(abs(d) < 0.01 for d in delta[:6]):
                print("Joystick neutral, stopping motion")
                break

            try:
                currentTCP = robot.GetActualTCPPose()[1]
                print(f"Current TCP: {currentTCP}")
                print(f"Delta: {delta}")
                
                violations = CheckSafetyPlanes(currentTCP)
                
                # Compute initial future TCP
                futureTCP = [currentTCP[i] - delta[i] for i in range(6)]
                
                # Adjust delta to reflect clamped futureTCP
                filtered_delta = delta[:]
                filter_index = [1, 0, 2, 3, 4, 5]
                # Block motion that worsens violations
                for i in range(6):
                    if i == 2:  # Special handling for Z-axis
                        if violations[i] == 'min' and filtered_delta[filter_index[i]] > 0:  # Block positive delta (moves Z down)
                            print(f"Blocking downward motion for Z-axis (at min limit)")
                            filtered_delta[filter_index[i]] = 0
                        elif violations[i] == 'max' and filtered_delta[filter_index[i]] < 0:  # Block negative delta (moves Z up)
                            print(f"Blocking upward motion for Z-axis (at max limit)")
                            filtered_delta[filter_index[i]] = 0
                    else:  # Other axes (X, Y, RX, RY, RZ)
                        if violations[i] == 'min' and filtered_delta[filter_index[i]] < 0:
                            print(f"Blocking negative motion for axis {i} (at min limit)")
                            filtered_delta[filter_index[i]] = 0
                        elif violations[i] == 'max' and filtered_delta[filter_index[i]] > 0:
                            print(f"Blocking positive motion for axis {i} (at max limit)")
                            filtered_delta[filter_index[i]] = 0
                
                print(f"Filtered Delta: {filtered_delta}")
                # Check joint limits
                futureJ = robot.GetInverseKin(type=0, desc_pos=futureTCP, config=-1)[1]
                if checkJointLimits(futureJ):
                    if any(abs(d) >= 0.01 for d in filtered_delta[:6]):
                        robot.ServoCart(2, filtered_delta, vel=JOG_SPEED, acc=JOG_ACCL)
                        print(f"Future TCP: {futureTCP}")
                    else:
                        print("No valid motion allowed, skipping ServoCart")
                else:
                    print(f"Future TCP: {futureTCP}")
                    print("Joint limit reached, stopping motion")
                    stop_event.set()
                    break
            except Exception as e:
                print(f"Robot command error: {e}")
                break

            time.sleep(0.01)
    except Exception as e:
        print(f"Motion execution error: {e}")
    finally:
        try:
            robot.ServoMoveEnd()
        except Exception as e:
            print(f"Error ending servo move: {e}")
        print("Motion execution ended")
        
def joystick_input_thread(robot, joy):
    """
    Poll joystick for inputs, start motion threads based on getDelta, and handle buttons.
    """
    print("Accepting Joystick Input")
    global motion_thread, motion_stop_event, gripper_open, drop_running, drop_thread, stop_drop_event
    button_states = [False] * 10

    while not stop_flag.is_set():
        # Poll buttons and axes
        buttons = [joy.get_button(i) for i in range(10)]
        axes = [truncate(joy.get_axis(i)) for i in range(joy.get_numaxes())]
        delta = getDelta(axes)

        # Manage motion thread
        if motion_thread and motion_thread.is_alive():
            # Stop motion if joystick is neutral
            if all(abs(d) < 0.01 for d in delta[:6]):
                motion_stop_event.set()
                motion_thread.join()
                motion_thread = None
        else:
            # Start new motion thread if joystick is active
            if any(abs(d) >= 0.01 for d in delta[:6]):
                print("Starting new motion thread")
                motion_stop_event = Event()
                motion_thread = Thread(target=motion_execution, args=(robot, joy, motion_stop_event))
                motion_thread.start()

        # Handle button presses
        for i in range(len(buttons)):
            if buttons[i] and not button_states[i]:
                print(f"Button {i} pressed")
                if i == 0:  # A
                    if not drop_running:
                        print("Starting DropatLoc")
                        try:
                            stop_drop_event = Event()
                            drop_running = True
                            drop_thread = Thread(target=runDropatLoc, args=(robot,))
                            drop_thread.start()
                        except Exception as e:
                            print(f"Failed to start DropatLoc thread: {e}")
                            drop_running = False
                            stop_drop_event = None
                    else:
                        print("Stopping DropatLoc")
                        try:
                            if stop_drop_event:
                                stop_drop_event.set()
                            if drop_thread and drop_thread.is_alive():
                                drop_thread.join()
                        except Exception as e:
                            print(f"Error stopping DropatLoc thread: {e}")
                        finally:
                            stop_drop_event = None
                            drop_thread = None
                            drop_running = False
                elif i == 1:  # B
                    reconfigure_gripper(robot)
                elif i == 2:  # X
                    gripper_open = toggleGripper(robot, gripper_open)
                elif i == 3:  # Y
                    robot.MoveJ(HOME, 1, 0, vel = JOG_SPEED, acc = JOG_ACCL)
                elif i == 4:  # LB
                    toggleDiffBalls(robot)
                elif i == 5:  # RB
                    togglePick(robot)
                elif i == 6:
                    ResetErrors(robot)
                elif i == 7:
                    err, pose = robot.GetActualJointPosDegree()
                    writetocsv(pose)
            button_states[i] = buttons[i]

        time.sleep(0.005)  # Poll frequently for responsiveness


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

        if motion_thread and motion_thread.is_alive():
            motion_stop_event.set()
            motion_thread.join()

        if pick_thread and pick_thread.is_alive():
            pick_running = False
            pick_thread.join()

        pygame.display.quit()
        pygame.quit()

def main():
    print("Starting Controller Script")
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

    MIDPOINT_UP   = CalculateZoffset(robot, MIDPOINT, up=1)
    POINT1_UP   = CalculateZoffset(robot, POINT1, up=1)
    POINT2_UP   = CalculateZoffset(robot, POINT2, up=1)
    

    robot.SetGripperConfig(4, 0)
    robot.ActGripper(1, 0)
    robot.ActGripper(1, 1)
    global gripper_open
    if not gripper_open:
        gripper_open = toggleGripper(robot, gripper_open)
        
    robot.SetSpeed(robot_speed)
    robot.SetOaccScale(100)

    err, Startpose = robot.GetActualTCPPose()
    print(f"Starting at position {Startpose}, with error code {err}")

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
