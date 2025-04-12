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
POINT1 = [-34.9,-72.7,90.17,-102.44,-92.39,-39.95]
POINT2 = [13.6,-72.7,90.94,-104.11,-94.17,0.69]
HOME = [-9.54,-80.59,74.67,-81.7,-92.29,-10.86]

MIDPOINT_UP = []
POINT1_UP = []
POINT2_UP = []

TARGET1 = POINT1
TARGET2 = POINT2


JOG_ACCL = 100
JOG_SPEED = 100
incr = 1.0
THRESHOLD = 0.7

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

def getJoystickCommand(axes):
    #
    # print(f"Axes being detected: {axes}")
    if axes[0] > THRESHOLD: return L_RIGHT
    if axes[0] < -THRESHOLD: return L_LEFT
    if axes[1] > THRESHOLD: return L_DOWN
    if axes[1] < -THRESHOLD: return L_UP
    if axes[2] > THRESHOLD: return R_RIGHT
    if axes[2] < -THRESHOLD: return R_LEFT
    if axes[3] > THRESHOLD: return R_UP
    if axes[3] < -THRESHOLD: return R_DOWN
    return None

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
    
    print(tcp_new)
    err, x = robot.GetInverseKin(type=0, desc_pos=tcp_new, config=-1)
    print(err, x, tcp_new)
    return(x)

def Pick(robot):
    global POINT1, POINT2, MIDPOINT, POINT1_UP, POINT2_UP, MIDPOINT_UP, gripper_open

    robot.MoveJ(POINT1_UP, 0, 0)
    robot.MoveJ(POINT1, 0, 0)
    gripper_open = toggleGripper(robot, gripper_open)
    gripper_open = toggleGripper(robot, gripper_open)
    robot.MoveJ(POINT1_UP, 0, 0)
    robot.MoveJ(POINT2_UP, 0, 0)
    robot.MoveJ(POINT2, 0, 0)
    gripper_open = toggleGripper(robot, gripper_open)
    gripper_open = toggleGripper(robot, gripper_open)
    robot.MoveJ(POINT2_UP, 0, 0)

def runPickLoop(robot, event):
    global pick_running, gripper_open
    
    while not event.is_set():
        Pick(robot)
        time.sleep(0.5)


def togglePick(robot):
    global pick_running, pick_thread, gripper_open, stop_pick

    pick_running = not pick_running

    if pick_running:
        print("Going MIDPOINT")
        if not gripper_open:
            gripper_open = toggleGripper(robot, gripper_open)
            time.sleep(.5)
        GoTO(robot, MIDPOINT)
        print("Starting Pick loop")
        stop_pick = Event()
        pick_thread = Thread(target=runPickLoop, args=(robot, stop_pick,))
        pick_thread.start()
    else:
        print("Stopping Pick loop")
        pick_running = False
        if stop_pick:
            stop_pick.set()
        if pick_thread and pick_thread.is_alive():
            pick_thread.join()

def GoTO(robot, P1):
    global gripper_open
    P1_UP = CalculateZoffset(robot, P1, up=1)

    robot.MoveJ(P1_UP, 0, 0)
    robot.MoveJ(P1, 0, 0)
    gripper_open = toggleGripper(robot, gripper_open)
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
    place_seq = ['P', 'G', 'S', 'H']
    pick_seq = [1, 2, 3, 4]
    for index in range(4):
        if stop_db_event.is_set():  # Check if stop was requested
            print("PickDiffBalls interrupted!")
            break
        GoTO(robot, pick_dict[pick_seq[index]])
        robot.MoveJ(HOME, 0, 0)
        time.sleep(2)
        val = runLuaScript(robot)
        ball = Assess(val)
        print(ball)
        GoTO(robot, place_dict[ball])
        robot.MoveJ(HOME, 0, 0)


def runDiffBallsLoop(robot):
    global db_running
    PickDiffBalls(robot)
    db_running = False  # Reset state after one run


def toggleDiffBalls(robot):
    global db_running, db_thread, stop_db_event

    if not db_running:
        print("Starting All Diff Balls loop")
        stop_db_event = Event()
        db_running = True
        db_thread = Thread(target=runDiffBallsLoop, args=(robot,))
        db_thread.start()
    else:
        print("Stopping All Diff Balls loop")
        db_running = False
        if stop_db_event:
            stop_db_event.set()
        if db_thread and db_thread.is_alive():
            db_thread.join()



def motion_execution(robot, joy, initial_cmd, stop_event):
    print("In motion execution")
    if not initial_cmd: return


    while not stop_event.is_set():
        # Poll joystick axes continuously
        # axes = [truncate(joy.get_axis(i)) for i in range(joy.get_numaxes())]
        # cmd = getJoystickCommand(axes)
        # if cmd != initial_cmd:
        #     print(f"Stopping motion due to change or release: {cmd}")
        
        delta = getServoCommand(initial_cmd)
        robot.ServoMoveStart()
        print("Servo command started")

        currentTCP = robot.GetActualTCPPose()[1]
        futureTCP = [currentTCP[i] + delta[i] for i in range(6)]
        futureJ = robot.GetInverseKin(type=0, desc_pos=futureTCP, config=-1)[1]

        if checkJointLimits(futureJ):
            robot.ServoCart(2, delta, vel=JOG_SPEED, acc=JOG_ACCL)
        else:
            break

        time.sleep(0.008)

    print("Ending Servo movement")
    robot.ServoMoveEnd()


def joystick_input_thread(robot, joy):
    print("Accepting JoyStick Input")
    global joystick_cmd, motion_thread, motion_stop_event, gripper_open
    last_cmd = None
    button_states = [False] * 10

    while not stop_flag.is_set():
        #print("..")
        axes = [truncate(joy.get_axis(i)) for i in range(joy.get_numaxes())]
        buttons = [joy.get_button(i) for i in range(10)]

        cmd = getJoystickCommand(axes)
        # with cmd_lock:
        #     joystick_cmd = cmd

        # Start/stop motion thread for axis input
        #print(cmd)
        # Always stop motion if no command is present
        # Start/stop motion thread for axis input
        if cmd != last_cmd:
            print("command != last command")
            if motion_thread and motion_thread.is_alive():
                print("Stopping previous motion and waiting for thread to join")
                motion_stop_event.set()
                motion_thread.join()
            if cmd:
                print("New thread")
                motion_stop_event = Event()
                motion_thread = Thread(target=motion_execution, args=(robot, joy, cmd, motion_stop_event))
                motion_thread.start()
            last_cmd = cmd



        # Button actions (one-shot)
        for i in range(len(buttons)):
            if buttons[i] and not button_states[i]:
                print(i)
                if i == 0:  # A
                    gripper_open = toggleGripper(robot, gripper_open)
                elif i == 1:  # B
                    togglePick(robot)
                elif i == 2:  # X
                    toggleDiffBalls(robot)
                elif i == 4:  # LB
                    closeGripper(robot)
                elif i == 5:  # RB
                    openGripper(robot)
                elif i == 8:
                    ResetErrors(robot)
                elif i ==9:
                    err, Jpose = robot.GetActualJointPosDegree()
                    writetocsv(Jpose)
            button_states[i] = buttons[i]

        time.sleep(0.02)

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
