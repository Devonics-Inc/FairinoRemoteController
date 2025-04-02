import sys
sys.path.insert(0, '/Users/FrTest/Desktop/FrController')
from fairino import Robot
import time
# Establish a connection with the robot controller and return a robot object if the connection is successful
robot = Robot.RPC('192.168.57.2')
'''
error,joint_pos = robot.GetActualJointPosDegree()
print("Current joint position of the robot",joint_pos)
joint_pos = [joint_pos[0],joint_pos[1],joint_pos[2],joint_pos[3],joint_pos[4],joint_pos[5]]
error_joint = 0
count =100
error = robot.ServoMoveStart() #ServoMoveStart
print("Servo motion start error code",error)
while(count):
    error = robot.ServoJ(joint_pos=joint_pos,axisPos=[0,0,0,0]) #joint space servo mode motion
    if error!=0:
        error_joint =error
    joint_pos[0] = joint_pos[0] + 0.1 # 1-axis movement 0.1 degree each time, 100 movements
    count = count - 1
    time.sleep(0.008)
print("Joint space servo mode motion error code",error_joint)
error = robot.ServoMoveEnd() # servo move end
print("Servo motion end error code",error)'''
mode = 2 #[0]-absolute motion (base coordinate system), [1]-incremental motion (base coordinate system), [2]-incremental motion (tool coordinate system)
n_pos = [0.0,0.0,-0.5,0.0,0.0,0.0] #Cartesian space bit-position increments
desc_pos = robot.GetActualTCPPose()
print("Current Cartesian position of the robot",desc_pos)

error_cart =0
error = robot.ServoMoveStart() #ServoMoveStart
print("Servo motion start error code",error)
for i in range(10):
    num = input("Enter ")
    print(num == 1)
    if num:
        error = robot.ServoCart(mode, n_pos, vel=100, acc=100) #Cartesian space servo mode motion
        #time.sleep(0.008)
        print("Running servo")
    
desc_pos = robot.GetActualTCPPose()
print("Final Cartesian position of the robot",desc_pos)
print("Cartesian space servo mode motion error code", error_cart)
error = robot.ServoMoveEnd() # servo move end
print("Servo motion end error code",error)
