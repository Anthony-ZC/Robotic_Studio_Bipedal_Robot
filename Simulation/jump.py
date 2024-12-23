import pybullet as p
import time
import pybullet_data
from math import sin
import numpy as np

# set up the simulation environment
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.81)

# load the robot and the ground
groundId = p.loadURDF("plane.urdf")

# load the robot
robotStartPos = [0,0,0.3]
robotStartOrientation = p.getQuaternionFromEuler([0,0,0])
robotId = p.loadURDF("model\myrobot_series.urdf",robotStartPos, robotStartOrientation,useFixedBase=False)
mode = p.POSITION_CONTROL

# set the initial position of the robot
offset = 0
p2 = [-15, 10, 10, -15]
p3 = [0, 0, -10, 0]
p5 = [10, 10, -15, 10]
p6 = [0, -10, 0, 0]

# pause for 5 seconds
time.sleep(5)

# run the simulation
for i in range (10000):
    # stand up phase
    if i <= 200:
        q = [0, 0, 0, 0]
    else:
        # jump phase
        t = i-100
        q = [20*np.pi/180 * sin(t/10), -40*np.pi/180 * sin(t/10), 20*np.pi/180 * sin(t/10), -40*np.pi/180 * sin(t/10)]
    p.setJointMotorControl2(robotId, 2, controlMode=mode, targetPosition=q[0])
    p.setJointMotorControl2(robotId, 3, controlMode=mode, targetPosition=q[1])
    p.setJointMotorControl2(robotId, 6, controlMode=mode, targetPosition=q[2])
    p.setJointMotorControl2(robotId, 7, controlMode=mode, targetPosition=q[3])

    p.setJointMotorControl2(robotId, 4, controlMode=mode, targetPosition=-q[0]-q[1])
    p.setJointMotorControl2(robotId, 8, controlMode=mode, targetPosition=-q[2]-q[3])

    p.setJointMotorControl2(robotId, 1, controlMode=mode, targetPosition=0)
    p.setJointMotorControl2(robotId, 5, controlMode=mode, targetPosition=0)

    p.stepSimulation()
    time.sleep(1./240.)
robotPos, robotOrn = p.getBasePositionAndOrientation(robotId)
print(robotPos, robotOrn)
p.disconnect()