import pybullet as p
import time
import pybullet_data
from math import sin
import numpy as np

def cubic_polynomial(ps,ts,extra_velocity_constraint,extra_acceleration_constraint,constraint_type = 'end'):
    """
    Calculate the cubic polynomial coefficients for the given waypoints and time stamps and constraints, which ensures the continuity of the velocity and acceleration at the waypoints.
    Extra zero constraints are added to the velocity and acceleration at the end or start of the waypoints.
    Solution is obtained by solving a linear system of equations.
    
    Args:
        ps (list): list of waypoints
        ts (list): list of time stamps
        extra_velocity_constraint (float): extra velocity constraint
        extra_acceleration_constraint (float): extra acceleration constraint
        constraint_type (str): constraint type, 'end' or 'start', for 'start' constraint, the first velocity and acceleration are constrained, for 'end' constraint, the last velocity and acceleration are constrained.

    Returns:
        cubic polynomial coefficients (numpy.ndarray): cubic polynomial coefficients
    """
    assert len(ps) == len(ts)+1
    
    n = len(ps)-1
    A =  np.zeros((n*4,n*4))
    
    # Position constraints
    for i in range(n):
        j = i
        A[i*2,j*4] = A[i*2,j*4+1] = A[i*2,j*4+2] = 0
        A[i*2,j*4+3] = 1

        A[i*2+1,j*4] = ts[j]**3
        A[i*2+1,j*4+1] = ts[j]**2
        A[i*2+1,j*4+2] = ts[j]
        A[i*2+1,j*4+3] = 1
    # Velocity constraints
    for i in range(2*n,3*n-1):
        j = i -2*n
        A[i,j*4] = 3 * ts[j]**2
        A[i,j*4+1] = 2 * ts[j]
        A[i,j*4+2] = 1
        A[i,j*4+6] = -1
    # Acceleration constraints
    for i in range(3*n,4*n-1):
        j = i -3*n
        A[i,j*4] = 6 * ts[j]
        A[i,j*4+1] = 2 
        A[i,j*4+5] = -2
    
    # Waypoints' position
    y =  np.zeros((n*4,1))
    y[0] = ps[0]
    y[2*n-1] = ps[-1]
    for i in range(len(ps)-2):
        y[2*i+1] = y[2*i+2] = ps[i+1]
        
    # Extra constraints
    if constraint_type == 'end':
        A[3*n-1,4*n-4] = 3 * ts[n-1]**2
        A[3*n-1,4*n-3] = 2 * ts[n-1]
        A[3*n-1,4*n-2] = 1
        A[4*n-1,4*n-4] = 6 * ts[n-1]
        A[4*n-1,4*n-3] = 2
    elif constraint_type == 'start':
        A[3*n-1,2] = 1
        A[4*n-1,1] = 2

    y[3*n-1] = extra_velocity_constraint
    y[4*n-1] = extra_acceleration_constraint

    return np.linalg.inv(A) @ y


def cubic_polynomial_periodic(ps,ts):
    """
    Calculate the cubic polynomial coefficients for the given waypoints and time stamps, which ensures the continuity of the velocity and acceleration at the waypoints.
    Boundary conditions are periodic, which means the velocity and acceleration at the end and start of the waypoints are equal.
    Solution is obtained by solving a linear system of equations.

    Args:
        ps (list): list of waypoints
        ts (list): list of time stamps

    Returns:
        cubic polynomial coefficients (numpy.ndarray): cubic polynomial coefficients
    """
    assert len(ps) == len(ts)+1
    
    n = len(ps)-1
    A =  np.zeros((n*4,n*4))

    # Position constraints
    for i in range(n):
        j = i
        A[i*2,j*4] = A[i*2,j*4+1] = A[i*2,j*4+2] = 0
        A[i*2,j*4+3] = 1

        A[i*2+1,j*4] = ts[j]**3
        A[i*2+1,j*4+1] = ts[j]**2
        A[i*2+1,j*4+2] = ts[j]
        A[i*2+1,j*4+3] = 1

    # Velocity constraints
    for i in range(2*n,3*n-1):
        j = i -2*n
        A[i,j*4] = 3 * ts[j]**2
        A[i,j*4+1] = 2 * ts[j]
        A[i,j*4+2] = 1
        A[i,j*4+6] = -1

    A[3*n-1,2] = -1
    A[3*n-1,4*n-4] = 3 * ts[n-1]**2
    A[3*n-1,4*n-3] = 2 * ts[n-1]
    A[3*n-1,4*n-2] = 1

    # Acceleration constraints
    for i in range(3*n,4*n-1):
        j = i -3*n
        A[i,j*4] = 6 * ts[j]
        A[i,j*4+1] = 2 
        A[i,j*4+5] = -2

    A[4*n-1,1] = -2
    A[4*n-1,4*n-4] = 6 * ts[n-1]
    A[4*n-1,4*n-3] = 2
    
    # Waypoints' position
    y =  np.zeros((n*4,1))
    y[0] = ps[0]
    y[2*n-1] = ps[-1]
    for i in range(len(ps)-2):
        y[2*i+1] = y[2*i+2] = ps[i+1]
        
    return np.linalg.inv(A) @ y

# set the initial position of the robot
offset =  13

# Walk parameters
walk_period = 40
Ts = walk_period/4.
walk_wait_time = 200

# time to stop (unused here)
stop_time = 3

# 1: servo1, 2: servo2, 3: servo3, 4: servo4, 5: servo5, 6: servo6, 7: servo7
curve_properties = [{}, {}, {}, {}, {}, {}, {}, {}, {}, {}]

# Stand up pose
pose_stand_up = [0, offset, -offset, 0, offset, -offset, 0]  

# Walk pose
walk_pose = {'left_font': {2:-10+offset, 3:0-offset},
                    'left_back': {2:10+offset, 3:0-offset},
                    'left_trainsition': {2:10+offset, 3:-10-offset},
                    'right_font': {5:-10+offset, 6:0-offset},
                    'right_back': {5:10+offset, 6:0-offset},
                    'right_trainsition': {5:10+offset, 6:-10-offset}}

# Cubic polynomial for walking
p2_walk = [walk_pose['left_font'][2], walk_pose['left_back'][2], walk_pose['left_trainsition'][2], walk_pose['left_font'][2]]
p3_walk = [walk_pose['left_font'][3], walk_pose['left_back'][3], walk_pose['left_trainsition'][3], walk_pose['left_font'][3]]
tl_walk = [2*Ts , Ts, Ts]

c2_walk = cubic_polynomial_periodic(p2_walk,tl_walk)
c3_walk = cubic_polynomial_periodic(p3_walk,tl_walk)

p5_walk = [walk_pose['right_back'][5], walk_pose['right_trainsition'][5], walk_pose['right_font'][5], walk_pose['right_back'][5]]
p6_walk = [walk_pose['right_back'][6], walk_pose['right_trainsition'][6], walk_pose['right_font'][6], walk_pose['right_back'][6]]
tr_walk = [Ts , Ts, 2*Ts]
c5_walk = cubic_polynomial_periodic(p5_walk,tr_walk)
c6_walk = cubic_polynomial_periodic(p6_walk,tr_walk)

# Calculate the start velocity and acceleration
c2_start_velocity = c2_walk[2]
c2_start_acceleration = 2 * c2_walk[1]
c3_start_velocity = c3_walk[2]
c3_start_acceleration = 2 * c3_walk[1]
c5_start_velocity = c5_walk[2]
c5_start_acceleration = 2 * c5_walk[1]
c6_start_velocity = c6_walk[2]
c6_start_acceleration = 2 * c6_walk[1]

# In preparation, the end velocity and acceleration are constrained
p2_prepare = [pose_stand_up[2-1], walk_pose['left_trainsition'][2], walk_pose['left_font'][2]]
p3_prepare = [pose_stand_up[3-1], walk_pose['left_trainsition'][3], walk_pose['left_font'][3]]
tl_prepare = [1 * Ts, 1 * Ts]

p5_prepare = [pose_stand_up[5-1],walk_pose['right_back'][5]]
p6_prepare = [pose_stand_up[6-1],walk_pose['right_back'][6]]
tr_prepare = [2 * Ts]
walk_prepare_time = sum(tl_prepare)

c2_prepare = cubic_polynomial(p2_prepare,tl_prepare,c2_start_velocity,c2_start_acceleration,constraint_type='end')
c3_prepare = cubic_polynomial(p3_prepare,tl_prepare,c3_start_velocity,c3_start_acceleration,constraint_type='end')
c5_prepare = cubic_polynomial(p5_prepare,tr_prepare,c5_start_velocity,c5_start_acceleration,constraint_type='end')
c6_prepare = cubic_polynomial(p6_prepare,tr_prepare,c6_start_velocity,c6_start_acceleration,constraint_type='end')

# In stop, the start velocity and acceleration are constrained (unused here)
p2_stop = [walk_pose['left_font'][2],pose_stand_up[2-1]]
p3_stop = [walk_pose['left_font'][3],pose_stand_up[3-1]]
tl_stop = [2 * Ts]

p5_stop = [walk_pose['right_back'][5],walk_pose['right_trainsition'][5],pose_stand_up[5-1]]
p6_stop = [walk_pose['right_back'][6],walk_pose['right_trainsition'][6],pose_stand_up[6-1]]
tr_stop = [1 * Ts, 1 * Ts]
walk_stop_time = sum(tr_stop)

c2_stop = cubic_polynomial(p2_stop,tl_stop,c2_start_velocity,c2_start_acceleration,constraint_type='start')
c3_stop = cubic_polynomial(p3_stop,tl_stop,c3_start_velocity,c3_start_acceleration,constraint_type='start')
c5_stop = cubic_polynomial(p5_stop,tr_stop,c5_start_velocity,c5_start_acceleration,constraint_type='start')
c6_stop = cubic_polynomial(p6_stop,tr_stop,c6_start_velocity,c6_start_acceleration,constraint_type='start')

# Store the cubic polynomial coefficients
curve_properties[0] = {'cubic_walk': c2_walk, 'time_stamp': np.cumsum(tl_walk),
                                'cubic_prepare': c2_prepare, 'time_stamp_prepare': np.cumsum(tl_prepare),
                                'cubic_stop': c2_stop, 'time_stamp_stop': np.cumsum(tl_stop)}
curve_properties[1] = {'cubic_walk': c3_walk, 'time_stamp': np.cumsum(tl_walk),
                                'cubic_prepare': c3_prepare, 'time_stamp_prepare': np.cumsum(tl_prepare),
                                'cubic_stop': c3_stop, 'time_stamp_stop': np.cumsum(tl_stop)}
curve_properties[2] = {'cubic_walk': c5_walk, 'time_stamp': np.cumsum(tr_walk),
                                'cubic_prepare': c5_prepare, 'time_stamp_prepare': np.cumsum(tr_prepare),
                                'cubic_stop': c5_stop, 'time_stamp_stop': np.cumsum(tr_stop)}
curve_properties[3] = {'cubic_walk': c6_walk, 'time_stamp': np.cumsum(tr_walk),
                            'cubic_prepare': c6_prepare, 'time_stamp_prepare': np.cumsum(tr_prepare),
                            'cubic_stop': c6_stop, 'time_stamp_stop': np.cumsum(tr_stop)}

q = [[], [], [], []]

# set up the simulation environment
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.81)

# load the robot and the ground
groundId = p.loadURDF("plane.urdf")

# load the robot
robotStartPos = [0,0,0.25]
robotStartOrientation = p.getQuaternionFromEuler([0,0,0])
robotId = p.loadURDF("model\myrobot_series.urdf",robotStartPos, robotStartOrientation,useFixedBase=False)
mode = p.POSITION_CONTROL

v = []



for i in range (10000):
    q=[]
    # Stand up phase
    if i <= walk_wait_time:
        q = [offset*np.pi/180, -offset*np.pi/180, offset*np.pi/180, -offset*np.pi/180]

    # Prepare phase
    elif  i <  walk_prepare_time + walk_wait_time:
        t = i - walk_wait_time
        for k in range(4):
            if curve_properties[k] == {}:
                continue
            if t <= curve_properties[k]['time_stamp_prepare'][0]:
                circle_t = t
                q_i = curve_properties[k]['cubic_prepare'][0] * circle_t**3 + curve_properties[k]['cubic_prepare'][1] * circle_t**2 + curve_properties[k]['cubic_prepare'][2] * circle_t + curve_properties[k]['cubic_prepare'][3]
            elif t <= curve_properties[k]['time_stamp_prepare'][1]:
                circle_t = t - curve_properties[k]['time_stamp_prepare'][0]
                q_i = curve_properties[k]['cubic_prepare'][4] * circle_t**3 + curve_properties[k]['cubic_prepare'][5] * circle_t**2 + curve_properties[k]['cubic_prepare'][6] * circle_t + curve_properties[k]['cubic_prepare'][7]
            q.append(q_i*np.pi/180)
    else:
        # W
        t = (i - walk_prepare_time - walk_wait_time) % walk_period
        for k in range(4):
            if curve_properties[k] == {}:
                continue
            if t <= curve_properties[k]['time_stamp'][0]:
                circle_t = t
                q_i = curve_properties[k]['cubic_walk'][0] * circle_t**3 + curve_properties[k]['cubic_walk'][1] * circle_t**2 + curve_properties[k]['cubic_walk'][2] * circle_t + curve_properties[k]['cubic_walk'][3]
            elif t <= curve_properties[k]['time_stamp'][1]:
                circle_t = t - curve_properties[k]['time_stamp'][0]
                q_i = curve_properties[k]['cubic_walk'][4] * circle_t**3 + curve_properties[k]['cubic_walk'][5] * circle_t**2 + curve_properties[k]['cubic_walk'][6] * circle_t + curve_properties[k]['cubic_walk'][7]
            elif t <= curve_properties[k]['time_stamp'][2]:
                circle_t = t - curve_properties[k]['time_stamp'][1]
                q_i = curve_properties[k]['cubic_walk'][8] * circle_t**3 + curve_properties[k]['cubic_walk'][9] * circle_t**2 + curve_properties[k]['cubic_walk'][10] * circle_t + curve_properties[k]['cubic_walk'][11]
            q.append(q_i*np.pi/180)

    p.setJointMotorControl2(robotId, 2, controlMode=mode, targetPosition=q[0])
    p.setJointMotorControl2(robotId, 3, controlMode=mode, targetPosition=q[1])
    p.setJointMotorControl2(robotId, 6, controlMode=mode, targetPosition=q[2])
    p.setJointMotorControl2(robotId, 7, controlMode=mode, targetPosition=q[3])

    p.setJointMotorControl2(robotId, 4, controlMode=mode, targetPosition=-q[0]-q[1])
    p.setJointMotorControl2(robotId, 8, controlMode=mode, targetPosition=-q[2]-q[3])

    p.setJointMotorControl2(robotId, 1, controlMode=mode, targetPosition=0)
    p.setJointMotorControl2(robotId, 5, controlMode=mode, targetPosition=0)

    p.stepSimulation()

    # get the linear velocity of the robot
    linear_velocity, angular_velocity = p.getBaseVelocity(robotId)

    # calculate the average linear velocity every 200 steps
    if (i+1)%200 != 0:
        v.append(np.linalg.norm(linear_velocity[0])*100)
    else:
        print("Linear Velocity:", sum(v)/len(v), "cm/s")
        v = []

    time.sleep(1./240.)


robotPos, robotOrn = p.getBasePositionAndOrientation(robotId)
print(robotPos, robotOrn)
p.disconnect()