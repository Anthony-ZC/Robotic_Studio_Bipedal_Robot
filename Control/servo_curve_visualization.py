import numpy as np
import matplotlib.pyplot as plt
from math import ceil


def cubic_polynomial(ps,ts,extra_velocity_constraint,extra_acceleration_constraint,constraint_type = 'end'):
    """
    Calculate the cubic polynomial coefficients for the given waypoints and time stamps and constraints.

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

    for i in range(n):
        j = i
        A[i*2,j*4] = A[i*2,j*4+1] = A[i*2,j*4+2] = 0
        A[i*2,j*4+3] = 1

        A[i*2+1,j*4] = ts[j]**3
        A[i*2+1,j*4+1] = ts[j]**2
        A[i*2+1,j*4+2] = ts[j]
        A[i*2+1,j*4+3] = 1

    for i in range(2*n,3*n-1):
        j = i -2*n
        A[i,j*4] = 3 * ts[j]**2
        A[i,j*4+1] = 2 * ts[j]
        A[i,j*4+2] = 1
        A[i,j*4+6] = -1

    for i in range(3*n,4*n-1):
        j = i -3*n
        A[i,j*4] = 6 * ts[j]
        A[i,j*4+1] = 2 
        A[i,j*4+5] = -2
    
    y =  np.zeros((n*4,1))
    y[0] = ps[0]
    y[2*n-1] = ps[-1]
    for i in range(len(ps)-2):
        y[2*i+1] = y[2*i+2] = ps[i+1]

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
    Calculate the cubic polynomial coefficients for the given waypoints and time stamps. Extra constraints are added to make the curve periodic (start and end velocity and acceleration are equal).

    Args:
        ps (list): list of waypoints
        ts (list): list of time stamps

    Returns:
        cubic polynomial coefficients (numpy.ndarray): cubic polynomial coefficients
    """
    assert len(ps) == len(ts)+1
    
    n = len(ps)-1
    A =  np.zeros((n*4,n*4))

    for i in range(n):
        j = i
        A[i*2,j*4] = A[i*2,j*4+1] = A[i*2,j*4+2] = 0
        A[i*2,j*4+3] = 1

        A[i*2+1,j*4] = ts[j]**3
        A[i*2+1,j*4+1] = ts[j]**2
        A[i*2+1,j*4+2] = ts[j]
        A[i*2+1,j*4+3] = 1

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

    for i in range(3*n,4*n-1):
        j = i -3*n
        A[i,j*4] = 6 * ts[j]
        A[i,j*4+1] = 2 
        A[i,j*4+5] = -2

    A[4*n-1,1] = -2
    A[4*n-1,4*n-4] = 6 * ts[n-1]
    A[4*n-1,4*n-3] = 2
    
    y =  np.zeros((n*4,1))
    y[0] = ps[0]
    y[2*n-1] = ps[-1]
    for i in range(len(ps)-2):
        y[2*i+1] = y[2*i+2] = ps[i+1]
        
    return np.linalg.inv(A) @ y  

# 1: servo1, 2: servo2, 3: servo3, 4: servo4, 5: servo5, 6: servo6, 7: servo7
curve_properties = [{}, {}, {}, {}, {}, {}, {}, {}, {}, {}]
q = [[], [], [], [], [], [], [], [], [], []]

# Walk parameters
walk_period = 0.8
Ts = walk_period/4.
walk_wait_time = 0
# time to stop
stop_time = 3

# Stand up pose
pose_stand_up = [112.32, 125, 105.80, 116.88, 108.00, 143.28, 100]

# Walk pose
walk_pose = {'left_font': {2:130.56, 3:96.96},
                    'left_back': {2:111.60, 3:116.88},
                    'left_trainsition': {2:111.60, 3:96.96},
                    'right_font': {5:97.92, 6:153.84},
                    'right_back': {5:119.28, 6:132.24},
                    'right_trainsition': {5:116.40, 6:153.36}}

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

# In stop, the start velocity and acceleration are constrained
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
curve_properties[1] = {'cubic_walk': c2_walk, 'time_stamp': np.cumsum(tl_walk),
                                'cubic_prepare': c2_prepare, 'time_stamp_prepare': np.cumsum(tl_prepare),
                                'cubic_stop': c2_stop, 'time_stamp_stop': np.cumsum(tl_stop)}
curve_properties[2] = {'cubic_walk': c3_walk, 'time_stamp': np.cumsum(tl_walk),
                                'cubic_prepare': c3_prepare, 'time_stamp_prepare': np.cumsum(tl_prepare),
                                'cubic_stop': c3_stop, 'time_stamp_stop': np.cumsum(tl_stop)}
curve_properties[4] = {'cubic_walk': c5_walk, 'time_stamp': np.cumsum(tr_walk),
                                'cubic_prepare': c5_prepare, 'time_stamp_prepare': np.cumsum(tr_prepare),
                                'cubic_stop': c5_stop, 'time_stamp_stop': np.cumsum(tr_stop)}
curve_properties[5] = {'cubic_walk': c6_walk, 'time_stamp': np.cumsum(tr_walk),
                            'cubic_prepare': c6_prepare, 'time_stamp_prepare': np.cumsum(tr_prepare),
                            'cubic_stop': c6_stop, 'time_stamp_stop': np.cumsum(tr_stop)}

# Function to calculate the servo angles
def f(timer2):
    # Prepare phase
    if timer2 < walk_prepare_time:
        t = timer2
        for i in range(7):
            if curve_properties[i] == {}:
                continue
            if t <= curve_properties[i]['time_stamp_prepare'][0]:
                circle_t = t
                q_i = curve_properties[i]['cubic_prepare'][0] * circle_t**3 + curve_properties[i]['cubic_prepare'][1] * circle_t**2 + curve_properties[i]['cubic_prepare'][2] * circle_t + curve_properties[i]['cubic_prepare'][3]
            elif t <= curve_properties[i]['time_stamp_prepare'][1]:
                circle_t = t - curve_properties[i]['time_stamp_prepare'][0]
                q_i = curve_properties[i]['cubic_prepare'][4] * circle_t**3 + curve_properties[i]['cubic_prepare'][5] * circle_t**2 + curve_properties[i]['cubic_prepare'][6] * circle_t + curve_properties[i]['cubic_prepare'][7]
            q[i].append(q_i[0])

        return
    
    t = (timer2 - walk_prepare_time) % walk_period
    # Stop phase
    if timer2>stop_time:
        global walk_wait_time
        if walk_wait_time == 0:
            walk_wait_time = ceil((timer2 - walk_prepare_time)/walk_period) * walk_period

        if timer2 > walk_prepare_time + walk_wait_time and timer2 < walk_prepare_time + walk_wait_time + walk_stop_time:
            t = timer2 - walk_prepare_time - walk_wait_time  - 1e-5
            for i in range(7):
                if curve_properties[i] == {}:
                    continue
                if t <= curve_properties[i]['time_stamp_stop'][0]:
                    circle_t = t
                    q_i = curve_properties[i]['cubic_stop'][0] * circle_t**3 + curve_properties[i]['cubic_stop'][1] * circle_t**2 + curve_properties[i]['cubic_stop'][2] * circle_t + curve_properties[i]['cubic_stop'][3]
                elif t <= curve_properties[i]['time_stamp_stop'][1]:
                    circle_t = t - curve_properties[i]['time_stamp_stop'][0]
                    q_i = curve_properties[i]['cubic_stop'][4] * circle_t**3 + curve_properties[i]['cubic_stop'][5] * circle_t**2 + curve_properties[i]['cubic_stop'][6] * circle_t + curve_properties[i]['cubic_stop'][7]
                q[i].append(q_i[0])
            return
        elif timer2 >= walk_prepare_time + walk_wait_time + walk_stop_time:
            t = walk_stop_time
            for i in range(7):
                if curve_properties[i] == {}:
                    continue
                if t <= curve_properties[i]['time_stamp_stop'][0]:
                    circle_t = t
                    q_i = curve_properties[i]['cubic_stop'][0] * circle_t**3 + curve_properties[i]['cubic_stop'][1] * circle_t**2 + curve_properties[i]['cubic_stop'][2] * circle_t + curve_properties[i]['cubic_stop'][3]
                elif t <= curve_properties[i]['time_stamp_stop'][1]:
                    circle_t = t - curve_properties[i]['time_stamp_stop'][0]
                    q_i = curve_properties[i]['cubic_stop'][4] * circle_t**3 + curve_properties[i]['cubic_stop'][5] * circle_t**2 + curve_properties[i]['cubic_stop'][6] * circle_t + curve_properties[i]['cubic_stop'][7]
                q[i].append(q_i[0])
            return
        
    # Periodic walking phase
    for i in range(7):
        if curve_properties[i] == {}:
            continue
        if t <= curve_properties[i]['time_stamp'][0]:
            circle_t = t
            q_i = curve_properties[i]['cubic_walk'][0] * circle_t**3 + curve_properties[i]['cubic_walk'][1] * circle_t**2 + curve_properties[i]['cubic_walk'][2] * circle_t + curve_properties[i]['cubic_walk'][3]
        elif t <= curve_properties[i]['time_stamp'][1]:
            circle_t = t - curve_properties[i]['time_stamp'][0]
            q_i = curve_properties[i]['cubic_walk'][4] * circle_t**3 + curve_properties[i]['cubic_walk'][5] * circle_t**2 + curve_properties[i]['cubic_walk'][6] * circle_t + curve_properties[i]['cubic_walk'][7]
        elif t <= curve_properties[i]['time_stamp'][2]:
            circle_t = t - curve_properties[i]['time_stamp'][1]
            q_i = curve_properties[i]['cubic_walk'][8] * circle_t**3 + curve_properties[i]['cubic_walk'][9] * circle_t**2 + curve_properties[i]['cubic_walk'][10] * circle_t + curve_properties[i]['cubic_walk'][11]
        q[i].append(q_i[0])

# Calculate the walking period
w = ceil((stop_time - walk_prepare_time)/walk_period) * walk_period

# Calculate the servo angles
tg = np.arange(0,5,0.01)
for a in tg:
    f(a)
# Plot the servo angles
fig, axs = plt.subplots(4, 1, figsize=(8, 12), sharex=True)

# Plot the servo angles of servo2, servo3, servo5, servo6
axs[0].plot(tg, q[1], color='blue', label='servo2')
axs[0].set_ylabel('servo angles (degrees)')
axs[0].set_ylim(85, 185) 
axs[0].legend()
axs[0].grid()
# Plot the vertical lines for the prepare phase, walking period and stop phase
axs[0].vlines(x=[walk_prepare_time,stop_time,w + walk_prepare_time, w + walk_prepare_time + walk_stop_time],  ymin=90, ymax=180, colors=['y', 'g', 'b','r'], linestyles='--')
# Plot the horizontal lines for the key waypoints
for c in [*p2_prepare,*p2_walk]:
    axs[0].axhline(y=c, color='r', linestyle='--')

axs[1].plot(tg, q[2], color='green', label='servo3')
axs[1].set_ylabel('servo angles (degrees)')
axs[1].set_ylim(85, 185)
axs[1].legend()
axs[1].grid()
axs[1].vlines(x=[walk_prepare_time,stop_time,w + walk_prepare_time, w + walk_prepare_time + walk_stop_time],  ymin=90, ymax=180, colors=['y', 'g', 'b','r'], linestyles='--')
for c in [*p3_prepare,*p3_walk]:
    axs[1].axhline(y=c, color='r', linestyle='--')

axs[2].plot(tg, q[4], color='red', label='servo5')
axs[2].set_ylabel('servo angles (degrees)')
axs[2].set_ylim(85, 185)
axs[2].legend()
axs[2].grid()
axs[2].vlines(x=[walk_prepare_time,stop_time,w + walk_prepare_time, w + walk_prepare_time + walk_stop_time],  ymin=90, ymax=180, colors=['y', 'g', 'b','r'], linestyles='--')

for c in [*p5_prepare,*p5_walk]:
    axs[2].axhline(y=c, color='r', linestyle='--')

axs[3].plot(tg, q[5], color='purple', label='servo6')
axs[3].set_ylabel('servo angles (degrees)')
axs[3].set_ylim(85, 185)
axs[3].set_xlabel('Time (s)')
axs[3].legend()
axs[3].grid()
axs[3].vlines(x=[walk_prepare_time,stop_time,w + walk_prepare_time, w + walk_prepare_time + walk_stop_time],  ymin=90, ymax=180, colors=['r', 'g', 'b'], linestyles='--')
for c in [*p6_prepare,*p6_walk]:
    axs[3].axhline(y=c, color='r', linestyle='--')
    
plt.tight_layout()
plt.show()