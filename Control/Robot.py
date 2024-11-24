from math import sin, cos, ceil
from pylx16a.lx16a import *
import time
import numpy as np
import threading
from copy import deepcopy

def cubic_polynomial(ps,ts,extra_velocity_constraint,extra_acceleration_constraint,constraint_type = 'end'):
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

class ServoError(Exception):
    def __init__(self, message):
        self.message = message
 
    def __str__(self):
        return self.message


class Robot:
    def __init__(self, servo_num):
        try:
            LX16A.initialize("/dev/ttyUSB0", 0.1)
        except:
            LX16A.initialize("/COM6", 0.1)

        self.servo_num = servo_num
        self.q = [0]*self.servo_num
        self.servo_status = [[False, False, False] for i in range(self.servo_num)]
        self.voltage_lower_limit = 4500
        self.voltage_upper_limit = 12000
        self.temperature_limit = 70
        self.angle_lower_limit = 0
        self.angle_upper_limit = 240

        self.robot_command = ''
        self.exiting = False

        self.leg_status = None
        self.head_status = None

        self.walk_prepare_time = 0
        self.walk_stop_time = 0
        self.walk_2_stop = False
        self.walk_wait_time = 0
        
        self.start_time = time.time()
        self.time_stamp = time.time()
        self.running_time = 0
        self.check_period = 10

        self.timer1 = 0
        self.timer2 = 0

        self.mian_proceed_time = 2
        self.main_step_time = 0.1
        self.main_sleep_time = 0.05

        self.pose_homing = [112.32, 94.48, 95.76, 116.64, 137.76, 151.68, 100]
        self.pose_stand_up = [112.32, 125, 105.80, 116.88, 108.00, 143.28, 100]
        self.curve_properties = [{} for i in range(self.servo_num)]

        self.initialize_servos()

    def listen_input(self):
        while True:
            self.robot_command = input() 
            print('Get command: {}'.format(self.robot_command))

    def initialize_servos(self):
        try:
            self.servos = [LX16A(i) for i in range(1,self.servo_num + 1)]
            for i in range(self.servo_num):
                self.servos[i].set_angle_limits(self.angle_lower_limit, self.angle_upper_limit)
                self.servos[i].set_vin_limits(self.voltage_lower_limit, self.voltage_upper_limit)
                self.servos[i].set_temp_limit(self.temperature_limit)
                self.servos[i].set_led_error_triggers(True, True, True)
                self.servos[i].led_power_on()
                self.servos[i].enable_torque()
        except ServoTimeoutError as e:
            print(f"Servo {e.id_} is not responding. Exiting...")
            quit()
        time.sleep(self.mian_proceed_time)

    def set_all_servos(self,move_time=0,skip=[]):
        assert self.servo_num == len(self.q)
        
        for i in range(len(self.q)):
            if i in skip:
                continue
            self.servos[i].move(self.q[i],move_time)
    
    def self_check(self):
        for i in range(self.servo_num):
            self.servo_status[i] = self.servos[i].get_led_error_triggers(True)
            if self.servo_status[i][0]:
                print(f'Servo {i+1} tempreture is too high!')
            if self.servo_status[i][1]:
                print(f'Servo {i+1} voltage is abnormal!')
            if self.servo_status[i][2]:
                print(f'Servo {i+1} is rocked!')

        if any(True in row for row in self.servo_status):
            raise ServoError('The robot is not working properly! Time: {:.2f}'.format(self.running_time))
        else:    
            print('The robot is working properly! Time: {:.2f}'.format(self.running_time))

    def homing(self,skip=[]):
        self.q = deepcopy(self.pose_homing)
        self.set_all_servos(self.mian_proceed_time * 1000,skip=skip)
        time.sleep(self.mian_proceed_time + 1)
        print('Successfully move to homing pose! Time: {:.2f}'.format(self.running_time))
    
    def stand_up(self,skip=[]):
        self.q = deepcopy(self.pose_stand_up)
        self.set_all_servos(self.mian_proceed_time * 1000,skip=skip)
        time.sleep(self.mian_proceed_time + 1)
        print('Successfully stand up! Time: {:.2f}'.format(self.running_time))

    def nod_head(self):
        self.servos[6].move(sin(self.timer1) * 15 + 100)

    def prepare_squat(self, squat_ratio = 0.6, squat_frequency = 2):
        old_difference = [self.pose_stand_up[i]-self.pose_homing[i] for i in range(self.servo_num)]
        down = [self.pose_stand_up[i]-old_difference[i] * squat_ratio for i in range(self.servo_num)]

        self.squating_frequency = squat_frequency
        self.squating_difference = [(self.pose_stand_up[i]-down[i])/2 for i in range(self.servo_num)]
        self.squating_middle = [(self.pose_stand_up[i]+down[i])/2 for i in range(self.servo_num)]

    def squat(self,skip=[]):
        self.q = [self.squating_difference[i]*cos(self.squating_frequency*self.timer2) + self.squating_middle[i] for i in range(self.servo_num)]
        self.set_all_servos(skip=skip)

    def prepare_step(self,step_period = 0.4):
        self.step_period = step_period
        self.step_ferq = 2 * pi / step_period
        pose_left_up = [112.32, 111.60, 96.96, 116.88, 108.00, 143.28, 100]
        pose_right_up = [112.32, 125, 105.80, 116.88, 116.40, 153.36, 100]

        self.step_middle_left = [(pose_left_up[i]+self.pose_stand_up[i])/2 for i in range(self.servo_num)]
        self.step_middle_right = [(pose_right_up[i]+self.pose_stand_up[i])/2 for i in range(self.servo_num)]

        self.step_difference_left = [(self.pose_stand_up[i]-pose_left_up[i])/2 for i in range(self.servo_num)]
        self.step_difference_right = [(self.pose_stand_up[i]-pose_right_up[i])/2 for i in range(self.servo_num)]

    def step(self,skip=[6]):
        self.q[1] = self.step_difference_left[1]*cos(self.step_ferq*self.timer2) + self.step_middle_left[1]
        self.q[2] = self.step_difference_left[2]*cos(self.step_ferq*self.timer2) + self.step_middle_left[2]
        if self.timer2 >= self.step_period/2:
            self.q[4] = self.step_difference_right[4]*cos(self.step_ferq*(self.timer2-self.step_period/2)) + self.step_middle_right[4]
            self.q[5] = self.step_difference_right[5]*cos(self.step_ferq*(self.timer2-self.step_period/2)) + self.step_middle_right[5]
        
        self.set_all_servos(skip=skip)

    def prepare_walk(self, walk_period = 0.8, skip=[6]):
        self.walk_period = walk_period

        Ts = walk_period/4.0
        p2_walk = [130.56, 111.60, 111.60, 130.56]
        p3_walk = [96.96, 116.88, 96.96, 96.96]
        tl_walk = [2*Ts , Ts, Ts]
        
        c2_walk = cubic_polynomial_periodic(p2_walk,tl_walk)
        c3_walk = cubic_polynomial_periodic(p3_walk,tl_walk)
        
        p5_walk = [119.28, 116.40, 97.92, 119.28]
        p6_walk = [132.24, 153.36, 153.84, 132.24]
        tr_walk = [Ts , Ts, 2*Ts]
        c5_walk = cubic_polynomial_periodic(p5_walk,tr_walk)
        c6_walk = cubic_polynomial_periodic(p6_walk,tr_walk)
  
        c2_start_velocity = c2_walk[2]
        c2_start_acceleration = 2 * c2_walk[1]
        c3_start_velocity = c3_walk[2]
        c3_start_acceleration = 2 * c3_walk[1]
        c5_start_velocity = c5_walk[2]
        c5_start_acceleration = 2 * c5_walk[1]
        c6_start_velocity = c6_walk[2]
        c6_start_acceleration = 2 * c6_walk[1]

        p2_prepare = [125, 111.60, 130.56]
        p3_prepare = [105.80, 96.96, 96.96]
        tl_prepare = [1 * Ts, 1 * Ts]

        p5_prepare = [108.00, 119.28]
        p6_prepare = [143.28, 132.24]
        tr_prepare = [2 * Ts]
        self.walk_prepare_time = sum(tl_prepare)
        
        c2_prepare = cubic_polynomial(p2_prepare,tl_prepare,c2_start_velocity,c2_start_acceleration,constraint_type='end')
        c3_prepare = cubic_polynomial(p3_prepare,tl_prepare,c3_start_velocity,c3_start_acceleration,constraint_type='end')
        c5_prepare = cubic_polynomial(p5_prepare,tr_prepare,c5_start_velocity,c5_start_acceleration,constraint_type='end')
        c6_prepare = cubic_polynomial(p6_prepare,tr_prepare,c6_start_velocity,c6_start_acceleration,constraint_type='end')

        p2_stop = [130.56, 125]
        p3_stop = [96.96, 105.80]
        tl_stop = [2 * Ts]

        p5_stop = [119.28, 116.40, 108.00]
        p6_stop = [132.24, 153.36, 143.28]
        tr_stop = [1 * Ts, 1 * Ts]
        self.walk_stop_time = sum(tr_stop)

        c2_stop = cubic_polynomial(p2_stop,tl_stop,c2_start_velocity,c2_start_acceleration,constraint_type='start')
        c3_stop = cubic_polynomial(p3_stop,tl_stop,c3_start_velocity,c3_start_acceleration,constraint_type='start')
        c5_stop = cubic_polynomial(p5_stop,tr_stop,c5_start_velocity,c5_start_acceleration,constraint_type='start')
        c6_stop = cubic_polynomial(p6_stop,tr_stop,c6_start_velocity,c6_start_acceleration,constraint_type='start')



        self.curve_properties[1] = {'cubic_walk': c2_walk, 'time_stamp': np.cumsum(tl_walk),
                                     'cubic_prepare': c2_prepare, 'time_stamp_prepare': np.cumsum(tl_prepare),
                                     'cubic_stop': c2_stop, 'time_stamp_stop': np.cumsum(tl_stop)}
        self.curve_properties[2] = {'cubic_walk': c3_walk, 'time_stamp': np.cumsum(tl_walk),
                                     'cubic_prepare': c3_prepare, 'time_stamp_prepare': np.cumsum(tl_prepare),
                                     'cubic_stop': c3_stop, 'time_stamp_stop': np.cumsum(tl_stop)}
        self.curve_properties[4] = {'cubic_walk': c5_walk, 'time_stamp': np.cumsum(tr_walk),
                                     'cubic_prepare': c5_prepare, 'time_stamp_prepare': np.cumsum(tr_prepare),
                                     'cubic_stop': c5_stop, 'time_stamp_stop': np.cumsum(tr_stop)}
        self.curve_properties[5] = {'cubic_walk': c6_walk, 'time_stamp': np.cumsum(tr_walk),
                                    'cubic_prepare': c6_prepare, 'time_stamp_prepare': np.cumsum(tr_prepare),
                                    'cubic_stop': c6_stop, 'time_stamp_stop': np.cumsum(tr_stop)}

        
    def walk(self,skip=[6]):
        if self.timer2 < self.walk_prepare_time:
            t = self.timer2
            for i in range(self.servo_num):
                if self.curve_properties[i] == {}:
                    continue
                if t <= self.curve_properties[i]['time_stamp_prepare'][0]:
                    circle_t = t
                    q_i = self.curve_properties[i]['cubic_prepare'][0] * circle_t**3 + self.curve_properties[i]['cubic_prepare'][1] * circle_t**2 + self.curve_properties[i]['cubic_prepare'][2] * circle_t + self.curve_properties[i]['cubic_prepare'][3]
                elif t <= self.curve_properties[i]['time_stamp_prepare'][1]:
                    circle_t = t - self.curve_properties[i]['time_stamp_prepare'][0]
                    q_i = self.curve_properties[i]['cubic_prepare'][4] * circle_t**3 + self.curve_properties[i]['cubic_prepare'][5] * circle_t**2 + self.curve_properties[i]['cubic_prepare'][6] * circle_t + self.curve_properties[i]['cubic_prepare'][7]
                self.q[i] = q_i[0]
            self.set_all_servos(skip=skip)
            return
        
        t = (self.timer2 - self.walk_prepare_time) % self.walk_period

        if self.walk_2_stop:
            if self.walk_wait_time == 0:
                self.walk_wait_time = ceil((self.timer2 - self.walk_prepare_time)/self.walk_period) * self.walk_period

            if self.timer2 > self.walk_prepare_time + self.walk_wait_time and self.timer2 < self.walk_prepare_time + self.walk_wait_time + self.walk_stop_time:
                t = self.timer2 - self.walk_prepare_time - self.walk_wait_time
                for i in range(self.servo_num):
                    if self.curve_properties[i] == {}:
                        continue
                    if t <= self.curve_properties[i]['time_stamp_stop'][0]:
                        circle_t = t
                        q_i = self.curve_properties[i]['cubic_stop'][0] * circle_t**3 + self.curve_properties[i]['cubic_stop'][1] * circle_t**2 + self.curve_properties[i]['cubic_stop'][2] * circle_t + self.curve_properties[i]['cubic_stop'][3]
                    elif t <= self.curve_properties[i]['time_stamp_stop'][1]:
                        circle_t = t - self.curve_properties[i]['time_stamp_stop'][0]
                        q_i = self.curve_properties[i]['cubic_stop'][4] * circle_t**3 + self.curve_properties[i]['cubic_stop'][5] * circle_t**2 + self.curve_properties[i]['cubic_stop'][6] * circle_t + self.curve_properties[i]['cubic_stop'][7]
                    self.q[i] = q_i[0]
                self.set_all_servos(skip=skip)
                return
            elif self.timer2 >= self.walk_prepare_time + self.walk_wait_time + self.walk_stop_time:
                t = self.walk_stop_time
                for i in range(self.servo_num):
                    if self.curve_properties[i] == {}:
                        continue
                    if t <= self.curve_properties[i]['time_stamp_stop'][0]:
                        circle_t = t
                        q_i = self.curve_properties[i]['cubic_stop'][0] * circle_t**3 + self.curve_properties[i]['cubic_stop'][1] * circle_t**2 + self.curve_properties[i]['cubic_stop'][2] * circle_t + self.curve_properties[i]['cubic_stop'][3]
                    elif t <= self.curve_properties[i]['time_stamp_stop'][1]:
                        circle_t = t - self.curve_properties[i]['time_stamp_stop'][0]
                        q_i = self.curve_properties[i]['cubic_stop'][4] * circle_t**3 + self.curve_properties[i]['cubic_stop'][5] * circle_t**2 + self.curve_properties[i]['cubic_stop'][6] * circle_t + self.curve_properties[i]['cubic_stop'][7]
                    self.q[i] = q_i[0]
                self.set_all_servos(skip=skip)
                self.walk_2_stop = False
                self.walk_wait_time = 0
                self.leg_status = None
                return

        for i in range(self.servo_num):
            if self.curve_properties[i] == {}:
                continue
            if t <= self.curve_properties[i]['time_stamp'][0]:
                circle_t = t
                q_i = self.curve_properties[i]['cubic_walk'][0] * circle_t**3 + self.curve_properties[i]['cubic_walk'][1] * circle_t**2 + self.curve_properties[i]['cubic_walk'][2] * circle_t + self.curve_properties[i]['cubic_walk'][3]
            elif t <= self.curve_properties[i]['time_stamp'][1]:
                circle_t = t - self.curve_properties[i]['time_stamp'][0]
                q_i = self.curve_properties[i]['cubic_walk'][4] * circle_t**3 + self.curve_properties[i]['cubic_walk'][5] * circle_t**2 + self.curve_properties[i]['cubic_walk'][6] * circle_t + self.curve_properties[i]['cubic_walk'][7]
            elif t <= self.curve_properties[i]['time_stamp'][2]:
                circle_t = t - self.curve_properties[i]['time_stamp'][1]
                q_i = self.curve_properties[i]['cubic_walk'][8] * circle_t**3 + self.curve_properties[i]['cubic_walk'][9] * circle_t**2 + self.curve_properties[i]['cubic_walk'][10] * circle_t + self.curve_properties[i]['cubic_walk'][11]

            self.q[i] = q_i[0]
        self.set_all_servos(skip=skip)
        
    def proceed_command(self):
        if self.robot_command == 'stand up':
            self.stand_up()
            self.robot_command = ''

        elif self.robot_command == 'shutdown':
            self.head_status = None
            self.leg_status = None
            self.homing()
            self.robot_command = ''
            self.exiting = True

        elif self.robot_command == 'nod':
            if self.head_status == None:
                self.head_status = 'nod'
                self.timer1 = 0
                print('Start noding! Time: {:.2f}'.format(self.running_time))
            else:
                print('Cannot nod head! Please stop the current head movement ({}) first! Time: {:.2f}'.format(self.head_status), self.running_time)
            self.robot_command = ''

        elif self.robot_command == 'stop head':
            self.head_status = None
            self.servos[6].move(100,1000)
            print('Successfully stop noding! Time: {:.2f}'.format(self.running_time))
            self.robot_command = ''

        elif self.robot_command == 'squat':
            if self.leg_status == None:
                self.leg_status = 'squat'
                self.prepare_squat(squat_ratio=0.6, squat_frequency=2)
                self.timer2 = 0
                print('Start squatting! Time: {:.2f}'.format(self.running_time))
            else:
                print('Cannot squat! Please stop the current leg movement ({}) first! Time: {:.2f}'.format(self.leg_status, self.running_time))
            self.robot_command = ''

        elif self.robot_command == 'stop leg':
            self.leg_status = None
            self.stand_up(skip=[6])
            print('Successfully stop moving legs! Time: {:.2f}'.format(self.running_time))
            self.robot_command = ''

        elif self.robot_command == 'stop':
            self.head_status = None
            self.leg_status = None
            self.stand_up()
            print('Successfully stop moving! Time: {:.2f}'.format(self.running_time))
            self.robot_command = ''
            
        elif self.robot_command == 'step':
            if self.leg_status == None:
                self.leg_status = 'step'
                self.timer2 = 0
                self.prepare_step(step_period=0.4)
                print('Start stepping! Time: {:.2f}'.format(self.running_time))
                self.robot_command = ''
            else:
                print('Cannot step! Please stop the current leg movement ({}) first! Time: {:.2f}'.format(self.leg_status, self.running_time))

        elif self.robot_command == 'walk1':
            if self.leg_status == None:
                self.leg_status = 'walk'
                self.prepare_walk(walk_period=0.8, skip=[6])
                self.timer2 = 0
                print('Start walking! Time: {:.2f}'.format(self.running_time))
                self.robot_command = ''
            else:
                print('Cannot walk! Please stop the current leg movement ({}) first! Time: {:.2f}'.format(self.leg_status, self.running_time))
        elif self.robot_command == 'stop walk':
            self.walk_2_stop = True
            print('Successfully stop walking! Time: {:.2f}'.format(self.running_time))
            self.robot_command = ''

        elif self.robot_command == '':
            pass
        else:
            print('Invalid command! Time: {:.2f}'.format(self.running_time))
            self.robot_command = ''

    def robot_loop(self):
        self.homing()
        try:
            while True:
                self.running_time = time.time() - self.start_time
                self.proceed_command()
                if self.exiting:
                    break

                if self.head_status == 'nod':
                    self.nod_head()
                    self.timer1 += self.main_step_time
                if self.leg_status == 'squat':
                    self.squat()
                    self.timer2 += self.main_step_time
                if self.leg_status == 'step':
                    self.step()
                    self.timer2 += self.main_step_time
                if self.leg_status == 'walk':
                    self.walk()
                    self.timer2 += self.main_step_time
                if self.leg_status or self.head_status:
                    time.sleep(self.main_sleep_time)
            
                if time.time() - self.time_stamp > self.check_period:
                    self.self_check()
                    self.time_stamp = time.time()
        except:
            print('Error occurs! Time: {:.2f}'.format(self.running_time))
            self.homing()

        for i in range(self.servo_num):
            self.servos[i].disable_torque()
            self.servos[i].led_power_off()
        print('Successfully shutdown! Time: {:.2f}'.format(self.running_time))
    
def main():
    robot = Robot(7)
    input_thread = threading.Thread(target=robot.listen_input, daemon=True)
    input_thread.start()
    robot.robot_loop()

if __name__ == '__main__':
    main()