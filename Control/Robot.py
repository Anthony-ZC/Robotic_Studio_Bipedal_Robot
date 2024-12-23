from math import sin, cos, ceil
from pylx16a.lx16a import *
import time,os
import numpy as np
import threading
from copy import deepcopy
import pygame
import speech_recognition as sr

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

class ServoError(Exception):
    '''
    Exception raised for errors in the servo.
    '''
    def __init__(self, message):
        self.message = message
 
    def __str__(self):
        return self.message


class Robot:
    '''
    Robot class for controlling the robot.
    '''
    def __init__(self, servo_num,control_mode = 'keyboard', enable_face = False):
        '''
        Initialize the robot.
        
        Args:
            servo_num (int): number of servos
            control_mode (str): control mode, 'keyboard' or 'mic'
            enable_face (bool): enable face or not
        '''

        # Initialize the servos
        try:
            # Raspberry Pi
            LX16A.initialize("/dev/ttyUSB0", 0.1)
        except:
            # Windows
            LX16A.initialize("/COM6", 0.1)

        self.servo_num = servo_num
        self.q = [0]*self.servo_num
        self.servo_status = [[False, False, False] for i in range(self.servo_num)]

        # Set the limits for the servos' voltage, temperature, and angle
        self.voltage_lower_limit = 4500
        self.voltage_upper_limit = 12000
        self.temperature_limit = 70
        self.angle_lower_limit = 0
        self.angle_upper_limit = 240

        # Initialize the robot command
        self.robot_command = ''
        self.exiting = False

        # Initialize the head and leg status
        self.leg_status = None
        self.head_status = None

        # Initialize the walk parameters
        self.walk_prepare_time = 0
        self.walk_stop_time = 0
        self.walk_2_stop = False
        self.walk_wait_time = 0
        
        # Initialize the timer and time
        self.start_time = time.time()
        self.time_stamp = time.time()
        self.running_time = 0
        self.check_period = 10
        # timer1 for head movement, timer2 for leg movement
        self.timer1 = 0
        self.timer2 = 0

        # Initialize the main loop parameters
        # time for long time operation
        self.mian_proceed_time = 2
        # time for each step
        self.main_step_time = 0.1
        # time for each sleep
        self.main_sleep_time = 0.05

        # Initialize the pose for homing and standing up
        self.pose_homing = [114, 94.48, 95.76, 116.64, 137.76, 151.68, 100]
        self.pose_stand_up = [114, 120, 104, 116.88, 108, 148, 100]

        # Initialize the cubic polynomial coefficients for the walk
        self.curve_properties = [{} for i in range(self.servo_num)]
        
        # Initialize the thread for input command
        if control_mode == 'keyboard':
            self.input_thread = threading.Thread(target=self.listen_input_keyboard, daemon=True)
        elif control_mode == 'mic':
            self.recognizer = sr.Recognizer()
            self.input_thread = threading.Thread(target=self.listen_input_mic, daemon=True)
        self.input_thread.start()
        
        # Initialize the face
        self.enable_face = enable_face
        self.initial_face()
        self.initialize_servos()

    def listen_input_keyboard(self):
        '''
        Listen to the keyboard input.
        '''
        while True:
            self.robot_command = input() 
            print('Get command: {}'.format(self.robot_command))

    def listen_input_mic(self):
        '''
        Listen to the microphone input.
        '''

        with sr.Microphone() as source:
            self.recognizer.adjust_for_ambient_noise(source)
            print("Start listening command...")
            
            while True:
                try:
                    audio_data = self.recognizer.listen(source, timeout=5, phrase_time_limit=10)
                    print("Recognizing...")

                    # if you want to use another language, change the language parameter
                    text = self.recognizer.recognize_google(audio_data, language="en-US") 
                    self.robot_command = text
                    print('Get command: {}'.format(self.robot_command))
                
                except sr.WaitTimeoutError:
                    print("No voidce input detected, please try again!")
                except sr.UnknownValueError:
                    print("Cannot recognize the voice, please try again!")
                except sr.RequestError as e:
                    print(f"Voice recognition service error: {e}")

    def initial_face(self):
        '''
        Initialize the face.
        '''
        if not self.enable_face:
            return
        
        # Set the display, ':0' for Raspberry Pi touch screen
        os.environ["DISPLAY"] = ":0" 
        pygame.init()
        pygame.mouse.set_visible(False)
        self.screen = pygame.display.set_mode((480, 320), pygame.FULLSCREEN)
        pygame.display.set_caption("Face")
        self.BLACK = (0, 0, 0)
        self.WHITE = (255, 255, 255)

        # Load the face images
        self.face_folder = "face"
        self.face_images = {}
        if os.path.exists(self.face_folder):
            for filename in os.listdir(self.face_folder):
                if filename.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.gif')):
                    image_path = os.path.join(self.face_folder, filename)
                    image = pygame.image.load(image_path)
                    image = pygame.transform.scale(image, (480, 320))
                    emotion = filename.split('.')[0]
                    self.face_images[emotion] = image

        # Set the current and backup emotion
        self.current_emotion = 'sleep'
        self.backup_emotion = 'neutral'
        self.show_face()

    def show_face(self):
        '''
        Show the face.
        '''
        if not self.enable_face:
            return
        # Fill the screen with black
        self.screen.fill(self.BLACK)

        # Show the corresponding face image
        self.screen.blit(self.face_images[self.current_emotion], (0, 0))

        # Update the display
        pygame.display.flip()
        
        # Save the current emotion as the backup emotion
        self.backup_emotion = self.current_emotion

    def initialize_servos(self):
        # Initialize the servos
        try:
            self.servos = [LX16A(i) for i in range(1,self.servo_num + 1)]
            for i in range(self.servo_num):
                # Set the limits for the servos
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
        '''
        Set all the servos to the target angles.
        
        Args:
            move_time (float): time for moving to the target angles
            skip (list): list of servos to skip
            '''
        assert self.servo_num == len(self.q)
        for i in range(len(self.q)):
            if i in skip:
                continue
            self.servos[i].move(self.q[i],move_time)
    
    def self_check(self):
        '''
        Check the status of the robot.
        '''
        for i in range(self.servo_num):
            # Get the status of the servos.
            self.servo_status[i] = self.servos[i].get_led_error_triggers(True)

            # If the temperature is too high, the voltage is abnormal, or the servo is rocked, print the warning message.
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
        '''
        Move the robot to the homing pose.
        '''
        self.q = deepcopy(self.pose_homing)
        self.set_all_servos(self.mian_proceed_time * 1000,skip=skip)
        time.sleep(self.mian_proceed_time + 1)
        print('Successfully move to homing pose! Time: {:.2f}'.format(self.running_time))
    
    def stand_up(self,skip=[]):
        '''
        Move the robot to the standing up pose.
        '''
        self.q = deepcopy(self.pose_stand_up)
        self.set_all_servos(self.mian_proceed_time * 1000,skip=skip)
        time.sleep(self.mian_proceed_time + 1)
        print('Successfully stand up! Time: {:.2f}'.format(self.running_time))

    def nod_head(self):
        '''
        Nod the head.
        '''
        self.servos[6].move(sin(self.timer1) * 15 + 100)

    def prepare_squat(self, squat_ratio = 0.6, squat_frequency = 2):
        '''
        Prepare the squat.
        
        Args:
            squat_ratio (float): squat ratio
            squat_frequency (float): squat frequency
        '''

        # Calculate the difference between the stand up pose and the homing pose
        old_difference = [self.pose_stand_up[i]-self.pose_homing[i] for i in range(self.servo_num)]
        # Calculate the squat pose with the squat ratio
        down = [self.pose_stand_up[i]-old_difference[i] * squat_ratio for i in range(self.servo_num)]

        # Calculate the squatting difference, middle, and frequency
        self.squating_frequency = squat_frequency
        self.squating_difference = [(self.pose_stand_up[i]-down[i])/2 for i in range(self.servo_num)]
        self.squating_middle = [(self.pose_stand_up[i]+down[i])/2 for i in range(self.servo_num)]

    def squat(self,skip=[]):
        '''
        Squat the robot.

        Args:
            skip (list): list of servos to skip
        '''
        self.q = [self.squating_difference[i]*cos(self.squating_frequency*self.timer2) + self.squating_middle[i] for i in range(self.servo_num)]
        self.set_all_servos(skip=skip)

    def prepare_step(self,step_period = 0.4):
        '''
        Prepare the step.
        
        Args:
            step_period (float): step period
        '''
        self.step_period = step_period
        self.step_ferq = 2 * np.pi / step_period
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

    def prepare_walk(self, walk_period = 0.8):
        '''
        Prepare the walk.
        
        Args:
            walk_period (float): walk period
        '''
        self.walk_period = walk_period
        # self.walk_pose = {'left_font': {2:130.56, 3:96.96},
        #                   'left_back': {2:111.60, 3:116.88},
        #                   'left_trainsition': {2:111.60, 3:96.96},
        #                   'right_font': {5:97.92, 6:153.84},
        #                   'right_back': {5:119.28, 6:132.24},
        #                   'right_trainsition': {5:116.40, 6:153.36}}
        # Walk pose
        self.walk_pose = {'left_font': {2:132, 3:94},
                          'left_back': {2:112, 3:114},
                          'left_trainsition': {2:112, 3:94},
                          'right_font': {5:98, 6:158},
                          'right_back': {5:118, 6:138},
                          'right_trainsition': {5:118, 6:158}}
        Ts = walk_period/4.0

        # Cubic polynomial for walking
        p2_walk = [self.walk_pose['left_font'][2], self.walk_pose['left_back'][2], self.walk_pose['left_trainsition'][2], self.walk_pose['left_font'][2]]
        p3_walk = [self.walk_pose['left_font'][3], self.walk_pose['left_back'][3], self.walk_pose['left_trainsition'][3], self.walk_pose['left_font'][3]]
        tl_walk = [2*Ts , Ts, Ts]
        
        c2_walk = cubic_polynomial_periodic(p2_walk,tl_walk)
        c3_walk = cubic_polynomial_periodic(p3_walk,tl_walk)
        
        p5_walk = [self.walk_pose['right_back'][5], self.walk_pose['right_trainsition'][5], self.walk_pose['right_font'][5], self.walk_pose['right_back'][5]]
        p6_walk = [self.walk_pose['right_back'][6], self.walk_pose['right_trainsition'][6], self.walk_pose['right_font'][6], self.walk_pose['right_back'][6]]
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

        # In perapre, the end velocity and acceleration are constrained
        p2_prepare = [self.pose_stand_up[2-1], self.walk_pose['left_trainsition'][2], self.walk_pose['left_font'][2]]
        p3_prepare = [self.pose_stand_up[3-1], self.walk_pose['left_trainsition'][3], self.walk_pose['left_font'][3]]
        tl_prepare = [1 * Ts, 1 * Ts]

        p5_prepare = [self.pose_stand_up[5-1],self.walk_pose['right_back'][5]]
        p6_prepare = [self.pose_stand_up[6-1],self.walk_pose['right_back'][6]]
        tr_prepare = [2 * Ts]
        self.walk_prepare_time = sum(tl_prepare)
        
        c2_prepare = cubic_polynomial(p2_prepare,tl_prepare,c2_start_velocity,c2_start_acceleration,constraint_type='end')
        c3_prepare = cubic_polynomial(p3_prepare,tl_prepare,c3_start_velocity,c3_start_acceleration,constraint_type='end')
        c5_prepare = cubic_polynomial(p5_prepare,tr_prepare,c5_start_velocity,c5_start_acceleration,constraint_type='end')
        c6_prepare = cubic_polynomial(p6_prepare,tr_prepare,c6_start_velocity,c6_start_acceleration,constraint_type='end')

        # In stop, the start velocity and acceleration are constrained
        p2_stop = [self.walk_pose['left_font'][2],self.pose_stand_up[2-1]]
        p3_stop = [self.walk_pose['left_font'][3],self.pose_stand_up[3-1]]
        tl_stop = [2 * Ts]

        p5_stop = [self.walk_pose['right_back'][5],self.walk_pose['right_trainsition'][5],self.pose_stand_up[5-1]]
        p6_stop = [self.walk_pose['right_back'][6],self.walk_pose['right_trainsition'][6],self.pose_stand_up[6-1]]
        tr_stop = [1 * Ts, 1 * Ts]
        self.walk_stop_time = sum(tr_stop)

        c2_stop = cubic_polynomial(p2_stop,tl_stop,c2_start_velocity,c2_start_acceleration,constraint_type='start')
        c3_stop = cubic_polynomial(p3_stop,tl_stop,c3_start_velocity,c3_start_acceleration,constraint_type='start')
        c5_stop = cubic_polynomial(p5_stop,tr_stop,c5_start_velocity,c5_start_acceleration,constraint_type='start')
        c6_stop = cubic_polynomial(p6_stop,tr_stop,c6_start_velocity,c6_start_acceleration,constraint_type='start')

        # Save the cubic polynomial coefficients
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
        '''
        Walk the robot.
        
        Args:
            skip (list): list of servos to skip
        '''

        # Prepare phase
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
        # Stop phase
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

        # Periodic walking phase
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
        '''
        Proceed the command.
        '''
        if self.robot_command == 'wake up':
            self.current_emotion = 'neutral'

            self.stand_up()
            self.robot_command = ''

        elif self.robot_command == 'shut down':
            self.head_status = None
            self.leg_status = None
            self.current_emotion = 'sleep'
            self.exiting = True

            self.homing()
            self.robot_command = ''

        elif self.robot_command == 'sleep':
            self.head_status = None
            self.leg_status = None
            self.current_emotion = 'sleep'

            self.homing()
            self.robot_command = ''

        # Fot head movement, the head_status is used to check the current head movement
        # The head_status can be 'nod' or None, which means the head is noding or not
        # There is a lock for the head movement, which means the head cannot move when the head_status is not None
        # The head_status should be reset to None after the head movement is finished
        # Timer1 is used to record the time for the head movement and should be reset to 0 at the beginning of the head movement
        elif self.robot_command == 'shake head':
            if self.head_status == None:
                self.head_status = 'nod'
                self.timer1 = 0
                self.current_emotion = 'happy'
                print('Start noding! Time: {:.2f}'.format(self.running_time))
            else:
                print('Cannot nod head! Please stop the current head movement ({}) first! Time: {:.2f}'.format(self.head_status), self.running_time)
            
            self.robot_command = ''

        elif self.robot_command == 'stop shaking head':
            self.head_status = None
            self.servos[6].move(100,1000)
            print('Successfully stop noding! Time: {:.2f}'.format(self.running_time))
            self.current_emotion = 'neutral'
            self.robot_command = ''

        # For leg movement, the leg_status is used to check the current leg movement
        # The leg_status can be 'squat', 'step', 'walk', or None, which means the leg is squatting, stepping, walking, or not moving
        # There is a lock for the leg movement, which means the leg cannot move when the leg_status is not None
        # The leg_status should be reset to None after the leg movement is finished
        # Timer2 is used to record the time for the leg movement and should be reset to 0 at the beginning of the leg movement
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
            self.current_emotion = 'neutral'
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

        elif self.robot_command == 'walk':
            if self.leg_status == None:
                self.leg_status = 'walk'
                self.timer2 = 0
                self.current_emotion = 'happy'
                self.show_face()
                self.prepare_walk(walk_period=0.8, skip=[6])
                print('Start walking! Time: {:.2f}'.format(self.running_time))
                self.robot_command = ''
            else:
                print('Cannot walk! Please stop the current leg movement ({}) first! Time: {:.2f}'.format(self.leg_status, self.running_time))
        
        elif self.robot_command == 'stop walk':
            self.walk_2_stop = True
            self.current_emotion = 'neutral'
            print('Successfully stop walking! Time: {:.2f}'.format(self.running_time))
            self.robot_command = ''
        
        # For face emotion, the current_emotion is used to show the current face emotion
        # The current_emotion can be 'happy', 'sad', 'angry', 'question', 'neutral', or 'sleep'
        # For certain commands, the face emotion will change to 'happy', 'sad', or 'angry' for a while and then change back to 'neutral'
        # If the command is unknown, the face emotion will change to 'question' for a while and then change back to 'neutral'
        elif self.robot_command in  ['hello','hi','thank you', 'you are clever', 'you are beautiful' ,"you're clever", "you're beautiful"]:
            self.current_emotion = 'happy'
            self.show_face()
            time.sleep(2)
            self.current_emotion = 'neutral'
            self.robot_command = ''

        elif self.robot_command in  ['you are stupid',  "you're stupid"]:
            self.current_emotion = 'angry'
            self.show_face()
            time.sleep(2)
            self.current_emotion = 'neutral'
            self.robot_command = ''
        elif self.robot_command in ['you are ugly',"you're ugly"]:
            self.current_emotion = 'sad'
            self.show_face()
            time.sleep(2)
            self.current_emotion = 'neutral'
            self.robot_command = ''

        elif self.robot_command == '':
            pass
        else:
            if self.leg_status==None and self.head_status==None:
                self.current_emotion = 'question'
                self.show_face()
                time.sleep(2)
                self.current_emotion = 'neutral'
            print('Unkown command! Time: {:.2f}'.format(self.running_time))
            self.robot_command = ''

        if self.current_emotion != self.backup_emotion:
            self.show_face()

    def robot_loop(self):
        '''
        Main loop for the robot.
        '''
        # before the main loop, the robot should be homed
        self.homing()

        try:
            while True:
                # Update the running time
                self.running_time = time.time() - self.start_time

                # Proceed the command
                self.proceed_command()

                # If the robot is exiting, break the loop
                if self.exiting:
                    break

                # If head_status or leg_status is not None, proceed the head or leg movement and update the timer
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
                
                # For every check_period, the robot should check the status
                if time.time() - self.time_stamp > self.check_period:
                    self.self_check()
                    self.time_stamp = time.time()
        except:
            print('Error occurs! Time: {:.2f}'.format(self.running_time))
            self.homing()
        
        # For the shutdown, the robot should release the servos' torque and power off the led
        for i in range(self.servo_num):
            self.servos[i].disable_torque()
            self.servos[i].led_power_off()
        print('Successfully shutdown! Time: {:.2f}'.format(self.running_time))
    
def main():
    # Initialize the robot
    robot = Robot(7, 'keyboard', True)
    robot.robot_loop()

if __name__ == '__main__':
    main()