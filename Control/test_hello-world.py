from math import sin, cos
from pylx16a.lx16a import *
import time

# change "/COM6" to the port where the LX-16A is connected
LX16A.initialize("/COM6", 0.1)

try:
    servo1 = LX16A(1)
    servo2 = LX16A(2)
    servo3 = LX16A(3)
    servo1.set_angle_limits(0, 240)
    servo2.set_angle_limits(0, 240)
    servo3.set_angle_limits(0, 240)
except ServoTimeoutError as e:
    print(f"Servo {e.id_} is not responding. Exiting...")
    quit()

t = 0
# give enough time for servo to initialization
time.sleep(3)
# move to start position
servo1.move(120,2000)
servo2.move(120,2000)
servo3.move(120,2000)
time.sleep(3)
while True:
    servo1.move(sin(t) * 20 + 120)
    servo2.move(sin(t) * 20 + 120)
    servo3.move(sin(t) * 20 + 120)

    time.sleep(0.05)
    t += 0.1
