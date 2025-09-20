import numpy as np
import math

# globals.py
g = 9.81            #m/s^2
endtime = 0.5        #seconds
step_time = 0.001   #seconds
constant_vel = 15   #m/s
time = np.linspace(0, endtime, math.floor(endtime/step_time+1))

# damper_curves settings

damper_curve = 0      #damper curve
damper_setting = 0    #curve setting