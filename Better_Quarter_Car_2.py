import math 
import numpy as np
import matplotlib.pyplot as plt
import roadNoise
import roadTrapezoid
import qcm_types
from scipy import integrate
from scipy.integrate import odeint
from scipy.interpolate import CubicSpline
from constants import *
from qcm_calculations import *

#QCM INPUTS
#define car input paramers

#define road profile
road_distance = constant_vel * time #longitduinal meteres traveled)

road_noise_disp, road_noise_vel, road_noise_accel = roadNoise.roadProfile(road_distance,'H',0)[:]
road_step_vel = roadTrapezoid.trapezoid(road_distance, 0.3, 0.2, 20/1000, 0.2, 2) 
#load_transfer = roadTrapezoid.trapezoid(road_distance, 0.6, 0.1, 10, 0.1, 1) #in N 

road_vel_profile = road_step_vel + 10*road_noise_vel  #combined velocity in m/m profiles of bump trapezoid corresponds to x distance positions
road_disp_profile = integrate.cumulative_simpson(road_vel_profile, x = road_distance, initial = 0) #integrated from velocity used as qcm input

# car parameters
'mu = unsprung mass (kg)'
'ms = sprung mass (kg)'
'x1_0 = initial position of unsprung mass (m)'
'x2_0 = initial position of sprung mass (m)'
'v1_0 = initial velocity of unsprung mass (m/s)'
'v2_0 = initial velocity of sprung mass (m/s)'
'k1 = tire spring rate (N/m)'
'k2 = suspension spring rate (N/m)'
'c1_percentage = tire damper as percentage of critical damping (0 to 1)'
'c2_percentage = suspension damper as percentage of critical damping (0 to 1)'

# create car object
car = qcm_types.QuarterCarModel(mu = 15.0, 
                      ms = 80.0, 
                      x1_0 = 200 / 1000, 
                      x2_0 = 500 / 1000, 
                      k1 = 157000, 
                      k2 = 30000, 
                      c1_percentage = 0.2, 
                      c2_percentage = 0.7)

### black box

#Initial Conditions 
state_0 = [car.mu.x, car.ms.x, car.mu.vy, car.ms.vy]      #initial state x1, x2, v1, v2
constants = [car.mu.mass, car.ms.mass, car.k1.k, car.k2.k, car.c1.c, car.c2.c, car.k1.l_0, car.k2.l_0]                    #constants
vars = [road_distance, road_vel_profile, road_disp_profile, time]  #constants that change but not depending on the state

def qcmFunction(x,t):
    return qcm_dstate(x, vars, constants, t)

x_out = odeint(qcmFunction, state_0, time)
x1_arr = x_out[:,0]
x2_arr = x_out[:,1]
v1_arr = x_out[:,2]
v2_arr = x_out[:,3]

### black box

#initialize calcs arrays
a1_arr = np.zeros(len(time)) 
a2_arr = np.zeros(len(time)) 
l1_arr = np.zeros(len(time)) 
l2_arr = np.zeros(len(time)) 
Fs1_arr = np.zeros(len(time)) 
Fs2_arr = np.zeros(len(time)) 
Fd1_arr = np.zeros(len(time)) 
Fd2_arr = np.zeros(len(time)) 
Fnet1_arr = np.zeros(len(time)) 
Fnet2_arr = np.zeros(len(time)) 

#re calc all extras 
for i in range(len(time)):
    car.mu.x = x1_arr[i]
    car.mu.vy = v1_arr[i]
    car.ms.x = x2_arr[i]
    car.ms.vy = v2_arr[i]

    calcs = qcm_calcs_car(car, road_vel_profile[i], road_disp_profile[i], road_distance[i])

    a1_arr[i] = calcs[1]
    a2_arr[i] = calcs[2]
    l1_arr[i] = calcs[3]
    l2_arr[i] = calcs[4]
    Fs1_arr[i] = calcs[5]
    Fs2_arr[i] = calcs[6]
    Fd1_arr[i] = calcs[7]
    Fd2_arr[i] = calcs[8]

    Fnet1_arr[i] = Fs1_arr[i]+ Fd1_arr[i]
    Fnet2_arr[i] = Fs2_arr[i]+ Fd2_arr[i]

fig, axs = plt.subplots(3,2)
axs[0,0].plot(time, road_disp_profile*1000, 'tab:brown', label='ground')
axs[0,0].set_title('ground (mm)')
axs[0,0].legend(ncol=3, loc='upper right')

axs[1,0].plot(time, x1_arr*1000, 'tab:blue', label='x1')
axs[1,0].plot(time, x2_arr*1000, 'tab:red', label='x2')
axs[1,0].plot(time, road_disp_profile*1000, 'tab:brown', label='ground')
axs[1,0].set_title('ground, x1, and x2 (mm)')
axs[1,0].legend(ncol=3, loc='upper right')

axs[2,0].plot(time, v1_arr, 'tab:blue', label='v1')
axs[2,0].plot(time, v2_arr, 'tab:red', label='v2')
axs[2,0].set_title('v1, and v2 m/s')
axs[2,0].legend(ncol=2, loc='upper right')

#acceleration has v as placeholder
axs[0,1].plot(time, a1_arr, 'tab:blue', label='a1')
axs[0,1].plot(time, a2_arr, 'tab:red', label='a2')
axs[0,1].set_title('a1, and a2 m/s^2')
axs[0,1].legend(ncol=2, loc='upper right')

#placeholder for spring lengths as well
"""Others"""
axs[1,1].plot(time, l1_arr, 'tab:blue', label='l1')
axs[1,1].plot(time, l2_arr, 'tab:red', label='l2')
axs[1,1].set_title('Springs Lengths (x1 to ground and x1 to x2)')
axs[1,1].legend(ncol=2, loc='upper right')

#axs[1,1].plot(tlog, v1log, 'tab:red')
#axs[1,1].plot(tlog, v2log, 'tab:green')
#axs[1,1].set_title('v1, and v2 m/s')

#placeholder for forces
axs[2,1].plot(time, Fnet1_arr, 'tab:blue', label='F Net 1')
axs[2,1].plot(time, Fs1_arr, 'tab:cyan', label='F Spring 1')
axs[2,1].plot(time, Fd1_arr, 'tab:green', label='F Damper 1')

axs[2,1].plot(time, Fnet2_arr, 'tab:red', label='F Net 2')
axs[2,1].plot(time, Fs2_arr, 'tab:pink', label='F Spring 2')
axs[2,1].plot(time, Fd2_arr, 'tab:orange', label='F Damper 2')

axs[2,1].set_title('Spring Damper Forces (N)')
axs[2,1].legend(ncol=2, loc='upper right')

axs[2,0].set(xlabel='t (sec)')
axs[2,1].set(xlabel='t (sec)')

plt.show()
