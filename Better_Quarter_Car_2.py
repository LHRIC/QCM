import math 
import numpy as np
import matplotlib.pyplot as plt
import roadNoise
import roadTrapezoid
from scipy import integrate
from scipy.integrate import odeint
from scipy.interpolate import CubicSpline

def qcm_dstate(state, vars, constants, t):
    x1, x2, v1, v2 = state[:]
    distance_road, vel_road, disp_road, time_array = vars[:]
    m1, m2, k1, k2, c1, c2, l1_0, l2_0, g = constants[:]
    
    #interpolate for these values
    vroad = np.interp(t,time_array, vel_road)
    droad = np.interp(t,time_array, disp_road)
    distance = np.interp(t,time_array, distance_road)

    dstate = qcm_calcs(x1,x2,v1,v2,l1_0,l2_0,vroad,droad,distance)[0]
    
    return dstate

def qcm_calcs(x1, x2, v1, v2, l1_0, l2_0 ,vroad, droad, distance):
    #current spring lengths
    l1 = x1 - droad
    l2 = x2 - x1
    
    #set spring lengths to max uncompressed to prevent extension into tension
    if l1 > l1_0:
        l1 = l1_0
    if l2 > l2_0:
        l2 = l2_0

    #Spring Forces
    Fs1 = -k1*(l1 - l1_0) 
    Fs2 = -k2*(l2 - l2_0)

    #Damper forces
    Fd1 = (v1-vroad)*c1
    Fd2 = (v2-v1)*c2
    if x1 > (droad + l1_0): #If QQM is off the ground, set bottom damper to zero
        Fd1 = 0

    #Net forces
    F1 = Fs1 - Fd1
    F2 = Fs2 - Fd2

    print(F1)

    # Calculate Derivatives of the State
    dx1 = v1                        #v1
    dx2 = v2                        #v2
    dv1 = (F1-F2 - m1*g) / m1       #a1
    dv2 = (F2 - m2*g) / m2          #a2

    dstate = [dx1, dx2, dv1, dv2]

    return [dstate, dv1, dv2, l1, l2, Fs1, Fs2, Fd1, Fd2]

#QCM INPUTS
#define car input paramers
g = 9.81            #m/s^2
endtime = 2        #seconds
step_time = 0.001   #seconds
constant_vel = 15   #m/s
time = np.linspace(0, endtime, math.floor(endtime/step_time+1))
print(time.size)

#define road profile
road_distance = constant_vel * time #longitduinal meteres traveled)

road_noise_disp, road_noise_vel, road_noise_accel = roadNoise.roadProfile(road_distance,'H',0)[:]
road_step_vel = roadTrapezoid.trapezoid(road_distance, 0.3, 0.2, 20/1000, 0.2, 2) 
#load_transfer = roadTrapezoid.trapezoid(road_distance, 0.6, 0.1, 10, 0.1, 1) #in N 

road_vel_profile = road_step_vel + 10*road_noise_vel  #combined velocity in m/m profiles of bump trapezoid corresponds to x distance positions
road_disp_profile = integrate.cumulative_simpson(road_vel_profile, x = road_distance, initial = 0) #integrated from velocity used as qcm input

#car inputs
m1 = 15         #kg mass 1 (unsprung)
m2 = 80         #kg mass 2 (quarter car)
k1 = 157000     #N/m    tire spring rate
k2 = 30000      #N/m    suspension spring rate (single corner)

#mass heights at equilibrium
x1_0 = 200 / 1000    #m initial position of mass 1
x2_0 = 500 / 1000    #m initial position of mass 2
v1_0 = 0
v2_0 = 0

#l0 is spring length zero force, calcluated for static equilibrium initial condition of x1 and x2
l1_0 = x1_0 + g*(m1+m2)/k1       #m
l2_0 = x2_0 - x1_0 + g*m2/k2     #m

#critical damping
m1CD = 2*math.sqrt(k1*m1) 
m2CD = 2*math.sqrt(k2*m2)

#damping coefficients
c1 = 0.2*m1CD      #N/m/s  tire damping
c2 = 0.7*m2CD      #N/m/s  spring damping

#Initial Conditions 
state_0 = [x1_0, x2_0, v1_0 ,v2_0]      #initial state x1, x2, v1, v2

c = [m1, m2, k1, k2, c1, c2, l1_0, l2_0, g]                    #constants
v = [road_distance, road_vel_profile, road_disp_profile, time]  #constants that change but not depending on the state

def qcmFunction(x,t):
    return qcm_dstate(x, v, c, t)

x_out = odeint(qcmFunction, state_0, time)
x1_arr = x_out[:,0]
x2_arr = x_out[:,1]
v1_arr = x_out[:,2]
v2_arr = x_out[:,3]

print(x1_arr)

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
    a1_arr[i] = qcm_calcs(x1_arr[i], x2_arr[i], v1_arr[i], v2_arr[i], l1_0, l2_0, road_vel_profile[i], road_disp_profile[i], road_distance[i])[1]
    a2_arr[i] = qcm_calcs(x1_arr[i], x2_arr[i], v1_arr[i], v2_arr[i], l1_0, l2_0, road_vel_profile[i], road_disp_profile[i], road_distance[i])[2]
    l1_arr[i] = qcm_calcs(x1_arr[i], x2_arr[i], v1_arr[i], v2_arr[i], l1_0, l2_0, road_vel_profile[i], road_disp_profile[i], road_distance[i])[3]
    l2_arr[i] = qcm_calcs(x1_arr[i], x2_arr[i], v1_arr[i], v2_arr[i], l1_0, l2_0, road_vel_profile[i], road_disp_profile[i], road_distance[i])[4]
    Fs1_arr[i] = qcm_calcs(x1_arr[i], x2_arr[i], v1_arr[i], v2_arr[i], l1_0, l2_0, road_vel_profile[i], road_disp_profile[i], road_distance[i])[5]
    Fs2_arr[i] = qcm_calcs(x1_arr[i], x2_arr[i], v1_arr[i], v2_arr[i], l1_0, l2_0, road_vel_profile[i], road_disp_profile[i], road_distance[i])[6]
    Fd1_arr[i] = qcm_calcs(x1_arr[i], x2_arr[i], v1_arr[i], v2_arr[i], l1_0, l2_0, road_vel_profile[i], road_disp_profile[i], road_distance[i])[7]
    Fd2_arr[i] = qcm_calcs(x1_arr[i], x2_arr[i], v1_arr[i], v2_arr[i], l1_0, l2_0, road_vel_profile[i], road_disp_profile[i], road_distance[i])[8]

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
