import math 
import numpy as np
import matplotlib.pyplot as plt
import roadNoise
import roadTrapezoid
from scipy import integrate
from scipy.integrate import odeint
from scipy.interpolate import CubicSpline

#QCM GLOBALS
g = 9.81            #m/s^2
endtime = 2       #seconds
step_time = 0.001   #seconds
constant_vel = 15   #m/s
time = np.linspace(0, endtime, math.floor(endtime/step_time+1))

class Mass:
    def __init__(self, m, x_0):
        self.mass = m
        self.x = x_0
        self.vy = 0
        self.ay = -g
class QCM:
    def __init__(self, ms, mu, k, c_percentage):
        self.l = ms * g / k  # length of spring
        self.mu = Mass(mu, 0)
        self.ms = Mass(ms, self.l)
        self.vx = constant_vel  #initial velocity of spring-damper
        self.k = k              #spring stiffness

        self.CD = 2 * math.sqrt(k * (self.ms.mass + self.mu.mass))
        self.c = c_percentage * self.CD        #damper coefficient

#define road profile
road_distance = constant_vel * time #longitduinal meteres traveled

road_noise_disp, road_noise_vel, road_noise_accel = roadNoise.roadProfile(road_distance,'H',0)[:]
road_step_vel = roadTrapezoid.trapezoid(road_distance, 0.3, 0.2, 20/1000, 0.2, 2) 
#load_transfer = roadTrapezoid.trapezoid(road_distance, 0.6, 0.1, 10, 0.1, 1) #in N 

road_vel_profile = road_step_vel + 10*road_noise_vel  #combined velocity in m/m profiles of bump trapezoid corresponds to x distance positions
road_disp_profile = integrate.cumulative_simpson(road_vel_profile, x = road_distance, initial = 0) #integrated from velocity used as qcm input

positions = road_disp_profile
velocities = np.gradient(road_vel_profile)
accelerations = np.gradient(velocities)

def step(qcm_obj, acceleration, velocity, position, time):
    spring_delta = qcm_obj.l - max(0, qcm_obj.l - position[time])
    input_force = acceleration[time] * (qcm_obj.mu.mass + qcm_obj.ms.mass)

    return qcm_acceleration(input_force, qcm_obj.ms.mass, qcm_obj.c, velocity[time], qcm_obj.k, spring_delta, qcm_obj.mu.mass)

#   "Solves" Spring-Damper 2nd Order NH DE
#   Input: input force, total mass, damper coefficient, instantaneous velocity of system, spring stiffness, position, unsprung mass
#   Output: acceleration of unsprung mass 
def qcm_acceleration(f, m_sm, c, vy, k, spring_delta, m_usm):
    return (f + (m_sm + m_usm) * g + c * vy + k * spring_delta) / m_usm

## main ##

for k in [10000, 20000, 30000, 40000]:
    for c in range(11):
        qcm = QCM(80, 15, k, c)
        
        net_a = np.zeros(len(time))
        for i in range(len(time)):
            net_a[i] = step(qcm, accelerations, velocities, positions, i)

        print(np.average(net_a))
    print()
    