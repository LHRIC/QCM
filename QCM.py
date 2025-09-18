import math 
from matplotlib import cm
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
    def __init__(self, m: float, x_0: float):
        self.mass = m
        self.x = x_0
        self.vy = 0
        self.ay = -g
class QCM:
    def __init__(self, ms: float, mu: float, k: float, c_percentage: float):
        self.l = (ms * g) / k   # length of spring
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

def step(qcm_obj: QCM, acceleration: np.ndarray, velocity: np.ndarray, position: np.ndarray, time: int) -> float:
    spring_delta = qcm_obj.l - max(0, qcm_obj.l - position[time])
    input_force = acceleration[time] * (qcm_obj.mu.mass + qcm_obj.ms.mass)

    return qcm_acceleration(input_force, qcm_obj.ms.mass, qcm_obj.c, velocity[time], qcm_obj.k, spring_delta, qcm_obj.mu.mass)

#   "Solves" Spring-Damper 2nd Order NH DE
#   Input: input force, total mass, damper coefficient, instantaneous velocity of system, spring stiffness, position, unsprung mass
#   Output: acceleration of unsprung mass 
def qcm_acceleration(f: float, m_sm: float, c: float, vy: float, k: float, spring_delta: float, m_usm: float) -> float:
    return (f + ((m_sm + m_usm) * g) + (c * vy) + (k * spring_delta)) / m_usm

## main ##

spring_constants = np.arange(10000, 105000, 5000) # spring const from 10000 to 105000, 5000 increments
damping_percentages = np.arange(0.1, 1.05, 0.05)  #10% to 100%, 10% increments

sprung_mass = 80   #kg
unsprung_mass = 15 #kg

average_a = np.zeros((len(spring_constants), len(damping_percentages)))

for k in spring_constants:
    for c in damping_percentages:
        qcm = QCM(sprung_mass, unsprung_mass, k, c)

        net_a = np.zeros(len(time))
        for i in range(len(time)):
            net_a[i] = step(qcm, accelerations, velocities, positions, i)

        average_a[np.where(spring_constants == k)[0][0], np.where(damping_percentages == c)[0][0]] = np.average(net_a)

fig, axs = plt.subplots(1)
axs.plot(damping_percentages, average_a, 'tab:brown', label='average a v.s. c')
axs.set_xlabel('Damping % of cd')
axs.set_ylabel('Average Acceleration (m/s^2)')
axs.set_title('Average Acceleration vs Damping % of CD')
# fig = plt.figure()
# ax = fig.add_subplot(projection='3d')

# spring_constants, damping_percentages = np.meshgrid(spring_constants, damping_percentages)

# ax.plot_surface(spring_constants, damping_percentages, average_a)
# ax.set_title('SD of A v.s. K & C')
# ax.set_xlabel('Spring Stiffness (N/m)')
# ax.set_ylabel('Damping %')
# ax.set_zlabel("Std. Dev of Acceleration (m/s^2)")

plt.show()