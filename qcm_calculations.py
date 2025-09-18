import numpy as np
import qcm_types
from constants import *

# qcm_calculations.py
def qcm_dstate(state, vars, constants, t):
    x1, x2, v1, v2 = state[:]
    distance_road, vel_road, disp_road, time_array = vars[:]
    m1, m2, k1, k2, c1, c2, l1_0, l2_0 = constants[:]
    
    #interpolate for these values
    vroad = np.interp(t,time_array, vel_road)
    droad = np.interp(t,time_array, disp_road)
    distance = np.interp(t,time_array, distance_road)

    dstate = qcm_calcs(x1, x2, v1, v2, l1_0, l2_0, k1, k2, c1, c2, m1, m2, vroad, droad, distance)[0]
    return dstate

def qcm_calcs_car(car: qcm_types.QuarterCarModel,vroad, droad, distance):
    #current spring lengths
    l1 = car.mu.x - droad
    l2 = car.ms.x - car.mu.x

    #set spring lengths to max uncompressed to prevent extension into tension
    if l1 > car.k1.l_0:
        l1 = car.k1.l_0
    if l2 > car.k2.l_0:
        l2 = car.k2.l_0

    #Spring Forces
    Fs1 = -car.k1.k*(l1 - car.k1.l_0)
    Fs2 = -car.k2.k*(l2 - car.k2.l_0)

    #Damper forces
    Fd1 = (car.mu.vy - vroad)*car.c1.c
    Fd2 = (car.ms.vy - car.mu.vy)*car.c2.c
    if car.mu.x > (droad + car.k1.l_0): #If QQM is off the ground, set bottom damper to zero
        Fd1 = 0

    #Net forces
    F1 = Fs1 - Fd1
    F2 = Fs2 - Fd2

    # Calculate Derivatives of the State
    dx1 = car.mu.vy                        #v1
    dx2 = car.ms.vy                        #v2
    dv1 = (F1-F2 - car.mu.mass*g) / car.mu.mass       #a1
    dv2 = (F2 - car.ms.mass*g) / car.ms.mass          #a2

    dstate = [dx1, dx2, dv1, dv2]

    return [dstate, dv1, dv2, l1, l2, Fs1, Fs2, Fd1, Fd2]

def qcm_calcs(x1, x2, v1, v2, l1_0, l2_0, k1, k2, c1, c2, m1, m2, vroad, droad, distance):
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

    # Calculate Derivatives of the State
    dx1 = v1                        #v1
    dx2 = v2                        #v2
    dv1 = (F1-F2 - m1* g) / m1       #a1
    dv2 = (F2 - m2* g) / m2          #a2

    dstate = [dx1, dx2, dv1, dv2]

    return [dstate, dv1, dv2, l1, l2, Fs1, Fs2, Fd1, Fd2]
