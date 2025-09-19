import math
import numpy as np
from constants import *
from nonlinear_damping.damper_curves import c_damper

# qcm_types.py
class Mass:
    def __init__(self, m: float, x_0: float = 0, v_0: float = 0):
        self.mass = m
        self.x = x_0
        self.vy = v_0
        self.ay = -g

class Spring:
    def __init__(self, k: float, l_0: float):
        self.k = k              #spring stiffness
        self.l_0 = l_0          #spring free length

#TODO make non-linear LETS GO WE DID IT
class Damper:
    def __init__(self, c: float):
        self.c = c              #damper coefficient
    def __init__(self, percentage_critical: float, k: float, m: float):
        self.c = percentage_critical * (2 * math.sqrt(k * m))  #damper coefficient

    # defualt to DSD_11_LS 0-4.3 V-C curve for now     
    def force(velocity: float, curve: int = 0, setting: int = 0) -> float:      
        return c_damper(velocity, curve, setting)

class QuarterCarModel:
    def __init__(self, mu: float, ms: float,x1_0: float, x2_0: float, k1: float, k2: float, c1_percentage: float, c2_percentage: float):
        self.mu = Mass(m = mu, x_0 = x1_0)          #unsprung mass
        self.ms = Mass(m = ms, x_0 = x2_0)          #sprung mass
        self.k1 = Spring(k = k1, l_0 = (self.mu.x) + g * (self.mu.mass + self.ms.mass) / k1)        #tire spring
        self.k2 = Spring(k = k2, l_0 = (self.ms.x - self.mu.x) + (g * self.ms.mass) / k2)        #suspension spring
        self.c1 = Damper(percentage_critical = c1_percentage, k = self.k1.k, m = self.mu.mass)                 #tire damper
        self.c2 = Damper(percentage_critical = c2_percentage, k = self.k2.k, m = self.ms.mass)                 #suspension damper