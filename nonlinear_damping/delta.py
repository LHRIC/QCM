from damper_curves import force_damper
import matplotlib.pyplot as plt
import numpy as np
import math

# crit_damp.py
max_velocity = 200 / 1000 # (mm/s)
step = 5 / 1000        # (mm/s)
curve = 2

class Damper:
    def __init__(self, c: float):
        self.c = c              #damper coefficient
    def __init__(self, percentage_critical: float, k: float, m: float):
        self.crit_c = 2 * math.sqrt(k * m)  #critical damper coefficient
        self.c = percentage_critical * (2 * math.sqrt(k * m))  #damper coefficient

    # defualt to DSD_11_LS 0-4.3 V-C curve for now     
    def force(self, velocity: float, curve: int = 0, setting: int = 0) -> float:
        
        return force_damper(velocity, curve, setting)

general_damper = Damper(
    percentage_critical=1,
    k=30000,
    m=80.0
)

velocities = np.arange(0, max_velocity + step, step)
for j in range(0, 7):
    nl_damper = np.zeros(len(velocities))
    l_damper = np.zeros(len(velocities))
    delta = np.zeros(len(velocities))
    for i in range(len(velocities)):
        nl_damper[i] = general_damper.force(velocities[i], curve, j)
        l_damper[i] = general_damper.c * velocities[i]
        delta[i] = abs(nl_damper[i] - l_damper[i])

    # statistics of delta
    print(f"Damper Setting: {j}")
    print(f"Max delta: {np.max(delta)}")
    print(f"Min delta: {np.min(delta)}")
    print(f"Mean delta: {np.mean(delta)}\n")

# plt.plot(velocities, nl_damper, label='Non-Linear Damper (DSD_12_LS)')
# plt.plot(velocities, l_damper, label='Linear Damper (100% Critical)')
# # plt.plot(velocities, delta, label='Difference (Non-Linear - Linear)')
# plt.xlabel('Velocity (mm/s)')
# plt.ylabel('Force (N)')
# plt.title('Damper Force vs Velocity')
# plt.legend()
# plt.grid()
# plt.show()
