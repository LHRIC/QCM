import math 
import numpy as np
import matplotlib.pyplot as plt
import colorednoise as cn
import scipy
from scipy import *

# function makes displacement profile
# inputs: 
# road classification (A - H string)
# road length (meters)
# noise type (displacement, velocity, or acceleration)
def roadProfile(road_positions, road_class: str, order: int):
    '''Inputs:
    Road Class: 'A' - 'H' |
    Road Positions: numpy array of longitudinal positions (meterrs) |
    Order: 1-displacement, 2-velocity, 3-acceleration |
    Outputs: numpy array of road profile values (meters for displacement, m/m for velocity, m/m^2 for acceleration)'''

    #make the psd
    discete_points = 1000
    freq_bounds = [0.01, 10] #cycle/meter

    freq_range = np.linspace((freq_bounds[0]), (freq_bounds[1]), num=discete_points) #cycle/meter
    psd_data = np.linspace(0, 0, num=discete_points) #initialize array to store discete PSD data

    for i in range(len(freq_range)):
        psd_data[i] = Gd('A',freq_range[i])
        psd_data[i] *= np.random.normal(loc=1, scale=0.3) #add noise to the data w/ normal distribution
        psd_data[i] = abs(psd_data[i]) #make sure no negative values

    '''
    #debug plot generated PSD with noise added
    plt.figure(figsize=(10, 6))
    plt.plot(freq_range, psd_data, label=f"Road Class {road_class}", color="blue")
    plt.xscale('log')
    plt.yscale('log')
    plt.xlabel("Frequency (cycle/meter) (Log Scale)")
    plt.ylabel("PSD (Log Scale)")
    plt.title(f"PSD vs Frequency for Road Class {road_class} (Log Scale)")
    plt.legend()
    plt.grid(True)
    plt.show()
    '''
    
    #make the road profile
    #empty road array
    road_height = np.linspace(0, 0, num = len(road_positions)) 

    #random phase for each sin wave
    phase_offsets = np.random.uniform(low = 0, high = road_positions[-1], size=discete_points)
    
    #sum sin waves at each frequency with offset - see paper in confluence
    amplitude = 0
    for j in range(len(psd_data)):
        #wave amplitdue
        amplitude = 0
        if j == 0:
            amplitude = 0
        else:   
            amplitude = math.sqrt(psd_data[j] * ((freq_range[j])/(j)) / math.pi)

        #for zero errors
        if math.isnan(amplitude):
            amplitude = 0
        
        for i in range(len(road_positions)):
            #add each wave across the road length
            road_height[i] += amplitude * math.sin(2*math.pi*freq_range[j]*road_positions[i] - phase_offsets[j])

    '''
    #debug plot FFT of road height for reference    
    fft_vals = scipy.fft.fft(road_height)
    step_size = road_positions[1]-road_positions[0]
    fft_freqs = scipy.fft.fftfreq(len(road_height), d=step_size)
    
    plt.figure(figsize=(10, 6))
    plt.plot(fft_freqs, np.abs(fft_vals), label="FFT Magnitude")
    plt.xscale('log')
    plt.yscale('log')
    plt.xlabel("Frequency (cycle/meter) (Log Scale)")
    plt.ylabel("Magnitude (Log Scale)")
    plt.title("Log-Log FFT of Road Height Profile")
    plt.legend()
    plt.grid(True, which="both", linestyle="--")
    plt.show()
    '''

    if order == 1:
        #z displacement in meters
        return road_height

    elif order == 2:
        #convert to velocity profile  in m/m (meters z vertical)/(meters x road position)
        road_vel = np.gradient(road_height, road_positions)
        return road_vel
    elif order == 3:
        #convert to acceleration profile  in m/m^2 (meters z vertical)/(meters x road position)^2
        road_vel = np.gradient(road_height, road_positions)
        road_accel = np.gradient(road_vel, road_positions)
        return road_accel
    elif order == 0:
        road_vel = np.gradient(road_height, road_positions)
        road_accel = np.gradient(road_vel, road_positions)
        combined = np.stack((road_height, road_vel, road_accel), axis = 0)
        return combined
    else:
        print("error: order must be 1,2, 3, or 0 for displacement, velocity, acceleration, or ALL")
        return -1

def Gd (road_class: str, freq: float):
    #equation from ISO 8608 Section B.5 Fitted PSDs Eqn. B.1 
    #set n0 based on road class
    if road_class == 'A': 
        Gd_n0 = 16e-6
    elif road_class == 'B':
        Gd_n0 = 64e-6      
    elif road_class == 'C':
        Gd_n0 = 256e-6
    elif road_class == 'D':
        Gd_n0 = 1024e-6
    elif road_class == 'E':
        Gd_n0 = 4096e-6
    elif road_class == 'F':
        Gd_n0 = 16384e-6
    elif road_class == 'G':
        Gd_n0 = 65536e-6
    elif road_class == 'H':
        Gd_n0 = 262144e-6
    else:
        print("Error: road class must be A, B, C, D, E, F, G, or H")
        return -1

    w = 2 # exponent used in fitting
    n0 = 0.1 #reference spatial frequency (cycle/meter)
    n = freq #function input in spatial  frequency (cycle/meter)

    return  Gd_n0 * (n/n0)**(-w) #Gd(n) = Gd(n0) * (n/n0)^-w




'''

#Example Usage and debug
road_length = 10 #meters
point_density = 100 #points/meter
road = np.linspace(0, road_length, num=(road_length*point_density)) #road array

road_noise = roadProfile(road,'E', 0) #all 3 profiles
road_disp = road_noise[0]
road_vel = road_noise[1]
road_accel = road_noise[2]

#road_disp = roadProfile(road,'E', 1) #test call to function
#road_vel = roadProfile(road,'E', 2) #test call to function
#road_accel = roadProfile(road, 'E',3) #test call to function

# Plot all three profiles in labeled subplots
fig, axs = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

axs[0].plot(road, road_disp, color='blue')
axs[0].set_ylabel("Displacement (m)")
axs[0].set_title("Road Displacement Profile")

axs[1].plot(road, road_vel, color='green')
axs[1].set_ylabel("Velocity (m/m)")
axs[1].set_title("Road Velocity Profile")

axs[2].plot(road, road_accel, color='red')
axs[2].set_ylabel("Acceleration (m/m²)")
axs[2].set_xlabel("Road Position (m)")
axs[2].set_title("Road Acceleration Profile")

plt.tight_layout()
plt.show()

'''