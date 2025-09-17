import math
import numpy as np
import matplotlib.pyplot as plt


#road profiles need to have equally spaced step sizes
#inputs are road array (floats), and floats all in meters besides the order (dimensionless int)
#outputs a road profile with a trapezoid added

def trapezoid(road_positions, step_start, step_distance, peak_height, peak_distance, order):
    output_profile = np.zeros(len(road_positions))
    step_size = road_positions[1]-road_positions[0]

    distance = 0
    for i in range(len(road_positions)):    
        distance = road_positions[i]
        ramp_slope = peak_height / step_distance

        if distance >= step_start and distance < (step_start + step_distance):
            output_profile[i] = (distance - step_start)* ramp_slope #ramp up(i)

        elif distance >= (step_start + step_distance) and distance < (step_start + step_distance + peak_distance):
            output_profile[i] = peak_height

        elif distance >= (step_start + step_distance + peak_distance) and distance < (step_start + 2*step_distance + peak_distance):
            output_profile[i] = - (distance - (step_start + 2*step_distance + peak_distance))* ramp_slope #ramp down(i)

    if order == 1:
        return output_profile #displacement profile
    elif order == 2:
        vel_profile = np.gradient(output_profile, road_positions)
        return vel_profile #velocity profile
    elif order == 3:
        vel_profile = np.gradient(output_profile, road_positions)
        accel_profile = np.gradient(vel_profile, road_positions)
        return accel_profile #accel profile
    else:
        print("error: only order 1, 2, or 3 accepted as an input for disp., velocity, or accel. profiles")
        return -1

'''

#debug plot of trapezoid
road_length = 10 #meters
road_array = np.linspace(0,road_length,1000)

road_height_zero = np.zeros(len(road_array))
road_height_disp = road_height_zero + trapezoid((road_array), 1, 1, 0.2, 1, 1)
road_height_vel = road_height_zero + trapezoid((road_array), 1, 1, 0.2, 1, 2)
road_height_accel = road_height_zero + trapezoid((road_array), 1, 1, 0.2, 1, 3)

# Plot road height and zero height
plt.figure(figsize=(10, 6))
plt.plot(road_array, road_height_zero, label="Zero Height", linestyle="--", color="gray")
plt.plot(road_array, road_height_disp, label="Road Height (Trapezoid)", color="blue")
plt.plot(road_array, road_height_vel, label="Road Height Vel. (Trapezoid)", color="green")
plt.plot(road_array, road_height_accel, label="Road Height Accel. (Trapezoid)", color="red")

plt.xlabel("Road Position")
plt.ylabel("Height")
plt.title("Road Height and Zero Height Profile")
plt.legend()
plt.grid(True)
plt.show()

'''