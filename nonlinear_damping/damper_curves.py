import csv
import matplotlib.pyplot as plt
import numpy as np

# damper_curves.py

# List of all damper curve filenames and their corresponding settings
curves = [
    "DSD_11_LS.csv", # 0
    "DSD_11_HS.csv", # 1
    "DSD_12_LS.csv", # 2
    "DSD_12_HS.csv", # 3
    "DSD_13_LS.csv", # 4
    "DSD_13_HS.csv", # 5
    "DSD_21_LS.csv", # 6
    "DSD_21_HS.csv", # 7
    "DSD_22_LS.csv", # 8
    "DSD_22_HS.csv", # 9
    "DSD_23_LS.csv", # 10
    "DSD_23_HS.csv"  # 11
]

settings = [
    "(0-4.3) V-C", # 0
    "(0-4.3) C", 
    "(0-4.3) V-R",
    "(0-4.3) R", 
    "(2-4.3) V-C", # 1
    "(2-4.3) C", 
    "(2-4.3) V-R",
    "(2-4.3) R", 
    "(4-4.3) V-C", # 2
    "(4-4.3) C", 
    "(4-4.3) V-R",
    "(4-4.3) R", 
    "(6-4.3) V-C", # 3
    "(6-4.3) C", 
    "(6-4.3) V-R",
    "(6-4.3) R", 
    "(10-4.3) V-C", # 4
    "(10-4.3) C", 
    "(10-4.3) V-R",
    "(10-4.3) R", 
    "(15-4.3) V-C", # 5
    "(15-4.3) C", 
    "(15-4.3) V-R",
    "(15-4.3) R", 
    "(25-4.3) V-C", # 6
    "(25-4.3) C", 
    "(25-4.3) V-R",
    "(25-4.3) R"
]

# Function to convert list of lists to numpy array and handle empty strings
def fliparr(arr: list[list[str]]) -> np.ndarray:
    newarr = np.ndarray((len(arr[0]), len(arr)))
    for i in range(len(arr)):
        for j in range(len(arr[i])):
            if(arr[i][j] == ''):
                newarr[j][i] = np.nan
            else:
                newarr[j][i] = float(arr[i][j])
    return newarr

# Function to get the damper force based on velocity, curve, and setting
# velocity in m/s, force in N
def force_damper(velocity: float, curve: int = 0, setting: int = 0) -> float:
    # Read the CSV file and convert to numpy array
    with open(f"nonlinear_damping/csv/{curves[curve]}", mode='r') as file:
        reader = csv.reader(file)
        next(reader)  # Skip the header row
        data = [row for row in reader]  
    data = fliparr(data)
    # ^^^ dont read the whole file every time, just once and store it somewhere

    # Select the right curve for cases v > 0 or v < 0
    index_velocity = (4 * setting) if (velocity > 0) else (4 * setting + 2)

    # Interpolate the damper force based on the velocity
    return np.interp(
        abs(velocity * 1000), 
        data[index_velocity], 
        data[index_velocity + 1]
        )

# # Plotting the data
# plt.figure(figsize=(10, 6))
# plt.plot(data[0], data[1], label='Low Speed Compression', color='blue')
# plt.show()