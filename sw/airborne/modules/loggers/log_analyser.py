import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

folder = "2.2Testing_hoek_van_holland"
folder_path = os.path.join("/home", "chris", "paparazzi", "chris_data", "file_logger", folder)
folder_path = os.path.join(folder_path, "2")
file = "2.csv"
file_path =  os.path.join(folder_path, file)

with open(file_path, "r") as f:
    df = pd.read_csv(file_path)

# Scaled sensors
gyro_scaled = df[["gyro_p", "gyro_q", "gyro_r"]]
accel_scaled = df[["accel_x", "accel_y", "accel_z"]]
mag_scaled = df[["mag_x", "mag_y", "mag_z"]]

# Gps states
gps_states = df[["GPS state aircraft"]]

# Airspeed
airspeed = df[["airspeed"]]

# Wind
windspeed = df[["wind->x", "wind->y", "wind->z"]]

# Aileron setpoint
aileron = df[["h_ctl_aileron_setpoint"]]

# Elevator setpoint
elevator = df[["h_ctl_elevator_setpoint"]]

# Roll setpoint follow me 
roll_gain = df[["h_ctl_roll_setpoint_follow_me","follow_me_roll"]]

# Dist wp follow 
distance_follow = df[["dist_wp_follow.x", "dist_wp_follow.y", "dist_wp_follow.z", "follow_me_heading"]]

# Ground utm 
ground_pos = df[["ground_utm.east", "ground_utm.north", "ground_utm.alt"]]

# Disco pos 
disco_pos = df[["pos_Utm->east", "pos_Utm->north", "pos_Utm->alt"]]

# Pos east
pos_east = df[["ground_utm.east", "pos_Utm->east"]]

# Post north
pos_north = df[["ground_utm.north", "pos_Utm->north"]]

# Pos alt
pos_alt = df[["ground_utm.alt", "pos_Utm->alt"]]

# Gain Airspeed
gain_air = df[["dist_wp_follow.y", "v_ctl_auto_airspeed_setpoint", "ap_mode"]]

# apmode 
ap_mode = df[["ap_mode"]].values

# Commands 
radio = df[["radio_pitch", "radio_roll", "radio_roll"]]

# Stationary Ground 
stationary = df["stationary_ground"]

# Random 
random = df[["ap_mode", "pos_Utm->alt","accel_x" ]]

# Attitude 
attitude = df[["roll","yaw","theta"]]

# Attitude plot
fig = plt.figure()
attitude.plot()
plt.savefig(os.path.join(folder_path, "Attitude.eps"), format="eps")
plt.show()

# Stationary ground plot 
fig = plt.figure()
stationary.plot()
plt.savefig(os.path.join(folder_path, "Stationary.eps"), format="eps")
plt.close()

# Radio plot 
# fig = plt.figure()
# radio.plot()
# plt.savefig(os.path.join(folder_path, "Radio.eps"), format="eps")
# plt.close()

# Random plot 
fig = plt.figure()
random.plot()
plt.savefig(os.path.join(folder_path, "Random.eps"), format="eps")
plt.close()


# Position colored with magnetic heading difference
fig = plt.figure().gca(projection='3d')
relative_pos = disco_pos - ground_pos.values
magnetic_heading = np.arctan2(df[["mag_x"]].values, df[["mag_y"]].values) * 180/np.pi
magnetic_heading += 180 
relative_pos['magnetic_heading'] = magnetic_heading
img = fig.scatter(relative_pos["pos_Utm->east"].values, relative_pos["pos_Utm->north"].values, relative_pos["pos_Utm->alt"].values, c=relative_pos["magnetic_heading"].values, cmap="twilight")
fig.set_xlabel('East')
fig.set_ylabel('North')
fig.set_zlabel('Alt')
plt.colorbar(img)
plt.savefig(os.path.join(folder_path, "mag_pos.eps"), format="eps") 
plt.close()

plt.figure()
heading = df[["follow_me_heading","ap_mode"]]
heading.plot()
plt.savefig(os.path.join(folder_path,"heading.eps"), format="eps")
plt.close()

plt.figure()
pos_alt.plot()
plt.savefig(os.path.join(folder_path, "pos_alt.eps"), format="eps")
plt.close()

plt.figure()
pos_north.plot()
plt.savefig(os.path.join(folder_path, "pos_north.eps"),format="eps")
plt.close()

plt.figure()
pos_east.plot()
plt.savefig(os.path.join(folder_path, "pos_east.eps"), format="eps")
plt.close()

plt.figure()
gyro_scaled.plot()
plt.savefig(os.path.join(folder_path, "gyro_scaled.eps"),format="eps")
plt.close()

plt.figure()
accel_scaled.plot()
plt.savefig(os.path.join(folder_path, "accel_scaled.eps"),format="eps")
plt.close()

plt.figure()
mag_scaled.plot()
plt.savefig(os.path.join(folder_path, "mag_scaled.eps"), format="eps")
plt.close()

plt.figure()
gps_states.plot()
plt.savefig(os.path.join(folder_path, "gps_states.eps"),format="eps")
plt.close()

plt.figure()
airspeed.plot()
plt.savefig(os.path.join(folder_path, "airspeed.eps"),format="eps")
plt.close()

plt.figure()
windspeed.plot()
plt.savefig(os.path.join(folder_path, "windspeed.eps"),format="eps")
plt.close()

plt.figure()
aileron.plot()
plt.savefig(os.path.join(folder_path, "aileron_setpoint.eps"), format="eps")
plt.close()

plt.figure()
altitude_values = df["pos_Utm->alt"].values
altitude_values[np.nan_to_num(altitude_values) > 10] += 2000
elevator["pos.alt"] = altitude_values
elevator.plot()
plt.savefig(os.path.join(folder_path, "elevator_setpoint.eps"), format="eps")
plt.close()

plt.figure()
roll_gain.plot()
plt.savefig(os.path.join(folder_path, "roll_gain.eps"), format="eps")
plt.close()

plt.figure()
distance_follow.plot()
plt.savefig(os.path.join(folder_path, "follow_me_distance.eps"), format="eps")
plt.close()

plt.figure()
gain_air.plot()
plt.savefig(os.path.join(folder_path, "gain_air.eps"),format="eps")
plt.close()





