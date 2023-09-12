import pyulog
import matplotlib.pyplot as plt
plt.rcParams["font.family"] = "serif"
# Specify the path to your ULG file
# ulg_file_path = '/home/zihan/PX4-Autopilot/integrationtests/python_src/px4_it/mavros/911d5014-b4e2-4400-810b-57625a4499cf.ulg'
ulg_file_path = '911d5014-b4e2-4400-810b-57625a4499cf.ulg'

# Load the ULG file
log = pyulog.ULog(ulg_file_path)

# Extract the distance sensor data
distance_sensor_data = log.get_dataset('distance_sensor')
print(distance_sensor_data)
#############################################################################
# Extract the baro sensor data
baro_sensor_data = log.get_dataset('vehicle_air_data')
baros = baro_sensor_data.data['baro_alt_meter']/10
timestamps_baro = baro_sensor_data.data['timestamp_sample'] / 1e6
# print(distance_sensor_data.list_value_changes('current_distance'))

timestamps = distance_sensor_data.data['timestamp'] / 1e6
print("length of the datat:", len(timestamps))
distances = distance_sensor_data.data['current_distance']


# Extract the GPS local sensor data
GPS_sensor_data = log.get_dataset('vehicle_local_position')
GPS_x = GPS_sensor_data.data['x']
GPS_y = GPS_sensor_data.data['y']
GPS_z = -1*GPS_sensor_data.data['z']
timestamps_GPS = GPS_sensor_data.data['timestamp_sample'] / 1e6

# Extract the GPS local sensor data
vehicle_global_position_data = log.get_dataset('vehicle_global_position')
vehicle_global_position_GPS_lon = vehicle_global_position_data.data['lon']
vehicle_global_position_GPS_lat = vehicle_global_position_data.data['lat']
vehicle_global_position_GPS_alt = vehicle_global_position_data.data['alt']

print("vehicle_global_position_GPS_lon:", vehicle_global_position_GPS_lon)


vehicle_global_position_timestamps_GPS = vehicle_global_position_data.data['timestamp_sample'] / 1e6

# Extract the sensor GPS local sensor data
sGPS_sensor_data = log.get_dataset('sensor_gps')
lon = sGPS_sensor_data.data['lon']
lat = sGPS_sensor_data.data['lat']
alt = sGPS_sensor_data.data['alt']
timestamps_sGPS = sGPS_sensor_data.data['time_utc_usec'] / 1e6


# Extract the sensor GPS local sensor data
vehicle_gps_position_data = log.get_dataset('vehicle_gps_position')
vehicle_gps_position_lon = vehicle_gps_position_data.data['lon']
vehicle_gps_position_lat = vehicle_gps_position_data.data['lat']
vehicle_gps_position_alt = vehicle_gps_position_data.data['alt']
vehicle_gps_position_timestamps_sGPS = vehicle_gps_position_data.data['time_utc_usec'] / 1e6
#############################################################################



crop_data = 0
timestamps = timestamps[crop_data:]
distances = distances[crop_data:]
# # Extract timestamps and distances
# timestamps = [record.timestamp / 1e6 for record in distance_sensor_data.data]
# distances = [record.distance for record in distance_sensor_data.data]

# # Extract timestamps and distances
# timestamps = [record['timestamp'] / 1e6 for record in distance_sensor_data.data]
# distances = [record['current_distance'] for record in distance_sensor_data.data]


# Specify the dataset name
dataset_name = 'distance_sensor'




#
# # Iterate through the log and extract distance sensor data
# for entry in log:
#     if entry.name == 'distance_sensor':
#         timestamps.append(entry.timestamp / 1e6)  # Convert to seconds
#         distances.append(entry.data['current_distance'])
#
#
# # # ['timestamp', 'device_id', 'min_distance', 'max_distance', 'current_distance', 'variance', 'h_fov', 'v_fov', 'q[0]', 'q[1]', 'q[2]', 'q[3]', 'signal_quality', 'type', 'orientation']
#
# Create the plot
plt.figure(figsize=(10, 6))
plt.plot(timestamps, distances, label='Distance Sensor')
# plt.plot(timestamps_baro, baros, label='Barometer')
plt.plot(timestamps_GPS, GPS_z, label='vehicle_local_position z')
# ax2 = plt.twinx()
# ax2.plot(timestamps_baro, alt, label='sensor_gps z')


plt.xlabel('Time [seconds]')
plt.ylabel('Height and Distance [meters]')
plt.title('Z')
plt.legend()
plt.grid(True)
plt.tight_layout()  # Adjust subplot spacing
##########################################

plt.figure(figsize=(10, 6))

plt.plot(GPS_x, GPS_y, label='GPS X Y')
plt.xlabel('X [meters]')
plt.ylabel('Y [meters]')
plt.title('XY')
plt.legend()
plt.grid(True)
plt.tight_layout()  # Adjust subplot spacing

##########################################

plt.figure(figsize=(10, 6))

plt.plot(lon, lat, label='GPS sensor')
plt.plot(vehicle_gps_position_lon, vehicle_gps_position_lat, label='vehicle_gps_position')
plt.xlabel('lon')
plt.ylabel('lat')
plt.title('GPS sensor')
plt.legend()
plt.grid(True)
plt.tight_layout()  # Adjust subplot spacing

plt.figure(figsize=(10, 6))

plt.plot(timestamps_sGPS, alt, label='GPS sensor z')
plt.xlabel('time')
plt.ylabel('alt')
plt.title('GPS sensor')
plt.legend()
plt.grid(True)
plt.tight_layout()  # Adjust subplot spacing


plt.figure(figsize=(10, 6))
plt.plot(vehicle_global_position_GPS_lon, vehicle_global_position_GPS_lat, label='vehicle_gps_position')
plt.xlabel('lon')
plt.ylabel('lat')
plt.title('vehicle_global_position')
plt.legend()
plt.grid(True)
plt.tight_layout()  # Adjust subplot spacing





############

# lon_len: 297
# vehicle_gps_position_len: 592
# timestamps_len: 298
# timestamps_GPS_len: 2972


crop_data = 100 #298
timestamps_len =  len(timestamps)

crop_percent = crop_data/timestamps_len

crop_lon = int(len(lon)*crop_percent)
crop_vehicle_gps_position_lon = int(len(vehicle_gps_position_lon)*crop_percent)
crop_timestamps = int(len(timestamps)*crop_percent)
crop_timestamps_GPS = int(len(timestamps_GPS)*crop_percent)



############
plt.figure(figsize=(8, 4))
plt.subplot(1, 2, 1)

print("lon_len:",len(lon))
print("vehicle_gps_position_len:",len(vehicle_gps_position_lon))

plt.plot(lon[crop_lon:], lat[crop_lon:], label='GPS sensor')
plt.plot(vehicle_gps_position_lon[crop_vehicle_gps_position_lon:], vehicle_gps_position_lat[crop_vehicle_gps_position_lon:], label='vehicle_gps_position')
plt.xlabel('lon')
plt.ylabel('lat')
plt.title('x-y-Position')
plt.legend()
plt.grid(True)
plt.tight_layout()  # Adjust subplot spacing






plt.subplot(1, 2, 2)

print("timestamps_len:",len(timestamps))
print("timestamps_GPS_len:",len(timestamps_GPS))

plt.plot(timestamps[crop_timestamps:], distances[crop_timestamps:], label='Distance Sensor')
# plt.plot(timestamps_baro, baros, label='Barometer')
plt.plot(timestamps_GPS[crop_timestamps_GPS:], GPS_z[crop_timestamps_GPS:], label='vehicle_local_position z')
# ax2 = plt.twinx()
# ax2.plot(timestamps_baro, alt, label='sensor_gps z')

plt.xlabel('Time [seconds]')
plt.ylabel('Height and Distance [meters]')
plt.title('z-Position')
plt.legend()
plt.grid(True)
plt.tight_layout()  # Adjust subplot spacing


##########################################
# Show the plot
plt.show()