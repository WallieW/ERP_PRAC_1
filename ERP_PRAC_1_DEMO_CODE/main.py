import numpy as np
import matplotlib.pyplot as plt
import serial
import math
import random

# variables to change during demo time
sampling_period = 60
simulation_time = 3000
initial_object_location = [10,10]    # in kilometers
initial_object_velocity = [0.02,0]    # in meters/s  (needs to be in decimal values (other wise it is km/s)
process_noise_std = 0.01        # in meters/s^2

# sensor noise parameters
range_error_std = 20            # in meters
range_error_mean = 0            # in meters

bearing_error_std = 1           # in degrees
bearing_error_mean = 0           # in degrees

# serial communication parameters
# serial_port_number = input("what is the serial port number?  ")
# serial_port_name = "COM"+serial_port_number
# baudrate = 115200


# starts the serial port setup (no opening or closing)
def serial_setup(portname, baudrate):
    serialOBJ = serial.Serial()
    serialOBJ.baudrate = baudrate
    serialOBJ.bytesize = serial.EIGHTBITS
    serialOBJ.parity = serial.PARITY_NONE
    serialOBJ.port = portname

    return serialOBJ

#sends the noisy sensor data to the stm32
def send_serial_data(serial_connection_obj, polar_measurements):
    if not serial_connection_obj.is_open:
        serial_connection_obj.open()

    polar_range,polar_bearing = polar_measurements
    string_data = str(polar_range)+":"+str(polar_bearing)
    serial_connection_obj.write(string_data.encode('utf-8'))

# read data from stm being sent back
def read_serial_data(serial_connection_obj):
    if not serial_connection_obj.is_open:
        serial_connection_obj.open()

    stm_data_read_bytes = serial_connection_obj.read()
    stm_read_data = stm_data_read_bytes.decode('utf-8')

    return stm_read_data

# converts the cartesian coordinates to polar coordinates (degrees)
def convert_to_polar(x,y):
    converted_range = np.sqrt(x**2 + y**2)
    converted_bearing_radians = np.atan2(y,x)
    converted_bearing_degrees = converted_bearing_radians * (180/math.pi)

    return converted_range, converted_bearing_degrees

# generate bearing, range sensor noise
def generate_sensor_noise(size):
    sensor_range_noise = np.random.normal(range_error_mean, range_error_std,size)
    sensor_bearing_noise = np.random.normal(bearing_error_mean,bearing_error_std,size)

    return sensor_range_noise,sensor_bearing_noise

#polar to cartesian plots
def polar_to_cartesian(polar_range, polar_bearing):
    cartesian_x = polar_range*np.cos(np.deg2rad(polar_bearing))
    cartesian_y = polar_range*np.sin(np.deg2rad(polar_bearing))

    return cartesian_x, cartesian_y

# plot the cartesian co-ordinates of the model
def generate_cartesian_plots(cartesian_model, cartesian_embedded, cartesian_python_converted):
    fig, (ax1,ax2,ax3) = plt.subplots(3,1, figsize=(10,8))
    fig.suptitle("Cartesian position plots")

    ax1.plot(cartesian_model[0]/1000 , cartesian_model[1]/1000)
    ax2.plot(cartesian_embedded[0]/1000, cartesian_embedded[1]/1000)
    ax3.plot(cartesian_python_converted[0]/1000, cartesian_python_converted[1]/1000)

    plt.legend()
    plt.show()

# plot the polar co-ordinates of the model
def generate_polar_plots(polar_model, polar_model_noisy):
    fig, (ax1, ax2) = plt.subplots(1,2, subplot_kw={'projection': 'polar'})
    fig.suptitle("Polar plots for model and noisy sensor")
    ax1.plot(np.deg2rad(polar_model[1]), polar_model[0]/1000 , "--")
    ax2.plot(np.deg2rad(polar_model_noisy[1]), polar_model_noisy[0]/1000 , "--")

    plt.legend()
    plt.show()


#model of the object that will be tracked
def polynomial_model(x0, dt, steps, position_noise_std=0.01, velocity_noise_std=0.01, acceleration_noise_std=0.01):
    trajectory = []

    # Generate different noise levels for position, velocity, and acceleration
    process_noise = np.zeros((steps, 6))
    process_noise[:, 0:2] = np.random.normal(0, position_noise_std, (steps, 2))  # Position noise
    process_noise[:, 2:4] = np.random.normal(0, velocity_noise_std, (steps, 2))  # Velocity noise
    process_noise[:, 4:6] = np.random.normal(0, acceleration_noise_std, (steps, 2))  # Acceleration noise

    for i in range(steps):
        # Compute new state based on constant acceleration model
        x = x0[0] + x0[2]
        dt + (x0[4] * dt * 2) / 2
        y = x0[1] + x0[3]
        dt + (x0[5] * dt * 2) / 2
        vx = x0[2] + x0[4] * dt
        vy = x0[3] + x0[5] * dt
        ax = x0[4]
        ay = x0[5]

        # Add noise to each state
        x += process_noise[i, 0]
        y += process_noise[i, 1]
        vx += process_noise[i, 2]  # Accumulate noise in velocity
        vy += process_noise[i, 3]
        ax += process_noise[i, 4]  # Accumulate noise in acceleration
        ay += process_noise[i, 5]

        # Update state
        x0 = [x, y, vx, vy, ax, ay]
        trajectory.append(x0)

    return np.array(trajectory)




# Plot simulated trajectory
# plt.figure(figsize=(8, 6))
# plt.plot(trajectory[:, 0], trajectory[:, 1], marker='o', label='Simulated Path')
# plt.xlabel("X Position")
# plt.ylabel("Y Position")
# plt.title("Polynomial Model Simulation with White Noise")
# plt.legend()
# plt.grid()
# plt.show()


# Function to generate polar measurements with noise
def generate_polar_measurements(x, y, range_noise_std=0.1, bearing_noise_std=0.05):
    target_range = math.sqrt(x ** 2 + y ** 2)
    target_bearing = math.atan2(y, x)  # Returns angle in [-π, π]

    # Generate noise
    range_noise = np.random.normal(0, range_noise_std)
    bearing_noise = np.random.normal(0, bearing_noise_std)

    # Add noise to range and bearing
    sensor_range = max(0, target_range + range_noise)  # Ensure non-negative range
    sensor_bearing = (target_bearing + bearing_noise) % (2 * np.pi)  # Ensure within [0, 2π]

    return sensor_range, sensor_bearing


# # Generate noisy polar measurements from trajectory
# bearings = []
# ranges = []
#
# for i in range(len(trajectory)):
#     x, y = trajectory[i][0], trajectory[i][1]
#     sensor_range, sensor_bearing = generate_polar_measurements(x, y)
#     ranges.append(sensor_range)
#     bearings.append(sensor_bearing)
#
# # Plot the polar measurements correctly
# plt.figure(figsize=(6, 6))
# ax = plt.subplot(111, projection='polar')
# ax.scatter(bearings, ranges, label="Noisy Measurements", color='red', s=10)  # Correct ordering: (θ, r)
# ax.set_theta_zero_location("E")  # Zero degrees (0 rad) points to the right
# ax.set_theta_direction(-1)  # Clockwise angle increase
# ax.set_title("Corrected Range and Bearing Plot")
# plt.legend()
# plt.show()









#start from here.
#1) polynomial_model()
#2) cartesian to polar, with added noise
#3)



#sets the initial state
# Initial state [x, y, vx, vy, ax, ay]
x0 = [initial_object_location[0]*1000, initial_object_location[1]*1000, initial_object_velocity[0], initial_object_velocity[1], np.random.normal(0,process_noise_std), np.random.normal(0,process_noise_std)]
# x0 = [0, 0, random.uniform(0, 10), random.uniform(0, 10), random.uniform(-0.2, 0.2), random.uniform(-0.2, 0.2)]
dt = 0.1 # Time step
steps = 3000  # Number of steps


# Simulate trajectory
trajectory = polynomial_model(x0, dt, steps)

model_x_coordinate =  trajectory[:,0]
model_y_coordinate = trajectory[:,1]
model_x_velocity = trajectory[:,2]
model_y_velocity = trajectory[:,3]
model_x_acc = trajectory[:,4]
model_y_acc = trajectory[:,5]

# cartesian_plot_1 = [model_x_coordinate, model_y_coordinate]
# cartesian_plot_2 = [model_x_velocity, model_y_velocity]
# cartesian_plot_3 = [model_x_acc, model_y_acc]


# polar conversions
model_polar_range, model_polar_bearing = convert_to_polar(model_x_coordinate , model_y_coordinate)
polar_range_noise_sensor, polar_bearing_noise_sensor = generate_sensor_noise(model_polar_range.size)
polar_range_noisy = model_polar_range + polar_range_noise_sensor                # polar range measurements to send to the stm32f
polar_bearing_noisy = (model_polar_bearing + polar_bearing_noise_sensor)%360    # polar bearing measurements to send to the stm32f

# polar_plot_1 = [model_polar_range, model_polar_bearing]
# polar_plot_2 = [polar_range_noisy, polar_bearing_noisy]

# python polar to cartesian conversion and plotting of the error:
noisy_cartesian_x , noisy_cartesian_y  = polar_to_cartesian(polar_range_noisy, polar_bearing_noisy)
cartesian_plot_3 = [noisy_cartesian_x, noisy_cartesian_y]


# gather the sample data
cartesian_normal_samples = []
cartesian_embedded_samples = []
cartesian_python_converted_samples = []

polar_non_noisy_samples = []
polar_noisy_samples = []

for i in range(0,steps, sampling_period):
    cartesian_normal_samples.append([float(model_x_coordinate[i]),float(model_y_coordinate[i])])
    cartesian_python_converted_samples.append([float(noisy_cartesian_x[i]) , float(noisy_cartesian_y[i])])
    polar_non_noisy_samples.append([float(model_polar_range[i]) , float(model_polar_bearing[i])])
    polar_noisy_samples.append([float(polar_range_noisy[i]) , float(polar_bearing_noisy[i])])


cartesian_sample_normal_x = [i[0] for i in cartesian_normal_samples]
cartesian_sample_normal_y = [i[1] for i in cartesian_normal_samples]

cartesian_python_converted_x = [i[0] for i in cartesian_python_converted_samples]
cartesian_python_converted_y = [i[1] for i in cartesian_python_converted_samples]

polar_non_noisy_x = [i[0] for i in polar_non_noisy_samples]
polar_non_noisy_y = [i[1] for i in polar_non_noisy_samples]

polar_noisy_x = [i[0] for i in polar_noisy_samples]
polar_noisy_y = [i[1] for i in polar_noisy_samples]



generate_cartesian_plots(np.array([cartesian_sample_normal_x , cartesian_sample_normal_y]),
                         np.array([0,0]),
                         np.array([cartesian_python_converted_x,cartesian_python_converted_y]))

generate_polar_plots(np.array([polar_non_noisy_x,polar_non_noisy_y]),
                     np.array([polar_noisy_x,polar_noisy_y]))

# send the sampled data to the stm32 through the com port


