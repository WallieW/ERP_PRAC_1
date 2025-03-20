import numpy as np
import matplotlib.pyplot as plt
import serial
import math
import random
from scipy.stats import norm,kstest

# variables to change during demo time
sampling_period = 60
simulation_time = 3000
initial_object_location = [10,10]   # in kilometers
initial_object_velocity = [25,25]    # in meters/s  (needs to be in decimal values (other wise it is km/s)
process_noise_std = 0.01        # in meters/s^2

# sensor noise parameters
range_error_std = 20            # in meters
range_error_mean = 0            # in meters

bearing_error_std = 1           # in degrees
bearing_error_mean = 0           # in degrees

# serial communication parameters
serial_port_number = input("what is the serial port number?  ")
serial_port_name = "COM"+serial_port_number
baudrate = 115200


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
    string_data = polar_range+","+polar_bearing+"\n"
    print("string data being sent = ", string_data)
    serial_connection_obj.write(string_data.encode('utf-8'))

# read data from stm being sent back
def read_serial_data(serial_connection_obj):
    if not serial_connection_obj.is_open:
        serial_connection_obj.open()

    stm_data_read_bytes = serial_connection_obj.readline()
    stm_read_data = stm_data_read_bytes.decode('utf-8')

    values = stm_read_data.split(',')

    return values

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
def generate_cartesian_plots(cartesian_model, cartesian_embedded, cartesian_velocity, embedded_velocity):
    fig, (ax1,ax2) = plt.subplots(2,1, figsize=(10,8))
    fig.suptitle("Cartesian position plots")

    ax1.plot(cartesian_model[0]/1000 , cartesian_model[1]/1000 , "-*" , color = "red", label= "Target model position (simulated)")
    ax1.plot(cartesian_embedded[0]/1000, cartesian_embedded[1]/1000, "-o" , color = "lightgreen" , alpha = 0.8, label = "Embedded system target position")
    ax1.set_xlabel("X distance from origin (km)")
    ax1.set_ylabel("Y distance from origin (km)")
    ax1.set_title("Target positional distance from origin ")

    time = np.linspace(0, simulation_time , int(simulation_time / sampling_period))
    model_velocity = np.sqrt(cartesian_velocity[0]**2 + cartesian_velocity[1]**2)

    ax2.plot( time[1:], model_velocity[1:] , "-*", color="red", label="Target model velocity (simulated)")
    ax2.plot(time[1:], embedded_velocity[1:], "-o", color="lightgreen", alpha=0.8, label="Embedded system target velocity")
    ax2.set_xlabel("Simulation time (sec)")
    ax2.set_ylabel("Target velocity (m/s)")
    ax2.set_title("Target velocity vs simulation time")

    ax1.grid(True)
    ax2.grid(True)

    ax1.legend()
    ax2.legend()
    plt.show()

# plot the polar co-ordinates of the model
def generate_polar_plots(polar_model, polar_model_noisy):
    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'}, figsize=(8, 6))
    fig.suptitle("Polar Plot for Model and Noisy Sensor")
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1)
    ax.plot(np.deg2rad(polar_model[1]), polar_model[0] / 1000, "-", color="darkblue", label="Model")
    ax.plot(np.deg2rad(polar_model_noisy[1]), polar_model_noisy[0] / 1000, "-*", alpha=0.7, color="lightgreen", label="Noisy Sensor")
    ax.grid(True)
    ax.legend()
    ax.set_ylabel("Range (km)", labelpad=20)
    ax.text(0.5, -0.1, "Angle (degrees)", transform=ax.transAxes, ha="center", va="center")
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
        ax = 0#x0[4]
        ay = 0#x0[5]

        #hi
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

def polynomial_model_deterministic(x0, dt, steps):
    trajectory = []
    for i in range(steps):
        # Compute new state based on constant acceleration model
        x = x0[0] + x0[2]
        dt + (x0[4] * dt * 2) / 2
        y = x0[1] + x0[3]
        dt + (x0[5] * dt * 2) / 2
        vx = x0[2] + x0[4] * dt
        vy = x0[3] + x0[5] * dt
        ax = 0
        ay = 0

        # Update state
        x0 = [x, y, vx, vy, ax, ay]
        trajectory.append(x0)
    return np.array(trajectory)

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

#sets the initial state
# Initial state [x, y, vx, vy, ax, ay]
x0 = [initial_object_location[0]*1000, initial_object_location[1]*1000, initial_object_velocity[0], initial_object_velocity[1], np.random.normal(0,process_noise_std), np.random.normal(0,process_noise_std)]
dt = 1 # Time step
steps = simulation_time  # Number of steps


# Simulate trajectory
trajectory = polynomial_model(x0, dt, steps)

model_x_coordinate =  trajectory[:,0]
model_y_coordinate = trajectory[:,1]
model_x_velocity = trajectory[:,2]
model_y_velocity = trajectory[:,3]
model_x_acc = trajectory[:,4]
model_y_acc = trajectory[:,5]

# polar conversions
model_polar_range, model_polar_bearing = convert_to_polar(model_x_coordinate , model_y_coordinate)
polar_range_noise_sensor, polar_bearing_noise_sensor = generate_sensor_noise(model_polar_range.size)
polar_range_noisy = model_polar_range + polar_range_noise_sensor                # polar range measurements to send to the stm32f
polar_bearing_noisy = (model_polar_bearing + polar_bearing_noise_sensor)%360    # polar bearing measurements to send to the stm32f

# python polar to cartesian conversion and plotting of the error:
noisy_cartesian_x , noisy_cartesian_y  = polar_to_cartesian(polar_range_noisy, polar_bearing_noisy)
cartesian_plot_3 = [noisy_cartesian_x, noisy_cartesian_y]

# gather the sample data
cartesian_normal_samples = []
cartesian_normal_velocity_samples = []

cartesian_embedded_samples = []
cartesian_python_converted_samples = []

polar_non_noisy_samples = []
polar_noisy_samples = []

for i in range(0,steps, sampling_period):
    cartesian_normal_samples.append([float(model_x_coordinate[i]),float(model_y_coordinate[i])])
    cartesian_python_converted_samples.append([float(noisy_cartesian_x[i]) , float(noisy_cartesian_y[i])])
    polar_non_noisy_samples.append([float(model_polar_range[i]) , float(model_polar_bearing[i])])
    polar_noisy_samples.append([float(polar_range_noisy[i]) , float(polar_bearing_noisy[i])])
    cartesian_normal_velocity_samples.append([float(model_x_velocity[i]) ,  float(model_y_velocity[i])])


cartesian_sample_normal_x = [i[0] for i in cartesian_normal_samples]
cartesian_sample_normal_y = [i[1] for i in cartesian_normal_samples]

cartesian_sample_normal_x_velocity = [i[0] for i in cartesian_normal_velocity_samples]
cartesian_sample_normal_y_velocity = [i[1] for i in cartesian_normal_velocity_samples]

cartesian_python_converted_x = [i[0] for i in cartesian_python_converted_samples]
cartesian_python_converted_y = [i[1] for i in cartesian_python_converted_samples]

polar_non_noisy_x = [i[0] for i in polar_non_noisy_samples]
polar_non_noisy_y = [i[1] for i in polar_non_noisy_samples]

polar_noisy_x = [i[0] for i in polar_noisy_samples]
polar_noisy_y = [i[1] for i in polar_noisy_samples]

# send the sampled data to the stm32 through the com port
embedded_response_x = []
embedded_response_y = []
embedded_response_velocity = []


serial_object =  serial_setup(serial_port_name , baudrate)

max_length_range = 14
max_length_bearing = 6

for i in range(0, int(steps/sampling_period)):

    pre_decimals_range = len(str(int(polar_noisy_x[i])))+1
    decimals_required_range = max_length_range - pre_decimals_range
    formatted_range = f"{polar_noisy_x[i]:.{decimals_required_range}f}"

    pre_decimals_bearing = len(str(int(polar_noisy_y[i])))+1
    decimals_required_bearing = max_length_bearing - pre_decimals_bearing
    formatted_bearing = f"{polar_noisy_y[i]:.{decimals_required_bearing}f}"

    send_serial_data(serial_object, (formatted_range, formatted_bearing))

    serial_data_returned = read_serial_data(serial_object)

    embedded_response_x.append(float(serial_data_returned[0]))
    embedded_response_y.append(float(serial_data_returned[1]))
    embedded_response_velocity.append(float(serial_data_returned[2]))

generate_cartesian_plots(np.array([cartesian_sample_normal_x , cartesian_sample_normal_y]),
                          np.array([embedded_response_x,embedded_response_y]),
                          np.array([cartesian_sample_normal_x_velocity, cartesian_sample_normal_y_velocity]),
                          np.array(embedded_response_velocity))

generate_polar_plots(np.array([polar_non_noisy_x,polar_non_noisy_y]),
                     np.array([polar_noisy_x,polar_noisy_y]))






def verify_target_model():
    x_position = [initial_object_location[0]*1000]
    y_position = [initial_object_location[1]*1000]

    x0 = [initial_object_location[0]*1000 ,initial_object_location[1]*1000,initial_object_velocity[0], initial_object_velocity[1], 0, 0 ]
    trajectory = polynomial_model_deterministic(x0, dt, steps)

    cartesian_deterministic_x = trajectory[:,0]
    cartesian_deterministic_y = trajectory[:,1]

    cartesian_deterministic_x_sampled = [initial_object_location[0]*1000]
    cartesian_deterministic_y_sampled = [initial_object_location[1]*1000]

    for i in range (0, int(simulation_time/sampling_period)):
        new_x_position = x_position[i] + initial_object_velocity[0]*sampling_period
        x_position.append(new_x_position)
        new_y_position = y_position[i] + initial_object_velocity[1]*sampling_period
        y_position.append(new_y_position)

        cartesian_deterministic_x_sampled.append(cartesian_deterministic_x[i*sampling_period])
        cartesian_deterministic_y_sampled.append(cartesian_deterministic_y[i*sampling_period])

    cartesian_deterministic_x_sampled.append(cartesian_deterministic_x[-1])
    cartesian_deterministic_y_sampled.append(cartesian_deterministic_y[-1])

    plt.plot(np.array(x_position)/1000 , np.array(y_position)/1000, "-o" , color = "red", label= "Movement equation model")
    plt.plot(np.array(cartesian_deterministic_x_sampled)/1000, np.array(cartesian_deterministic_y_sampled)/1000 , "-*" , color = "lightgreen", alpha = 0.9, label = "Deterministic target model")
    plt.legend()
    plt.title("Target trajectory position comparison (deterministic(polynomial) vs movement equations)")
    plt.xlabel("X Position from origin (km)")
    plt.ylabel("Y Position from origin (km)")
    plt.grid(True)
    plt.show()

    return cartesian_deterministic_x_sampled,cartesian_deterministic_y_sampled,x_position,y_position

def verfiy_noise_distrubtion(noise_range, noise_bearing):
    #uncomment if more samples required to show better normal dist
    # noise_samples = 10000
    # noise_range, noise_bearing = generate_sensor_noise(noise_samples)

    fig, (ax1, ax2) = plt.subplots(1,2 , figsize = (10,6))

    #genertaes bins into which the range values falls for the histogram
    bins_for_range = np.linspace(noise_range.min(), noise_range.max(), 100)
    #displays the histogram of the actual noise that was generated
    ax1.hist(noise_range, bins=bins_for_range, density=True, label="Range noise histogram")

    # x values to throw into a normal pdf
    x_range = np.linspace(noise_range.min(), noise_range.max(), 1000)
    #calculates the output of a normal pdf with the same mean and std to verify against generated noise
    pdf_range = norm.pdf(x_range, loc=range_error_mean, scale=range_error_std)
    #plots the theoretical pdf
    ax1.plot(x_range, pdf_range, 'r', linewidth=2, label='Theoretical PDF')
    ax1.set_title('Range Noise Distribution')
    ax1.set_xlabel('Range Noise (meters)')
    ax1.set_ylabel('Probability Density')
    ax1.legend()


    #does the same as range but now for bearing valuess
    bins_bearing = np.linspace(noise_bearing.min(), noise_bearing.max(), 50)
    ax2.hist(noise_bearing, bins=bins_bearing, density=True, label='Bearing noise Histogram')

    # same for range as for bearing
    x_bearing = np.linspace(noise_bearing.min(), noise_bearing.max(), 1000)
    pdf_bearing = norm.pdf(x_bearing, loc=bearing_error_mean, scale=bearing_error_std)
    ax2.plot(x_bearing, pdf_bearing, 'r', linewidth=2, label='Theoretical PDF')
    ax2.set_title('Bearing Noise Distribution')
    ax2.set_xlabel('Bearing Noise (degrees)')
    ax2.set_ylabel('Probability Density')
    ax2.legend()

    range_noise_mean = np.mean(noise_range)
    range_noise_std = np.std(noise_range)

    print("this is the mean of the range noise: ", range_noise_mean, " this is the std: ", range_noise_std)

    bearing_noise_mean = np.mean(noise_bearing)
    bearing_noise_std = np.std(noise_bearing)
    print("this is the mean of the bearing noise: ", bearing_noise_mean, " this is the std: ", bearing_noise_std)

    ax1.grid(True)
    ax2.grid(True)
    plt.show()

def non_linearity_of_conversion(number_of_samples = 15000,range_mean = 5,range_standard_deviation = 2 ,theta_mean = 0,theta_standard_deviation = 0.3  ):
# # Step 1: Generate polar measurements (range, bearing) from a Gaussian distribution
# number_of_samples = 15000

# # Parameters for the Gaussian distributions
# range_mean = 5  # Mean of the range (meters)
# range_standard_deviation = 2  # Standard deviation of the range (meters)
# theta_mean = 0  # Mean of the bearing (radians)
# theta_standard_deviation = 0.3  # Standard deviation of the bearing (radians)

    # range and bearing samples from Gaussian distribution
    r = np.random.normal(range_mean, range_standard_deviation, number_of_samples)
    theta = np.random.normal(theta_mean, theta_standard_deviation, number_of_samples)

    # Step 2: Convert polar coordinates to Cartesian coordinates
    x = r * np.cos(theta)
    y = r * np.sin(theta)

    # Step 3: Visualize the data in polar and Cartesian coordinates
    plt.figure(figsize=(15, 8))  # Larger figure size
    # Polar plot
    ax1 = plt.subplot(1, 3, 1, projection='polar')
    ax1.scatter(theta, r, c='blue', s=1)
    ax1.set_title("Polar Coordinates")
    ax1.set_xlabel("Bearing (degrees)")
    ax1.set_ylabel("Range (m)")
    ax1.grid(True)

    # Polar plot (second version, scatter)
    ax2 = plt.subplot(1, 3, 2)
    ax2.scatter(theta, r, c='blue', s=1)
    ax2.set_title("Polar Coordinates on Cartesian plane (Scatter Plot)")
    ax2.set_xlabel("Bearing (radians)")
    ax2.set_ylabel("Range (m)")
    ax2.grid(True)

    # Cartesian plot
    ax3 = plt.subplot(1, 3, 3)
    ax3.scatter(x, y, c='red', s=1)
    ax3.set_title("Cartesian Coordinates")
    ax3.set_xlabel("X coordinates")
    ax3.set_ylabel("Y coordinates")
    ax3.grid(True)

    # Adjust layout to prevent overlapping
    plt.tight_layout()

    # Show all the plots
    plt.show()

def calculate_stats(non_det_x, non_det_y, det_x, det_y, move_x, move_y, embedded_v, model_v, embedded_x, embedded_y):
    error_non_det_x = non_det_x - det_x[:-2]
    error_non_det_y = non_det_y - det_y[:-2]

    error_move_x = move_x - det_x[:-1]
    error_move_y = move_y - det_y[:-1]

    error_non_det_move_x = non_det_x - move_x[:-1]
    error_non_det_move_y = non_det_y - move_y[:-1]

    time = np.linspace(0, simulation_time, int(simulation_time / sampling_period))
    model_velocity = np.sqrt(model_v[0] ** 2 + model_v[1] ** 2)

    velocity_difference = embedded_v[1:] - model_velocity[1:]

    conversion_x_diff = embedded_x - non_det_x
    conversion_y_diff = embedded_y - non_det_y

    range_diff = np.sqrt(conversion_x_diff**2 + conversion_y_diff**2)

    fig, axes = plt.subplots(1, 3, figsize=(18, 6))

    axes[2].scatter(time[1:], velocity_difference, color='blue', label='Velocity difference')
    axes[2].set_title('Difference of model velocity vs embedded velocity')
    axes[2].set_xlabel('Time')
    axes[2].set_ylabel('Velocity (m/s)')
    axes[2].legend()
    axes[2].grid(True)

    axes[1].scatter(time, range_diff, color='green', label='Prediction error')
    axes[1].set_title('Converted measurement vs real data')
    axes[1].set_xlabel('Time')
    axes[1].set_ylabel('Distance (m)')
    axes[1].legend()
    axes[1].grid(True)

    axes[0].scatter(error_non_det_move_x, error_non_det_move_y, color='red', label='Non-Det vs Move')
    axes[0].set_title('Error: Non-Det vs Move')
    axes[0].set_xlabel('Error in X')
    axes[0].set_ylabel('Error in Y')
    axes[0].legend()
    axes[0].grid(True)

    rmse_non_det_x = np.sqrt(np.mean((error_non_det_x) ** 2))
    rmse_non_det_y = np.sqrt(np.mean((error_non_det_y) ** 2))
    rmse_move_x = np.sqrt(np.mean((error_move_x) ** 2))
    rmse_move_y = np.sqrt(np.mean((error_move_y) ** 2))
    rmse_non_det_move_x = np.sqrt(np.mean((error_non_det_move_x) ** 2))
    rmse_non_det_move_y = np.sqrt(np.mean((error_non_det_move_y) ** 2))

    print("RMSE Non-Det X:", rmse_non_det_x)
    print("RMSE Non-Det Y:", rmse_non_det_y)
    print("RMSE Move X:", rmse_move_x)
    print("RMSE Move Y:", rmse_move_y)
    print("RMSE Non-Det vs Move X:", rmse_non_det_move_x)
    print("RMSE Non-Det vs Move Y:", rmse_non_det_move_y)

    rmse_velocity_diff = np.sqrt(np.mean(velocity_difference ** 2))
    rmse_conversion_x_diff = np.sqrt(np.mean(conversion_x_diff ** 2))
    rmse_conversion_y_diff = np.sqrt(np.mean(conversion_y_diff ** 2))
    rmse_range_diff = np.sqrt(np.mean(range_diff ** 2))

    print("RMSE Velocity Difference:", rmse_velocity_diff)
    print("RMSE Conversion X Difference:", rmse_conversion_x_diff)
    print("RMSE Conversion Y Difference:", rmse_conversion_y_diff)
    print("RMSE Range Difference:", rmse_range_diff)

    plt.tight_layout()
    plt.show()




deterministic_x, deterministic_y, movement_x, movement_y = verify_target_model()

verfiy_noise_distrubtion(polar_range_noise_sensor, polar_bearing_noise_sensor)

non_linearity_of_conversion()

calculate_stats(np.array(cartesian_sample_normal_x),np.array(cartesian_sample_normal_y),
                np.array(deterministic_x), np.array(deterministic_y),
                np.array(movement_x), np.array(movement_y),
                np.array(embedded_response_velocity),
                np.array([cartesian_sample_normal_x_velocity, cartesian_sample_normal_y_velocity]),
                np.array(embedded_response_x), np.array(embedded_response_y))

