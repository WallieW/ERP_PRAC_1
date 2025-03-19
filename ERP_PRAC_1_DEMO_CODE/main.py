import numpy as np
import matplotlib.pyplot as plt
import serial
import math

# variables to change during demo time
sampling_period = 60
simulation_time = 3000
initial_object_location = []    # in meters
initial_object_velocity = []    # in meters/s
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

def serial_setup(portname, baudrate):
    serialOBJ = serial.Serial()

def convert_to_polar(x,y):
    converted_range = math.sqrt(x**2 + y**2)
    converted_bearing_radians = math.atan2(y/x)
    converted_bearing_degrees = converted_bearing_radians * (180/math.pi)

    return converted_range, converted_bearing_degrees













