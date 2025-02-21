import multiprocessing
import ctypes
import time
import math
import zmq
import numpy as np
from VN100 import VN100IMU, IMUData  # Assuming VN100IMU class and IMUData dataclass are available
from RF import I2CSender
from ServoLatch import ServoController
import lgpio
import os
from collections import deque
import signal
import sys

# Threshold values (Flight Use)
# landingAccMagThreshold = 30  # m/s^2
# groundLevel = 150 # CHANGE THIS VALUE TO CALIBRATE IMU 133.34
# initialAltitudeThreshold = groundLevel + 25 # This need to be larger
# landingAltitudeThreshold = groundLevel + 20


# # Test Use
landingAccMagThreshold = 25 # m/s^2
groundLevel = 4.66 # M
initialAltitudeThreshold = groundLevel -20
landingAltitudeThreshold = groundLevel + 10


# Timeout tracking variables
LOW_ALTITUDE_TIMEOUT = 10
RF_TIMEOUT = 120 # Default 300s = 5 Min
SERVO_WAIT_TIME = 10 # Default 10s
# LOGGER_TIMEOUT = 60
# LOGGER_BUFFER = 2 * 60 * 100 # 2 x 60 minutes, at 100 Hz

# Temperature Offser
tOffset = -6

# Servo Angle
servoStartAngle = 0
servoEndAngle = 275

# RF ENABLE
global GPIO_ENABLE

# Global list to track processes
processes = []

# Shared data structure for IMU data (using Array for faster access)
shared_imu_data = multiprocessing.Array(ctypes.c_double, 11)  # Array for Q_w, Q_x, Q_y, Q_z, a_x, a_y, a_z, temperature, pressure, altitude, accel_magnitude
shared_rf_data = multiprocessing.Array(ctypes.c_double, 13) # temperature, apogee, battry_percentage, survivability_percentage, Q_w, Q_x, Q_y, Q_z, detection_time_H, detection_time_M, detection_time_S, max_velocity, landing_velocity

# Shared values for landing detection and survivability
landing_detected = multiprocessing.Value(ctypes.c_bool, False)  # Boolean for landing detection
landing_detection_time = multiprocessing.Value(ctypes.c_double, 0.0)  # Time of landing detection
survivability_percentage = multiprocessing.Value(ctypes.c_double, 0.0)  # Double for survivability percentage
current_velocity = multiprocessing.Value(ctypes.c_double, 0.0)  
# Shared values for velocity
max_velocity = multiprocessing.Value(ctypes.c_double, 0.0)  # Double for max velocity
landing_velocity = multiprocessing.Value(ctypes.c_double, 0.0)  # Double for landing velocity
velocity = multiprocessing.Value(ctypes.c_double, 0.0)  # Double for current velocity

apogee_reached = multiprocessing.Value(ctypes.c_double, 0.0)  # Double for apogee
battry_percentage = multiprocessing.Value(ctypes.c_double, 0.0)  # Double for battry percentage

# State flags
ledState = multiprocessing.Value(ctypes.c_bool, False)  # LED state flag
landedState = multiprocessing.Value(ctypes.c_bool, False)  # Landed state flag
initialAltitudeAchieved = multiprocessing.Value(ctypes.c_bool, False)  # Initial altitude achieved flag
stop_requested = multiprocessing.Value(ctypes.c_bool, False) # Global Process Stop Flag

sender = None
imu = None

def imu_data_process(imu, shared_data):
    """
    Process function to continuously read data from the IMU and update the shared data structure.
    """
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)

    while True:
        if stop_requested.value:
            break

        imu.readData()

        if imu.currentData:
            # Update shared data
            shared_data[0] = imu.currentData.Q_w
            shared_data[1] = imu.currentData.Q_x
            shared_data[2] = imu.currentData.Q_y
            shared_data[3] = imu.currentData.Q_z
            shared_data[4] = imu.currentData.a_x
            shared_data[5] = imu.currentData.a_y
            shared_data[6] = imu.currentData.a_z
            shared_data[7] = imu.currentData.temperature + tOffset
            shared_data[8] = imu.currentData.pressure
            
            # Calculate altitude based on pressure and update shared data
            seaLevelPressure = 101.325  # Standard atmospheric pressure at sea level in hPa
            altitude = 44330.0 * (1.0 - math.pow(shared_data[8] / seaLevelPressure, 0.1903))
            shared_data[9] = altitude
            # print(shared_data[8])

            # Update apogee if the current altitude is higher than the recorded apogee
            if altitude > apogee_reached.value:
                apogee_reached.value = altitude

            # Calculate acceleration magnitude and update shared data
            accel_magnitude = math.sqrt(shared_data[4] ** 2 + shared_data[5] ** 2 + shared_data[6] ** 2)
            shared_data[10] = accel_magnitude


# TO-DO: Add timeout timer and photoresistor
def landing_detection_process(shared_data, landing_detected, landing_detection_time):
    """
    Process function to monitor IMU data for landing detection condition.
    Sets landing_detected to True based on altitude and acceleration thresholds.
    Publishes this condition over ZMQ socket.
    """
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)


    # Set up ZMQ publisher
    # context = zmq.Context()
    # socket = context.socket(zmq.PUB)
    # socket.bind("tcp://*:5556")
    timer_set = False
    landed_time_set = False

    initial_altitude_logged = False
    landing_logged = False    

    while True:
        if stop_requested.value:
            break    

        # Use calculated altitude and acceleration magnitude from shared data
        accel_magnitude = shared_data[10]
        altitude = shared_data[9]

        # Check if initial altitude threshold is achieved
        if altitude > initialAltitudeThreshold:
            initialAltitudeAchieved.value = True

        # Update landed state if conditions are met
        if initialAltitudeAchieved.value and altitude < landingAltitudeThreshold and accel_magnitude > landingAccMagThreshold:
            landedState.value = True
            landing_detected.value = True

            # Record the system time of landing detection
            if not landed_time_set:
                landing_detection_time.value = time.time()
                landed_time_set = True

            # Publish landing detected message
            # socket.send_string("Landing Detected")

        if initialAltitudeAchieved.value and altitude < landingAltitudeThreshold:
                
            if not timer_set:    
                timeout_start_time = time.perf_counter()  # Start the timer 
                timer_set = True

            # print(f"{(time.perf_counter() - timeout_start_time):.2f}")

            if time.perf_counter() - timeout_start_time >= LOW_ALTITUDE_TIMEOUT:
                # Landing conditions are met
                landedState.value = True
                landing_detected.value = True
                if not landed_time_set:
                    landing_detection_time.value = time.time()
                    landed_time_set = True
                # socket.send_string("Landing Detected")
                # print("Landing Detected!")
    

        if initialAltitudeAchieved.value and not initial_altitude_logged:
            print(f"Initial Altitude Achieved at {time.perf_counter():.2f} seconds")
            initial_altitude_logged = True

        if landing_detected.value and not landing_logged:
            print(f"Landing Detected at {time.perf_counter():.2f} seconds")
            landing_logged = True

         
def survivability_process(shared_data, survivability_percentage):
    """
    Process function to calculate and update the survivability percentage.
    """
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)

    # Constants
    omega = 52.90  # lumped spinal frequency (rad/s)
    zeta = 0.224  # damping ratio
    g = 9.81  # acceleration on Earth
    max_DRI = 17.7

    # Initial values
    X = 0.0  # relative spinal deflection
    X_dot = 0.0  # relative spinal velocity
    X_max = 0.0  # maximum spinal compression

    last_time = time.time()

    while True:
        if stop_requested.value:
            break

        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time

        # Use calculated acceleration magnitude from shared data
        z_ddot = shared_data[6] + 9.18
        z_ddot = abs(z_ddot)
        # print(z_ddot)
        
        # Calculate the spinal acceleration (X_ddot)
        X_ddot = z_ddot - (2 * zeta * omega * X_dot) - (pow(omega, 2) * X)
        # print(X_ddot)

        # Update velocity using Euler's Method
        X_dot += X_ddot * dt
        X += X_dot * dt
        # print(X)

        # max displacement updating
        if abs(X) > X_max:
            X_max = abs(X)
        
        # Calculate DRI and survivability percentage
        cur_DRI = (pow(omega, 2) / g) * X_max  # calculated DRI given X_max
        # print(cur_DRI)

        # survive_percentage = (max_DRI - abs(cur_DRI)) / max_DRI * 100  # converts DRI value to percentage based on max allowable DRI
        if cur_DRI <= max_DRI:
            survivability_percentage.value = 100
        else:
            survivability_percentage.value = 80

        if cur_DRI >= (max_DRI * 2):
            survivability_percentage.value = 50


        # Update the shared survivability_percentage value
        # survivability_percentage.value = survive_percentage


def update_rf_data_process(shared_imu_data, landing_detected, landing_detection_time, shared_rf_data, apogee_reached, battry_percentage, survivability_percentage, max_velocity, landing_velocity):
    """
    Process function to update shared_rf_data continuously until landing is detected.
    """
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)

    landed_time_set = False  # Flag to ensure landing time is only set once
    landed_velocity_set = False
    landing_velocity.value = 0
    max_velocity.value = 0
    start_time = time.perf_counter()
    velocity_ready_flag = False

    while True:
        if stop_requested.value:
            break

        shared_rf_data[12] = landing_velocity.value  # Landing velocity
        # Update Landing velocity
        if landing_detected.value and not landed_velocity_set:
            landing_velocity.value = abs(current_velocity.value)
            landed_velocity_set = True

        # Ensure exactly 5 seconds pass before updating Max Velocity
        if not velocity_ready_flag and time.perf_counter() - start_time > 5:
            velocity_ready_flag = True  # Only set once after 5 seconds

        if velocity_ready_flag and abs(current_velocity.value) >= abs(max_velocity.value):
            max_velocity.value = abs(current_velocity.value)
        
        # Update RF data based on shared IMU data and other shared values
        shared_rf_data[0] = shared_imu_data[7]  # Temperature from IMU data
        shared_rf_data[1] = apogee_reached.value  # Apogee reached
        shared_rf_data[2] = battry_percentage.value  # Battery percentage
        shared_rf_data[3] = survivability_percentage.value  # Survivability percentage

        # Update velocity data
        shared_rf_data[11] = max_velocity.value  # Max velocity
        
        # Set landing time only once after landing is detected
        if landing_detected.value and not landed_time_set:
            detection_time = time.strftime('%H:%M:%S', time.localtime(landing_detection_time.value))
            hours, minutes, seconds = map(int, detection_time.split(':'))
            shared_rf_data[8] = hours  # Detection time hours
            shared_rf_data[9] = minutes  # Detection time minutes
            shared_rf_data[10] = seconds  # Detection time seconds
            landed_time_set = True


        # Quarternion need to always be updated
        shared_rf_data[4] = shared_imu_data[0]  # Q_w (quaternion w)
        shared_rf_data[5] = shared_imu_data[1]  # Q_x (quaternion x)
        shared_rf_data[6] = shared_imu_data[2]  # Q_y (quaternion y)
        shared_rf_data[7] = shared_imu_data[3]  # Q_z (quaternion z)

def send_rf_data_process(shared_rf_data, landing_detected):
    """
    Process function to send RF data when landing is detected.
    """
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)

    # TO-DO: I have changed this to be active all time to avoid junk data. 
    while True:
        if stop_requested.value:
            break

        # Continuously Sending RF data
        sender.set_active(True)
        rf_data = [float(value) for value in shared_rf_data]  # Convert shared_rf_data to a list of floats
        sender.monitor_and_send(rf_data)
        time.sleep(0.5)

def release_latch_servo(servo, landing_detected):
    """
    Process function to release the latch and servo when landing is detected.
    """
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)

    action_triggered = False  # Flag to ensure action happens only once

    while True:
        if stop_requested.value:
            break

        if landing_detected.value and not action_triggered:
            action_triggered = True  # Set the flag
            # servo.set_servo_angle(servoEndAngle)  # Servo
            lgpio.gpio_write(GPIO_ENABLE, 19, 1)  # Latch
            lgpio.gpio_write(GPIO_ENABLE, 17, 1)  # RF
            # print("RF Enabled")
            # Wait for 5 minutes (300 seconds) and turn off RF
            start_time = time.perf_counter()
            servo_moved = False

            while True:
                if stop_requested.value:
                            break
                
                current_time = time.perf_counter()            
                
                if current_time - start_time >= SERVO_WAIT_TIME and not servo_moved:
                    servo.set_servo_angle(servoEndAngle)
                    servo_moved = True

                if current_time - start_time >= RF_TIMEOUT:
                    lgpio.gpio_write(GPIO_ENABLE, 17, 0)  # Turn off RF
                    # print("RF Disabled")
                    break
            break  # Stop the process after releasing the latch and servo


def data_logging_process(shared_imu_data, shared_rf_data, landing_detected, apogee_reached, current_velocity, landedState, initialAltitudeAchieved):
    """
    Logging process with launch detection (based on initialAltitudeAchieved) and post-landing timeout.
    Logs pre-launch data (2-minute rolling buffer at 100 Hz) and post-launch data into separate files, then combines them with column headers.
    """
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)

    target_frequency = 100  # Hz
    interval = 1 / target_frequency  # Seconds (10ms for 100Hz)
    start_time = time.perf_counter()

    time.sleep(5)

    # Write header once before entering the loop
    with open("data_log.txt", "w") as f:
        f.write("Time,Q_w,Q_x,Q_y,Q_z,a_x,a_y,a_z,temperature,pressure,altitude,accel_magnitude,apogee,battery_percentage,survivability_percentage,detection_time_H,detection_time_M,detection_time_S,max_velocity,landing_velocity,current_velocity,landedState,initialAltitudeAchieved\n")

    while True:
        if stop_requested.value:
            break

        current_time = time.perf_counter()  # Update time

        if current_time - start_time >= interval:
            # Format data into a string
            data_str = (
                f"{current_time:.2f},"               # 1  Time
                f"{shared_imu_data[0]:.2f},"         # 2  Q_w
                f"{shared_imu_data[1]:.2f},"         # 3  Q_x
                f"{shared_imu_data[2]:.2f},"         # 4  Q_y
                f"{shared_imu_data[3]:.2f},"         # 5  Q_z
                f"{shared_imu_data[4]:.2f},"         # 6  a_x
                f"{shared_imu_data[5]:.2f},"         # 7  a_y
                f"{shared_imu_data[6]:.2f},"         # 8  a_z
                f"{shared_imu_data[7]:.2f},"         # 9  temperature
                f"{shared_imu_data[8]:.2f},"         # 10 pressure
                f"{shared_imu_data[9]:.2f},"         # 11 altitude
                f"{shared_imu_data[10]:.2f},"        # 12 accel_magnitude

                f"{shared_rf_data[1]:.2f},"          # 13 apogee
                f"{shared_rf_data[2]:.2f},"          # 14 battery_percentage
                f"{shared_rf_data[3]:.2f},"          # 15 survivability_percentage
                f"{shared_rf_data[8]:.0f},"          # 16 detection_time_H
                f"{shared_rf_data[9]:.0f},"          # 17 detection_time_M
                f"{shared_rf_data[10]:.0f},"         # 18 detection_time_S
                f"{shared_rf_data[11]:.2f},"         # 19 max_velocity
                f"{shared_rf_data[12]:.2f},"         # 20 landing_velocity

                f"{current_velocity.value:.2f},"     # 21 current_velocity
                f"{int(landedState.value)},"         # 22 landedState
                f"{int(initialAltitudeAchieved.value)}\n"  # 23 initialAltitudeAchieved
            )



            # Open file, write data, flush, and close
            with open("data_log.txt", "a") as f:
                f.write(data_str)
                f.flush()  # Ensure data is written immediately

            start_time = current_time  # Update time step

def update_velocity_process(shared_imu_data, landing_detected):
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    
    # Parameters
    target_frequency = 100  # Hz
    interval = 1 / target_frequency  # Seconds (5.56ms for 180Hz)
    cutoff_freq = 0.75  # Hz
    tau = 1 / (2 * np.pi * cutoff_freq)  # Time constant

    # Compute filter coefficients
    T = interval  # Sampling interval
    a0 = 2 / (T + 2 * tau)
    a1 = (T - 2 * tau) / (T + 2 * tau)

    window_size = 100  # Number of recent altitude readings to average
    altitude_window = []  # List to store recent altitude readings

    # Initial conditions
    previous_altitude = 0.0
    previous_velocity = 0.0
    # altitude = 0

    # Time tracking
    start_time = time.perf_counter()

    while True:
        if stop_requested.value:
            break

        current_time = time.perf_counter()
        if current_time - start_time >= interval:
            # Get the current altitude from shared data
            raw_altitude = shared_imu_data[9]  # Raw noisy altitude data
            
            # Add the new altitude reading to the window
            altitude_window.append(raw_altitude)
            if len(altitude_window) > window_size:
                altitude_window.pop(0)  # Remove the oldest reading if the window is full
            
            # Calculate the filtered altitude (moving average)
            altitude = sum(altitude_window) / len(altitude_window)

            # Calculate velocity using the difference equation
            current_velocity.value = a0 * (altitude - previous_altitude) - a1 * previous_velocity
            # print(f"Filtered Altitude: {altitude:.2f}, Velocity: {current_velocity.value:.2f}, MaxV: {max_velocity.value:.2f}, LandingV: {landing_velocity.value:.2f}")

            # Update previous values
            previous_altitude = altitude
            previous_velocity = current_velocity.value

            # Reset the time step
            start_time = current_time

def signal_handler(signum, frame):
    """
    Minimal signal handler that sets a global 'stop' flag
    and does no I/O.
    """
    stop_requested.value = True


def cleanup():
    # Safe to do prints & file I/O as a normal function
    print("\nStopping processes...")
    sender.close()

    # Terminate all child processes
    for p in processes:
        if p.is_alive():
            p.terminate()
    for p in processes:
        p.join()

    # GPIO Cleanup
    lgpio.gpio_write(GPIO_ENABLE, 17, 0)
    lgpio.gpio_write(GPIO_ENABLE, 19, 0)
    lgpio.gpiochip_close(GPIO_ENABLE)
    servo.release()

    print("Processes stopped. GPIO cleanup done.")


if __name__ == "__main__":
    # Initialize the IMU
    imu = VN100IMU()
    sender = I2CSender()

    # Initialize ENABLE
    GPIO_ENABLE = lgpio.gpiochip_open(0)  # Open GPIO chip 0
    try:
        lgpio.gpio_claim_output(GPIO_ENABLE, 17)  # Set GPIO 17 as ENABLE
    except lgpio.error as e:
        if "GPIO busy" in str(e):
            print("GPIO 17 is busy, releasing and reinitializing...")
            lgpio.gpio_free(GPIO_ENABLE, 17)  # Free GPIO 17 if busy
            lgpio.gpio_claim_output(GPIO_ENABLE, 17)  # Set GPIO 17 as output again

    try:
        lgpio.gpio_claim_output(GPIO_ENABLE, 19)  # Set GPIO 19 as LATCH
    except lgpio.error as e:
        if "GPIO busy" in str(e):
            print("GPIO 19 is busy, releasing and reinitializing...")
            lgpio.gpio_free(GPIO_ENABLE, 19)  # Free GPIO 19 if busy
            lgpio.gpio_claim_output(GPIO_ENABLE, 19)  # Set GPIO 19 as output again

    lgpio.gpio_write(GPIO_ENABLE, 17, 0)  # Disable RF
    lgpio.gpio_write(GPIO_ENABLE, 19, 0)  # Disable LATCH

    # Initialize Servo
    servo = ServoController(servo_pin=13)
    servo.set_servo_angle(servoStartAngle)

    # Start processes and store in global list
    processes.extend([
        multiprocessing.Process(target=imu_data_process, args=(imu, shared_imu_data)),
        multiprocessing.Process(target=landing_detection_process, args=(shared_imu_data, landing_detected, landing_detection_time)),
        multiprocessing.Process(target=survivability_process, args=(shared_imu_data, survivability_percentage)),
        multiprocessing.Process(target=update_rf_data_process, args=(
            shared_imu_data, landing_detected, landing_detection_time, shared_rf_data,
            apogee_reached, battry_percentage, survivability_percentage, max_velocity, landing_velocity
        )),
        multiprocessing.Process(target=send_rf_data_process, args=(shared_rf_data, landing_detected)),
        multiprocessing.Process(target=data_logging_process, args=(
            shared_imu_data, shared_rf_data, landing_detected, apogee_reached, current_velocity, landedState, initialAltitudeAchieved
        )),
        multiprocessing.Process(target=release_latch_servo, args=(servo, landing_detected,)),
        multiprocessing.Process(target=update_velocity_process, args=(shared_imu_data, landing_detected,))
    ])

    # Start all processes
    for p in processes:
        p.start()

    # Register signal handlers **AFTER** defining processes
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP, signal.SIGQUIT):
        signal.signal(sig, signal_handler)


    try:
        while True:
            detection_time = time.strftime('%H:%M:%S', time.localtime(landing_detection_time.value)) if landing_detected.value else "N/A"
            print(
                f"Altitude: {shared_imu_data[9]:.2f} m, "
                f"Acceleration Magnitude: {shared_imu_data[10]:.2f} m/s^2, "
                f"Landing Detected: {landing_detected.value}, "
                f"Detection Time: {detection_time}"
            )
            # print("RF Shared Data: " + ", ".join(f"{value:.2f}" for value in shared_rf_data))
            
            # Catch Exit
            if stop_requested.value:
                # If signaled, do real cleanup and exit
                cleanup()
                sys.exit(0)

    except KeyboardInterrupt:
        cleanup()
        sys.exit(0)
