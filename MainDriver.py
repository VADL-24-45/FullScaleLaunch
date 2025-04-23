import multiprocessing
import ctypes
import time
import math
import numpy as np
from VN100 import VN100IMU, IMUData  # Assuming VN100IMU class and IMUData dataclass are available
from RF import I2CSender
from ServoLatch import ServoController
import lgpio
import os
from collections import deque
import signal
import sys
from Battery import ADS1115
from scipy.interpolate import interp1d
from DRI import DRI

PRINT_MODE = False
PRINT_FREQUENCY = 10

######################################################################################
# New constants (all in seconds or meters)
GROUND_LEVEL = 120
LAUNCH_THRESHOLD = GROUND_LEVEL + 100 # Default (300)
LAUNCH_DURATION = 2.0 # Default (2)
FREEZE_PERIOD = 20 # Default (50)
LOW_ALT_THRESHOLD = GROUND_LEVEL + 20 # Default (20)
MIN_STABLE_DURATION = 6.0 # Default (10)
RF_FAILSAFE_DURATION = 150 # Default (180s)
RF_OFF_TIMEOUT = 180 # Default (180)
LANDING_ACCEL_THRESHOLD = 40

SERVO_RF_WAIT_TIME = 15 # Default 15s
LOGGER_TIMEOUT = 120
LOGGER_BUFFER = 18000 # 3 minutes x 60 seconds/min x 100 Hz
######################################################################################

# Temperature Offser
tOffset = -10.11

# INVARIANTS

# Servo Angle
servoStartAngle = 0
servoEndAngle = 275

# RF ENABLE
global GPIO_ENABLE

# Global list to track processes
processes = []

# Shared data structure for IMU data (using Array for faster access)
shared_imu_data = multiprocessing.Array(ctypes.c_double, 11)  # Array for Q_w, Q_x, Q_y, Q_z, a_x, a_y, a_z, temperature, pressure, altitude, accel_magnitude
shared_rf_data = multiprocessing.Array(ctypes.c_double, 14) # temperature, apogee, battery_percentage, survivability_percentage, Q_w, Q_x, Q_y, Q_z, detection_time_H, detection_time_M, detection_time_S, max_velocity, landing_velocity, landing_acceleration

# Shared values for landing detection and survivability
landing_detection_time = multiprocessing.Value(ctypes.c_double, 0.0)  # Time of landing detection
survivability_percentage = multiprocessing.Value(ctypes.c_double, 0.0)  # Double for survivability percentage
current_velocity = multiprocessing.Value(ctypes.c_double, 0.0)  
# Shared values for velocity
max_velocity = multiprocessing.Value(ctypes.c_double, 0.0)  # Double for max velocity
landing_velocity = multiprocessing.Value(ctypes.c_double, 0.0)  # Double for landing velocity
landing_acceleration = multiprocessing.Value(ctypes.c_double, 0.0)  # Double for landing acceleration

apogee_reached = multiprocessing.Value(ctypes.c_double, 0.0)  # Double for apogee
battery_percentage = multiprocessing.Value(ctypes.c_double, 0.0)  # Double for battry percentage

# State flags
ledState = multiprocessing.Value(ctypes.c_bool, False)  # LED state flag
landedState = multiprocessing.Value(ctypes.c_bool, False)  # Landed state flag
initialAltitudeAchieved = multiprocessing.Value(ctypes.c_bool, False)  # Initial altitude achieved flag
stop_requested = multiprocessing.Value(ctypes.c_bool, False) # Global Process Stop Flag
detachParachute = multiprocessing.Value(ctypes.c_bool, False) # Global Process Parashute Detachment Flag

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
            if altitude - GROUND_LEVEL > apogee_reached.value:
                apogee_reached.value = altitude - GROUND_LEVEL

            # Calculate acceleration magnitude and update shared data
            accel_magnitude = math.sqrt(shared_data[4] ** 2 + shared_data[5] ** 2 + shared_data[6] ** 2)
            shared_data[10] = accel_magnitude

def landing_detection_process(shared_data, landedState, landing_detection_time):
    """
    Process function implementing the new robust decision logic:
    
    1. Launch Detection (Altitude > 1000ft / ~304.8m sustained for 3 s)
       - When detected, set initialAltitudeAchieved and record launch_detection_time.
    2. Freeze Period (FREEZE_PERIOD seconds after launch detection)
       - During this period, ignore low-altitude logic.
    3. Low Altitude Tracking (Altitude < 70ft / ~21.3 m)
       - After freeze, if altitude is below the threshold, update min_altitude
         only if a new lower value is found. Any violation (altitude above threshold)
         resets the stored min_altitude to +inf.
    4. Landed Detection
       - If no new minimum is recorded for MIN_STABLE_DURATION seconds,
         set landedState to True.
    5. RF Fail-Safe
       - If RF_FAILSAFE_DURATION seconds have passed since launch detection
         without landing, force landedState.
    """
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN) 

    FREEZE_START_DEBUG = False
    FREEZE_END_DEBUG = False
    LOW_ALT_DEBUG = False
    LD_PRINT_DEBUG = True

    # Timers and state for launch detection
    launch_timer = None
    launch_detection_time = None
    landed_time_set = False

    # Variables for low altitude tracking
    min_altitude = float("inf")
    max_acceleration_after_low = - float("inf")
    min_alt_timer = None

    # Main Logic Starts Here
    while True:
        if stop_requested.value:
            break    

        current_time = time.perf_counter() # Get current time
        altitude = shared_data[9] # Get current altitude
        acceleration = abs(shared_data[10]) # Get current acceleration
        
        # --- (1) Launch Detection ---
        if not initialAltitudeAchieved.value:
            if altitude > LAUNCH_THRESHOLD: # > Launch Threshold
                if launch_timer is None:
                    launch_timer = current_time # Set timer
                    if LD_PRINT_DEBUG: 
                        print(f"DEBUG: Above launch threshold ({LAUNCH_THRESHOLD}m). Starting launch timer...")

                elif (current_time - launch_timer) >= LAUNCH_DURATION: # Time Check Pass
                    initialAltitudeAchieved.value = True # High Altitude Achieved
                    launch_detection_time = current_time # Record Time of High Altitude Achieve
                    if LD_PRINT_DEBUG:
                        print(f"DEBUG: Launch detected at t={current_time:.2f}s (Altitude: {altitude:.2f} m)")
            else:
                if launch_timer is not None:
                    if LD_PRINT_DEBUG:
                        print(f"DEBUG: Launch timer reset (Altitude dropped to {altitude:.2f} m)")
                launch_timer = None
        
        # --- (RF FAIL-SAFE) ---
        if initialAltitudeAchieved.value and (launch_detection_time is not None) and not landedState.value:
            if (current_time - launch_detection_time) >= RF_FAILSAFE_DURATION: # Timer for RF Timeout
                landedState.value = True  # Only set RF
                detachParachute.value = True # Now also set detatchment
                if not landed_time_set:
                    landing_detection_time.value = time.time()
                    landed_time_set = True
                print("DEBUG: RF Fail-safe triggered: Forcing landed state due to timeout.")
        
        # --- (2) Freeze Period ---
        if initialAltitudeAchieved.value and launch_detection_time is not None:
            if (current_time - launch_detection_time) < FREEZE_PERIOD: # Freeze Period
                
                #############################DEBUG################################
                if not FREEZE_START_DEBUG and LD_PRINT_DEBUG:
                    print("DEBUG: Start freeze period")
                    FREEZE_START_DEBUG = True
                ###################################################################

                # During freeze period, ignore low altitude changes; reset min altitude tracking.
                if min_altitude != float("inf"):
                    if LD_PRINT_DEBUG:
                        print("DEBUG: In freeze period. Resetting low-altitude tracking.")
                min_altitude = float("inf")
                max_acceleration_after_low = - float("inf")
                min_alt_timer = None
            else:
                #############################DEBUG################################
                if not FREEZE_END_DEBUG and LD_PRINT_DEBUG:
                    print("DEBUG: End freeze period")
                    FREEZE_END_DEBUG = True
                ##################################################################

                # --- (3) Low Altitude Tracking ---
                if altitude < LOW_ALT_THRESHOLD:

                    max_acceleration_after_low = max(max_acceleration_after_low, acceleration)
                    #############################DEBUG################################
                    if not LOW_ALT_DEBUG:
                        if LD_PRINT_DEBUG:
                            print("DEBUG: Low altitude detected")
                        LOW_ALT_DEBUG = True
                    ##################################################################
                   
                    # Altitude is below threshold: update minimum if lower value found.
                    if altitude - 1 < min_altitude: # Norse Margin of 1 m
                        min_altitude = altitude - 1 # Update new minimum
                        min_alt_timer = current_time # Re-start Timer
                        if LD_PRINT_DEBUG:
                            print(f"DEBUG: New minimum altitude recorded: {min_altitude:.2f} m at t={current_time:.2f}s")
                    else:
                        # No new lower reading; do nothing.
                        pass
                    
                else:
                    # Altitude violation: above threshold. Reset low altitude tracking.
                    if min_altitude != float("inf"):
                        if LD_PRINT_DEBUG:
                            print(f"DEBUG: Altitude violation (Altitude: {altitude:.2f} m) - resetting min altitude.")
                    min_altitude = float("inf") # Set minimum to infinity at violation
                    min_alt_timer = None
                    max_acceleration_after_low = - float("inf")


                
                # --- (4) Landed Detection ---
                if min_alt_timer is not None:
                    stable_duration = current_time - min_alt_timer
                    if LD_PRINT_DEBUG:
                        print(f"DEBUG: Stable duration at low altitude: {stable_duration:.2f} s")
                    if stable_duration >= MIN_STABLE_DURATION and max_acceleration_after_low > LANDING_ACCEL_THRESHOLD and not landedState.value:
                        landedState.value = True
                        detachParachute.value = True
                        if not landed_time_set:
                            landing_detection_time.value = time.time()
                            landed_time_set = True
                        if LD_PRINT_DEBUG and landedState.value:
                            print(f"DEBUG: Landed state detected after stable low altitude of {MIN_STABLE_DURATION} s.")
                            return


def survivability_process(shared_data,
                          current_velocity,
                          landing_velocity,
                          landing_acceleration,
                          survivability_percentage):
    signal.signal(signal.SIGINT,  signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)

    target_frequency = 100
    interval         = 1 / target_frequency
    buffer_len       = int(target_frequency * (MIN_STABLE_DURATION + 30))   # -15 s

    omega_n, zeta = 52.9, 0.224
    dri_done      = False
    DR_val        = 0.0

    time_buf  = deque(maxlen=buffer_len)
    accel_buf = deque(maxlen=buffer_len)
    vel_buf   = deque(maxlen=buffer_len)

    last_tick = time.perf_counter()

    while True:
        if stop_requested.value:
            return

        now = time.perf_counter()
        if now - last_tick >= interval:
            last_tick = now

            # collect data until landing detected
            if not landedState.value and not dri_done:
                time_buf.append(now)
                accel_buf.append(shared_data[4])     # a_x
                vel_buf.append(abs(current_velocity.value))

            else:
                # After Landing
                if not dri_done and len(accel_buf) >= 2:
                    t_arr = np.array(time_buf)
                    a_arr = np.array(accel_buf)
                    v_arr = np.array(vel_buf)

                    # peak deceleration
                    acc_idx = int(np.argmax(np.abs(a_arr)))
                    landing_acceleration.value = float(a_arr[acc_idx])

                    # peak speed
                    vel_idx = int(np.argmax(np.abs(v_arr)))
                    landing_velocity.value     = float(v_arr[vel_idx])


                    # DRI
                    t_uniform = np.linspace(t_arr[0], t_arr[-1], len(t_arr))
                    a_uniform = interp1d(t_arr, a_arr, kind='linear',
                                         fill_value="extrapolate")(t_uniform)
                    DR_arr = DRI().calc_DRI(t_uniform, a_uniform,
                                            omega_n, zeta)
                    DR_val = float(np.max(np.abs(DR_arr)))

                    dri_done = True
                    survivability_percentage.value = DR_val

                    if PRINT_MODE:
                        print(f"[survivability] a={landing_acceleration.value:.2f} "
                              f"v={landing_velocity.value:.2f} "
                              f"DRI={DR_val:.2f}")

                    return      # stop the subprocess; work is done



def update_rf_data_process(shared_imu_data, landedState, landing_detection_time, shared_rf_data, apogee_reached, battery_percentage, survivability_percentage, max_velocity, landing_velocity):
    """
    Process function to update shared_rf_data continuously until landing is detected.
    """
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)

    landed_time_set = False  # Flag to ensure landing time is only set once
    landed_velocity_set = False
    max_velocity.value = 0
    start_time = time.perf_counter()
    velocity_ready_flag = False

    while True:
        if stop_requested.value:
            break

        shared_rf_data[12] = landing_velocity.value  # Landing velocity

        # Ensure exactly 5 seconds pass before updating Max Velocity
        if not velocity_ready_flag and time.perf_counter() - start_time > 5:
            velocity_ready_flag = True  # Only set once after 5 seconds

        if velocity_ready_flag and abs(current_velocity.value) >= abs(max_velocity.value):
            max_velocity.value = abs(current_velocity.value)

        shared_rf_data[13] = landing_acceleration.value
        
        # Update RF data based on shared IMU data and other shared values
        shared_rf_data[0] = shared_imu_data[7]  # Temperature from IMU data
        shared_rf_data[1] = apogee_reached.value  # Apogee reached
        shared_rf_data[2] = battery_percentage.value  # Battery percentage
        shared_rf_data[3] = survivability_percentage.value  # Survivability percentage

        # Update velocity data
        shared_rf_data[11] = max_velocity.value  # Max velocity
        
        # Set landing time only once after landing is detected
        if landedState.value and not landed_time_set:
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

def send_rf_data_process(shared_rf_data, landedState):
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

def release_latch_servo(servo, landedState):
    """
    Process function to release the latch and servo when landing is detected.
    """
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)

    action_triggered = False  # Flag to ensure action happens only once

    while True:
        if stop_requested.value:
            break

        if detachParachute.value and not action_triggered:
            action_triggered = True  # Set the flag
            lgpio.gpio_write(GPIO_ENABLE, 19, 1)  # Latch

        if landedState.value:
            start_time = time.perf_counter() # Begin Timer
            servo_moved = False

            while True:
                if stop_requested.value:
                    break

                current_time = time.perf_counter()            
                
                if current_time - start_time >= SERVO_RF_WAIT_TIME and not servo_moved:
                    servo.set_servo_angle(servoEndAngle) # Extend Antenna
                    lgpio.gpio_write(GPIO_ENABLE, 17, 1)  # RF
                    servo_moved = True

                if current_time - start_time >= RF_OFF_TIMEOUT:
                    lgpio.gpio_write(GPIO_ENABLE, 17, 0)  # Turn off RF
                    # print("RF Disabled")
                    break

            break  # Stop the process after releasing the latch and servo


def data_logging_process(shared_imu_data, shared_rf_data, apogee_reached, current_velocity, landedState, initialAltitudeAchieved):
    """
    Logging process with launch detection (based on initialAltitudeAchieved) and post-landing timeout.
    Logs pre-launch data (2-minute rolling buffer at 100 Hz) and post-launch data into separate files, then combines them with column headers.
    """
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)

    # File names
    pre_file = "data_log_pre.txt"
    post_file = "data_log_post.txt"
    output_file = "data_log_combined.txt"
    # Rolling buffer for pre-launch data (2-minute rolling window at 100 Hz)
    rolling_buffer = deque(maxlen=LOGGER_BUFFER)  # 12000 entries for 2 minutes at 100 Hz
    target_frequency = 100  # Hz
    interval = 1 / target_frequency  # 10 ms
    start_time = time.perf_counter()
    last_logging_time = start_time  # To control logging frequency
    landing_event_time = None  # To track the time of landing detection

    print("Data logging process started.")

    # Clear post file at the start to avoid appending old data
    with open(post_file, "w") as post_f:
        pass  # Just open and close to truncate the file

    while True:
        if stop_requested.value:
            break

        current_time = time.perf_counter()  # Current time

        # Ensure consistent logging frequency
        if current_time - last_logging_time >= interval:
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

            if not initialAltitudeAchieved.value:
                rolling_buffer.append(data_str)
                with open(pre_file, "w") as pre_f:
                    pre_f.write("".join(rolling_buffer))
                    pre_f.flush()
            else:
                with open(post_file, "a") as post_f:
                    post_f.write(data_str)
                    post_f.flush()

                if landedState.value and landing_event_time is None:
                    landing_event_time = current_time
                    print("Landing detected. Starting post-landing timeout.")

                if landing_event_time and current_time - landing_event_time >= LOGGER_TIMEOUT:
                    print("Timeout reached. Stopping logging process.")
                    break

            last_logging_time = current_time

    combine_files(pre_file, post_file, output_file)
    print(f"Data logging completed. Logs combined into {output_file}")

def combine_files(pre_file, post_file, output_file):
    """
    Combine pre_file and post_file into a single output file, with column headers added.
    """
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)

    headers = "Time,Q_w,Q_x,Q_y,Q_z,a_x,a_y,a_z,temperature,pressure,altitude,accel_magnitude,apogee,battery_percentage,survivability_percentage,detection_time_H,detection_time_M,detection_time_S,max_velocity,landing_velocity,current_velocity,landedState,initialAltitudeAchieved\n"

    with open(output_file, "w") as out_f:
        out_f.write(headers)
        
        if os.path.exists(pre_file):
            with open(pre_file, "r") as pre_f:
                out_f.write(pre_f.read())

        if os.path.exists(post_file):
            with open(post_file, "r") as post_f:
                out_f.write(post_f.read())

def update_velocity_process(shared_imu_data, landedState):
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

def update_battery_process(battery_percentage):
    # Ignore SIGINT and SIGTERM in this process
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)

    interval = 1
    start_time = time.perf_counter()
    battery = ADS1115(i2c_bus=6, device_address=0x48)
    lowest_battery_percent = 100
 
    while True:
        if stop_requested.value:
            break

        current_time = time.perf_counter()
        if current_time - start_time >= interval:
            battery_percentage_local = battery.battery_voltage_to_percentage(battery.calc_vbat(battery.get_adc()))

            if battery_percentage_local <= lowest_battery_percent:
                lowest_battery_percent = battery_percentage_local

            battery_percentage.value = lowest_battery_percent
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
        multiprocessing.Process(target=landing_detection_process, args=(shared_imu_data, landedState, landing_detection_time)),
        multiprocessing.Process(target=survivability_process, args=(shared_imu_data, current_velocity, landing_velocity, landing_acceleration, survivability_percentage)),
        multiprocessing.Process(target=update_rf_data_process, args=(
            shared_imu_data, landedState, landing_detection_time, shared_rf_data,
            apogee_reached, battery_percentage, survivability_percentage, max_velocity, landing_velocity
        )),
        multiprocessing.Process(target=send_rf_data_process, args=(shared_rf_data, landedState)),
        multiprocessing.Process(target=data_logging_process, args=(
            shared_imu_data, shared_rf_data, apogee_reached, current_velocity, landedState, initialAltitudeAchieved
        )),
        multiprocessing.Process(target=release_latch_servo, args=(servo, landedState,)),
        multiprocessing.Process(target=update_velocity_process, args=(shared_imu_data, landedState,)),
        multiprocessing.Process(target=update_battery_process, args=(battery_percentage,))
    ])

    # Start all processes
    for p in processes:
        p.start()

    # Register signal handlers **AFTER** defining processes
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP, signal.SIGQUIT):
        signal.signal(sig, signal_handler)


    try:
        target_frequency = PRINT_FREQUENCY  # Hz
        interval = 1 / target_frequency  
        start_time = time.perf_counter()

        while True:
            # Printing at lower frequency
            current_time = time.perf_counter()
            if current_time - start_time >= interval and PRINT_MODE:
                # Print Ver 1
                detection_time = time.strftime('%H:%M:%S', time.localtime(landing_detection_time.value)) if landedState.value else "N/A"
                print(
                    f"Alt: {shared_imu_data[9]:.2f} m, "
                    f"Acc Mag: {shared_imu_data[10]:.2f} m/s^2, "
                    f"Qw: {shared_imu_data[0]:.2f}, "
                    f"Launch: {initialAltitudeAchieved.value}, "
                    f"Land: {landedState.value}, "
                    f"Bat: {battery_percentage.value:.2f}%, "
                    f"Temp: {shared_rf_data[0]:.2f}C, "
                    f"Time: {detection_time}"
                )

                # Print Ver 2
                # print("RF Shared Data: " + ", ".join(f"{value:.2f}" for value in shared_rf_data))
                
                start_time = current_time

            # Catch Exit
            if stop_requested.value:
                # If signaled, do real cleanup and exit
                cleanup()
                sys.exit(0)

    except KeyboardInterrupt:
        cleanup()
        sys.exit(0)
