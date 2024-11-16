import multiprocessing
import ctypes
import time
import math
import zmq
from VN100 import VN100IMU, IMUData  # Assuming VN100IMU class and IMUData dataclass are available
from RF import I2CSender
from ServoLatch import ServoController
import lgpio

# Threshold values
landingAccMagThreshold = 20  # m/s^2
groundLevel = 90.39  # CHANGE THIS VALUE TO CALIBRATE IMU
landingAltitudeThreshold = groundLevel + 100
initialAltitudeThreshold = groundLevel + 0

# Temperature Offser
tOffset = -6

# Servo Angle
# servoStartAngle = 0
servoEndAngle = 275


# RF ENABLE
global GPIO_ENABLE

# Shared data structure for IMU data (using Array for faster access)
shared_imu_data = multiprocessing.Array(ctypes.c_double, 11)  # Array for Q_w, Q_x, Q_y, Q_z, a_x, a_y, a_z, temperature, pressure, altitude, accel_magnitude
shared_rf_data = multiprocessing.Array(ctypes.c_double, 13) # temperature, apogee, battry_percentage, survivability_percentage, Q_w, Q_x, Q_y, Q_z, detection_time_H, detection_time_M, detection_time_S, max_velocity, landing_velocity

# Shared values for landing detection and survivability
landing_detected = multiprocessing.Value(ctypes.c_bool, False)  # Boolean for landing detection
landing_detection_time = multiprocessing.Value(ctypes.c_double, 0.0)  # Time of landing detection
survivability_percentage = multiprocessing.Value(ctypes.c_double, 0.0)  # Double for survivability percentage

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


def imu_data_process(imu, shared_data):
    """
    Process function to continuously read data from the IMU and update the shared data structure.
    """
    while True:
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
    # Set up ZMQ publisher
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://*:5556")

    while True:
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
            landing_detection_time.value = time.time()

            # Publish landing detected message
            socket.send_string("Landing Detected")

         
def survivability_process(shared_data, survivability_percentage):
    """
    Process function to calculate and update the survivability percentage.
    """
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

        survive_percentage = (max_DRI - abs(cur_DRI)) / max_DRI * 100  # converts DRI value to percentage based on max allowable DRI

        # Update the shared survivability_percentage value
        survivability_percentage.value = survive_percentage


def update_rf_data_process(shared_imu_data, landing_detected, landing_detection_time, shared_rf_data, apogee_reached, battry_percentage, survivability_percentage, max_velocity, landing_velocity):
    """
    Process function to update shared_rf_data continuously until landing is detected.
    """
    landed_time_set = False  # Flag to ensure landing time is only set once

    while True:
        if not landing_detected.value:
            # Update RF data based on shared IMU data and other shared values
            shared_rf_data[0] = shared_imu_data[7]  # Temperature from IMU data
            shared_rf_data[1] = apogee_reached.value  # Apogee reached
            shared_rf_data[2] = battry_percentage.value  # Battery percentage
            shared_rf_data[3] = survivability_percentage.value  # Survivability percentage

            # Update velocity data
            shared_rf_data[11] = max_velocity.value  # Max velocity
            shared_rf_data[12] = landing_velocity.value  # Landing velocity
        
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
    sender = I2CSender()
    while True:
        if landing_detected.value:
            lgpio.gpio_write(GPIO_ENABLE, 17, 1)
            sender.set_active(True)
            rf_data = [float(value) for value in shared_rf_data]  # Convert shared_rf_data to a list of floats
            sender.monitor_and_send(rf_data)
            time.sleep(0.5)

def release_latch_servo(servo, landing_detected):
    """
    Process function to release the latch and servo when landing is detected.
    """
    while True:
        if landing_detected.value:
            servo.set_servo_angle(servoEndAngle)
            lgpio.gpio_write(GPIO_ENABLE, 19, 1)
            break  # Stop the process after releasing the latch and servo

def data_logging_process(shared_imu_data, shared_rf_data, landing_detected, apogee_reached, landedState, initialAltitudeAchieved):
    """
    Process function to log data into a text file which can later be opened with Excel.
    """
    with open("data_log.txt", "w") as f:
        # Write header to file
        f.write("Time,Q_w,Q_x,Q_y,Q_z,a_x,a_y,a_z,temperature,pressure,altitude,accel_magnitude,temperature_2,apogee,battery_percentage,survivability_percentage,detection_time_H,detection_time_M,detection_time_S,max_velocity,landing_velocity,apogee_reached,landedState,initialAltitudeAchieved\n")
        while True:
            current_time = time.strftime('%H:%M:%S', time.localtime())
            # Format data into a string
            data_str = (
                f"{current_time},{shared_imu_data[0]:.2f},{shared_imu_data[1]:.2f},{shared_imu_data[2]:.2f},{shared_imu_data[3]:.2f},"
                f"{shared_imu_data[4]:.2f},{shared_imu_data[5]:.2f},{shared_imu_data[6]:.2f},{shared_imu_data[7]:.2f},{shared_imu_data[8]:.2f},"
                f"{shared_imu_data[9]:.2f},{shared_imu_data[10]:.2f},{shared_rf_data[0]:.2f},{shared_rf_data[1]:.2f},{shared_rf_data[2]:.2f},"
                f"{shared_rf_data[3]:.2f},{shared_rf_data[8]:.0f},{shared_rf_data[9]:.0f},{shared_rf_data[10]:.0f},{shared_rf_data[11]:.2f},"
                f"{shared_rf_data[12]:.2f},{apogee_reached.value:.2f},{int(landedState.value)},{int(initialAltitudeAchieved.value)}\n"
            )
            # Write data to file
            f.write(data_str)
            f.flush()  # Ensure data is written immediately


# Main program
if __name__ == "__main__":
    # Initialize the IMU
    imu = VN100IMU()

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
            print("GPIO 17 is busy, releasing and reinitializing...")
            lgpio.gpio_free(GPIO_ENABLE, 19)  # Free GPIO 17 if busy
            lgpio.gpio_claim_output(GPIO_ENABLE, 19)  # Set GPIO 17 as output again

    lgpio.gpio_write(GPIO_ENABLE, 17, 0)  # Set GPIO 17 to low, DISABLE RF
    lgpio.gpio_write(GPIO_ENABLE, 19, 0)  # Set GPIO 19 to low, DISABLE LATCH

    # Initialize Servo
    servo = ServoController(servo_pin=13)
    #servo.set_servo_angle(servoStartAngle)

    # Start the IMU data process
    imu_process = multiprocessing.Process(target=imu_data_process, args=(imu, shared_imu_data))
    imu_process.start()

    # Start the landing detection process
    landing_process = multiprocessing.Process(target=landing_detection_process, args=(shared_imu_data, landing_detected, landing_detection_time))
    landing_process.start()

    # Start the survivability process
    survivability_process = multiprocessing.Process(target=survivability_process, args=(shared_imu_data, survivability_percentage))
    survivability_process.start()

    # Start the RF data update process
    rf_data_process = multiprocessing.Process(target=update_rf_data_process, args=(
        shared_imu_data, landing_detected, landing_detection_time, shared_rf_data,
        apogee_reached, battry_percentage, survivability_percentage, max_velocity, landing_velocity
    ))
    rf_data_process.start()


    # Start the RF data sending process
    send_rf_data_process = multiprocessing.Process(target=send_rf_data_process, args=(
        shared_rf_data, landing_detected
    ))
    send_rf_data_process.start()

    # Start the data logging process
    data_logging_process = multiprocessing.Process(target=data_logging_process, args=(
        shared_imu_data, shared_rf_data, landing_detected, apogee_reached, landedState, initialAltitudeAchieved
    ))
    data_logging_process.start()

    # Start the Servo Latch process
    release_latch_servo_process = multiprocessing.Process(target=release_latch_servo, args=(servo, landing_detected,))
    release_latch_servo_process.start()

    try:
        # Keep the main process alive and print altitude, acceleration magnitude, and landing detected periodically for debugging
        while True:
            # detection_time = time.strftime('%H:%M:%S', time.localtime(landing_detection_time.value)) if landing_detected.value else "N/A"
            # print(f"Altitude: {shared_imu_data[9]:.2f} m, Acceleration Magnitude: {shared_imu_data[10]:.2f} m/s^2, Landing Detected: {landing_detected.value}, Detection Time: {detection_time}")
            print("RF Shared Data: " + ", ".join(f"{value:.2f}" for value in shared_rf_data))
            # print(f"Pressure: {shared_imu_data[8]:.2f}, Pressure: {shared_imu_data[9]:.2f}")
            # print(survivability_percentage.value)

    except KeyboardInterrupt:
        print("Stopping processes...")
        imu_process.terminate()
        landing_process.terminate()
        survivability_process.terminate()
        rf_data_process.terminate()
        send_rf_data_process.terminate()
        release_latch_servo_process.terminate()
        data_logging_process.terminate()
        imu_process.join()       
        landing_process.join()
        survivability_process.join()
        rf_data_process.join()
        send_rf_data_process.join()
        data_logging_process.join()
        print("Processes stopped.")
        lgpio.gpio_write(GPIO_ENABLE, 17, 0)
        lgpio.gpio_write(GPIO_ENABLE, 19, 0)
        lgpio.gpiochip_close(GPIO_ENABLE)
        #servo.set_servo_angle(servoStartAngle)
        servo.release()
        print("GPIO cleanup and program terminated.")
