import multiprocessing
import ctypes
import time
import math
import zmq
from VN100 import VN100IMU, IMUData  # Assuming VN100IMU class and IMUData dataclass are available

# Shared data structure for IMU data (using Array for faster access)
shared_imu_data = multiprocessing.Array(ctypes.c_double, 11)  # Array for Q_w, Q_x, Q_y, Q_z, a_x, a_y, a_z, temperature, pressure, altitude, accel_magnitude

# Shared values for landing detection and survivability
landing_detected = multiprocessing.Value(ctypes.c_bool, False)  # Boolean for landing detection
survivability_percentage = multiprocessing.Value(ctypes.c_double, 0.0)  # Double for survivability percentage

# Threshold values
landingAccMagThreshold = 15  # m/s^2
groundLevel = 159.32  # CHANGE THIS VALUE TO CALIBRATE IMU
landingAltitudeThreshold = groundLevel + 100
initialAltitudeThreshold = groundLevel + 0

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
            shared_data[7] = imu.currentData.temperature
            shared_data[8] = imu.currentData.pressure
            
            # Calculate altitude based on pressure and update shared data
            seaLevelPressure = 101.325  # Standard atmospheric pressure at sea level in hPa
            altitude = 44330.0 * (1.0 - math.pow(shared_data[8] / seaLevelPressure, 0.1903))
            shared_data[9] = altitude

            # Calculate acceleration magnitude and update shared data
            accel_magnitude = math.sqrt(shared_data[4] ** 2 + shared_data[5] ** 2 + shared_data[6] ** 2)
            shared_data[10] = accel_magnitude


# TO-DO: Add timeout timer and photoresistor
def landing_detection_process(shared_data, landing_detected):
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

            # Publish landing detected message
            socket.send_string("Landing Detected")

         
def survivability_process(shared_data, survivability_percentage):
    """
    Process function to calculate and update the survivability percentage.
    Placeholder calculation that could be modified with specific criteria.
    """
    while True:
        # Use calculated acceleration magnitude from shared data
        accel_magnitude = shared_data[10]
        survivability_percentage.value = max(0, 100 - (accel_magnitude / 20) * 100)  # TO-DO: Placeholder logic for survivability

# Main program
if __name__ == "__main__":
    # Initialize the IMU
    imu = VN100IMU()

    # Start the IMU data process
    imu_process = multiprocessing.Process(target=imu_data_process, args=(imu, shared_imu_data))
    imu_process.start()

    # Start the landing detection process
    landing_process = multiprocessing.Process(target=landing_detection_process, args=(shared_imu_data, landing_detected))
    landing_process.start()

    # Start the survivability process
    survivability_process = multiprocessing.Process(target=survivability_process, args=(shared_imu_data, survivability_percentage))
    survivability_process.start()

    try:
        # Keep the main process alive and print altitude, acceleration magnitude, and landing detected periodically for debugging
        while True:
            print(f"Altitude: {shared_imu_data[9]:.2f} m, Acceleration Magnitude: {shared_imu_data[10]:.2f} m/s^2, Landing Detected: {landing_detected.value}")

    except KeyboardInterrupt:
        print("Stopping processes...")
        imu_process.terminate()
        landing_process.terminate()
        survivability_process.terminate()
        imu_process.join()
        landing_process.join()
        survivability_process.join()
        print("Processes stopped.")
