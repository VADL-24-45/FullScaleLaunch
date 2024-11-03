import multiprocessing
import time
import ctypes
from VN100 import VN100IMU, IMUData  # Assuming VN100IMU class and IMUData dataclass are available

# Shared data structure for IMU data (using Array for faster access)
shared_imu_data = multiprocessing.Array(ctypes.c_double, 9)  # Array for Q_w, Q_x, Q_y, Q_z, a_x, a_y, a_z, temperature, pressure

# Shared values for landing detection and survivability
landing_detected = multiprocessing.Value(ctypes.c_bool, False)  # Boolean for landing detection
survivability_percentage = multiprocessing.Value(ctypes.c_double, 0.0)  # Double for survivability percentage

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

def landing_detection_process(shared_data, landing_detected):
    """
    Process function to monitor IMU data for landing detection condition.
    Sets landing_detected to True if the acceleration magnitude exceeds 10, otherwise False.
    """
    while True:
        # Calculate the acceleration magnitude
        accel_magnitude = (shared_data[4] ** 2 + shared_data[5] ** 2 + shared_data[6] ** 2) ** 0.5
        print(f"Acceleration Magnitude: {accel_magnitude:.3f}")
        
        # Set landing detection flag based on threshold
        landing_detected.value = accel_magnitude > 10

        # Optional sleep to control process frequency
        time.sleep(1 / 160)

def survivability_process(shared_data, survivability_percentage):
    """
    Process function to calculate and update the survivability percentage.
    Placeholder calculation that could be modified with specific criteria.
    """
    while True:
        # Example calculation based on placeholder logic
        accel_magnitude = (shared_data[4] ** 2 + shared_data[5] ** 2 + shared_data[6] ** 2) ** 0.5
        survivability_percentage.value = max(0, 100 - (accel_magnitude / 20) * 100)  # Placeholder logic for survivability

        print(f"Survivability Percentage: {survivability_percentage.value:.2f}%")

        # Optional sleep to control process frequency
        time.sleep(1 / 160)

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
        # Keep the main process alive and print landing detection and survivability status periodically
        while True:
            print(f"Landing Detected: {landing_detected.value}, Survivability: {survivability_percentage.value:.2f}%")

    except KeyboardInterrupt:
        print("Stopping processes...")
        imu_process.terminate()
        landing_process.terminate()
        survivability_process.terminate()
        imu_process.join()
        landing_process.join()
        survivability_process.join()
        print("Processes stopped.")
