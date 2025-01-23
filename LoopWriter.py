import time
import os
from collections import deque

def data_logging_process_test_2(shared_imu_data, shared_rf_data, initialAltitudeAchieved, landing_detected, apogee_reached, current_velocity, landedState):
    """
    Logging process with launch detection (based on initialAltitudeAchieved) and post-landing timeout.
    Logs pre-launch data (1-minute rolling buffer) and post-launch data into separate files, then combines them.
    """
    # File names
    pre_file = "data_log_test_2_pre.txt"
    post_file = "data_log_test_2_post.txt"
    output_file = "data_log_test_2_combined.txt"

    # Rolling buffer for pre-launch data (1-minute rolling window)
    rolling_buffer = deque(maxlen=600)  # 600 entries for 1 minute at 10 Hz (100 ms interval)
    target_frequency = 10  # Hz
    interval = 1 / target_frequency  # 100 ms

    start_time = time.perf_counter()
    last_logging_time = start_time  # To control logging frequency
    landing_event_time = None  # To track the time of landing detection

    print("Data logging process test 2 started.")

    with open(pre_file, "w") as pre_f, open(post_file, "w") as post_f:
        while True:
            current_time = time.perf_counter()  # Current time

            # Ensure consistent logging frequency
            if current_time - last_logging_time >= interval:
                # Pre-launch logging (rolling buffer) until initialAltitudeAchieved is True
                if not initialAltitudeAchieved.value:
                    # Collect data into rolling buffer
                    rolling_buffer.append(
                        f"{current_time:.2f},{shared_imu_data[0]:.2f},{shared_imu_data[1]:.2f},{shared_imu_data[2]:.2f},{shared_imu_data[3]:.2f},"
                        f"{shared_imu_data[4]:.2f},{shared_imu_data[5]:.2f},{shared_imu_data[6]:.2f},{shared_imu_data[7]:.2f},{shared_imu_data[8]:.2f},"
                        f"{shared_imu_data[9]:.2f},{shared_imu_data[10]:.2f},{shared_rf_data[0]:.2f},{shared_rf_data[1]:.2f},{shared_rf_data[2]:.2f},"
                        f"{shared_rf_data[3]:.2f},{apogee_reached.value:.2f},{current_velocity.value:.2f},{int(landedState.value)}\n"
                    )

                    # Write rolling buffer to pre-file
                    pre_f.seek(0)
                    pre_f.truncate()
                    pre_f.write("".join(rolling_buffer))
                    pre_f.flush()
                else:
                    # Start continuous logging after launch detection
                    post_f.write(
                        f"{current_time:.2f},{shared_imu_data[0]:.2f},{shared_imu_data[1]:.2f},{shared_imu_data[2]:.2f},{shared_imu_data[3]:.2f},"
                        f"{shared_imu_data[4]:.2f},{shared_imu_data[5]:.2f},{shared_imu_data[6]:.2f},{shared_imu_data[7]:.2f},{shared_imu_data[8]:.2f},"
                        f"{shared_imu_data[9]:.2f},{shared_imu_data[10]:.2f},{shared_rf_data[0]:.2f},{shared_rf_data[1]:.2f},{shared_rf_data[2]:.2f},"
                        f"{shared_rf_data[3]:.2f},{apogee_reached.value:.2f},{current_velocity.value:.2f},{int(landedState.value)}\n"
                    )
                    post_f.flush()

                    # Check for landing event
                    if landing_detected.value and landing_event_time is None:
                        landing_event_time = current_time
                        print("Landing detected. Starting post-landing timeout.")

                    # Timeout after 3 minutes post-landing
                    if landing_event_time and current_time - landing_event_time >= 180:
                        print("Timeout reached. Stopping logging process.")
                        break

                # Update the last logging time
                last_logging_time = current_time

    # Combine pre and post files
    combine_files(pre_file, post_file, output_file)
    print(f"Data logging completed. Logs combined into {output_file}")


def combine_files(pre_file, post_file, output_file):
    """
    Combine pre_file and post_file into a single output file, with column headers added.
    """
    # Define column headers
    headers = "Time,Q_w,Q_x,Q_y,Q_z,a_x,a_y,a_z,temperature,pressure,altitude,accel_magnitude,temperature_2,apogee,battery_percentage,survivability_percentage,detection_time_H,detection_time_M,detection_time_S,max_velocity,landing_velocity,apogee_reached, current_velocity, landedState,initialAltitudeAchieved\n"

    with open(output_file, "w") as out_f:
        # Write headers to the output file
        out_f.write(headers)

        # Add contents of the pre-file if it exists
        if os.path.exists(pre_file):
            with open(pre_file, "r") as pre_f:
                out_f.write(pre_f.read())

        # Add contents of the post-file if it exists
        if os.path.exists(post_file):
            with open(post_file, "r") as post_f:
                out_f.write(post_f.read())

    print(f"Combined files into {output_file} with column headers added.")
