def data_logging_process_test(shared_imu_data, shared_rf_data, landing_detected, apogee_reached, current_velocity, landedState, initialAltitudeAchieved):
    """
    Process function to log data into a text file (data_log_test.txt) which can later be opened with Excel.
    Adds a 3-minute timeout after the 'landing_detected' event is marked True.
    """
    max_velocity.value = 0
    target_frequency = 100  # Hz
    interval = 1 / target_frequency  # Seconds (10ms for 100Hz)
    start_time = time.perf_counter()
    landing_event_time = None  # Variable to track the time of landing detection

    time.sleep(5)

    with open("data_log_test.txt", "w") as f:
        # Write header to file
        f.write("Time,Q_w,Q_x,Q_y,Q_z,a_x,a_y,a_z,temperature,pressure,altitude,accel_magnitude,temperature_2,apogee,battery_percentage,survivability_percentage,detection_time_H,detection_time_M,detection_time_S,max_velocity,landing_velocity,apogee_reached,current_velocity,landedState,initialAltitudeAchieved\n")
        while True:
            current_time = time.perf_counter()  # Update current time

            # Stop logging if timeout after landing_detected has occurred
            if landing_detected.value and landing_event_time is None:
                landing_event_time = current_time  # Mark the time when landing was detected
            
            if landing_event_time and current_time - landing_event_time >= 180:  # Timeout after 3 minutes
                print("Timeout reached. Closing the file.")
                break  # Exit the loop safely
            
            # Perform logging at the specified interval
            if current_time - start_time >= interval:
                # Format data into a string
                data_str = (
                    f"{current_time:.2f},{shared_imu_data[0]:.2f},{shared_imu_data[1]:.2f},{shared_imu_data[2]:.2f},{shared_imu_data[3]:.2f},"
                    f"{shared_imu_data[4]:.2f},{shared_imu_data[5]:.2f},{shared_imu_data[6]:.2f},{shared_imu_data[7]:.2f},{shared_imu_data[8]:.2f},"
                    f"{shared_imu_data[9]:.2f},{shared_imu_data[10]:.2f},{shared_rf_data[0]:.2f},{shared_rf_data[1]:.2f},{shared_rf_data[2]:.2f},"
                    f"{shared_rf_data[3]:.2f},{shared_rf_data[8]:.0f},{shared_rf_data[9]:.0f},{shared_rf_data[10]:.0f},{shared_rf_data[11]:.2f},"
                    f"{shared_rf_data[12]:.2f},{apogee_reached.value:.2f},{current_velocity.value:.2f},{int(landedState.value)},{int(initialAltitudeAchieved.value)}\n"
                )
                # Write data to file
                f.write(data_str)
                f.flush()  # Ensure data is written immediately

                # Update Max Velocity
                if abs(current_velocity.value) >= abs(max_velocity.value):
                    max_velocity.value = current_velocity.value

                start_time = current_time  # Update time step
