import time
from collections import deque
from multiprocessing import Process, Value, Lock
import os

def rolling_logger(shared_buffer, flag, lock, pre_file):
    """
    Logs the last 30 seconds of data into the pre_file.
    """
    time_window =   # 30 seconds rolling window
    with open(pre_file, "w") as f:
        while True:
            if flag.value:  # Exit rolling logging when flag is True
                print("Rolling logger exiting.")
                break

            current_time = time.perf_counter()

            # Update the shared buffer
            with lock:
                shared_buffer.append(current_time)
                while (current_time - shared_buffer[0]) > time_window:
                    shared_buffer.popleft()

                # Write the buffer to the pre_file
                f.seek(0)
                f.truncate()
                f.write("\n".join(f"{x:.2f}" for x in shared_buffer) + "\n")
                f.flush()

            time.sleep(0.1)  # Logging frequency


def continuous_logger(shared_buffer, flag, lock, post_file):
    """
    Continuously logs data into the post_file once the flag is True.
    """
    with open(post_file, "w") as f:
        while not flag.value:  # Wait for flag to turn True
            time.sleep(0.1)

        print("Continuous logger started.")
        with lock:
            # Write any remaining data from the buffer to the post_file
            f.write("\n".join(f"{x:.2f}" for x in shared_buffer) + "\n")
            f.flush()

        # Append new data continuously
        while True:
            current_time = time.perf_counter()
            f.write(f"{current_time:.2f}\n")
            f.flush()
            time.sleep(0.1)  # Logging frequency


def combine_files(pre_file, post_file, output_file):
    """
    Combine pre_file and post_file into a single output file.
    """
    with open(output_file, "w") as out_f:
        if os.path.exists(pre_file):
            with open(pre_file, "r") as pre_f:
                out_f.write(pre_f.read())
        if os.path.exists(post_file):
            with open(post_file, "r") as post_f:
                out_f.write(post_f.read())
    print(f"Combined files into {output_file}")


if __name__ == "__main__":
    pre_file = "loopWriterTest_pre.txt"
    post_file = "loopWriterTest_post.txt"
    output_file = "loopWriterTest_combined.txt"

    # Shared resources
    shared_buffer = deque(maxlen=10000)  # Large buffer for shared data
    flag = Value('b', False)  # Boolean flag to switch modes
    lock = Lock()  # Lock for synchronizing access to the shared buffer

    try:
        # Start the processes
        rolling_process = Process(target=rolling_logger, args=(shared_buffer, flag, lock, pre_file))
        continuous_process = Process(target=continuous_logger, args=(shared_buffer, flag, lock, post_file))

        rolling_process.start()
        continuous_process.start()

        print("Processes started. Enter 'True' to switch modes.")

        # Wait for user input to change the flag
        while True:
            user_input = input("Enter 'True' to switch to continuous logging: ").strip().lower()
            if user_input == "true":
                with flag.get_lock():
                    flag.value = True
                print("Flag switched to True.")
                break

        # Wait for processes to finish
        rolling_process.join()
        continuous_process.join()

    except KeyboardInterrupt:
        print("\nProgram interrupted. Terminating processes...")
        rolling_process.terminate()
        continuous_process.terminate()
        rolling_process.join()
        continuous_process.join()
    finally:
        # Combine files on exit
        combine_files(pre_file, post_file, output_file)
        print("Program exited gracefully.")
