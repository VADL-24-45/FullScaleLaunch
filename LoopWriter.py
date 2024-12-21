import time
from multiprocessing import Process, Value
import os

def rolling_logger(flag, pre_file, time_window=5):
    """
    Logs the last 5 seconds of data into the pre_file.
    """
    deque_buffer = []  # Local buffer for rolling window
    print("Rolling logger started.")
    with open(pre_file, "w") as f:
        while True:
            if flag.value:  # Exit when the flag is set to True
                print("Rolling logger exiting.")
                break

            current_time = time.perf_counter()
            deque_buffer.append(current_time)

            # Remove data older than the time window
            deque_buffer = [t for t in deque_buffer if current_time - t <= time_window]

            # Write the buffer to the pre_file
            f.seek(0)
            f.truncate()
            f.write("\n".join(f"{x:.2f}" for x in deque_buffer) + "\n")
            f.flush()

            print(f"Rolling Logger - Current Buffer: {deque_buffer}")
            time.sleep(1)  # Logging frequency: 1 second


def continuous_logger(flag, post_file):
    """
    Continuously logs data into the post_file after the flag is set to True.
    """
    print("Continuous logger waiting for flag.")
    with open(post_file, "w") as f:
        while not flag.value:  # Wait until the flag is True
            time.sleep(1)

        print("Continuous logger started.")
        while True:
            current_time = time.perf_counter()
            f.write(f"{current_time:.2f}\n")
            f.flush()
            print(f"Continuous Logger - Recorded: {current_time:.2f}")
            time.sleep(1)  # Logging frequency: 1 second


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

    # Shared boolean flag to control process behavior
    flag = Value('b', False)  # Start in rolling logger mode

    try:
        # Start the processes
        rolling_process = Process(target=rolling_logger, args=(flag, pre_file))
        continuous_process = Process(target=continuous_logger, args=(flag, post_file))

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
