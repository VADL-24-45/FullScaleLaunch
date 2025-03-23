from smbus2 import SMBus
import struct
import sys
import time

class I2CSender:
    def __init__(self, addr=0x8, bus_num=1):
        self.addr = addr
        self.bus = SMBus(bus_num)  # indicates /dev/i2c-1
        self.end_marker = -999.0  # Special value to indicate end of 13 floats
        self.active = False

    # Function to send one float at a time over I2C
    def send_float(self, data):
        byte_data = struct.pack('f', data)  # Pack a single float into bytes
        try:
            self.bus.write_i2c_block_data(self.addr, 0, list(byte_data))  # Send float data as bytes
        except OSError as e:
            if e.errno == 121:  # Catch Remote I/O error (Errno 121)
                # print("I/O error occurred, skipping this transmission.")
                pass

    # Function to start the sending process if active is True
    def monitor_and_send(self, float_data):
        if self.active:
            for f in float_data:
                self.send_float(f)  # Send one float at a time
                # print(f"Data sent: {f}")  # Print to console for confirmation
                time.sleep(0.1)  # Small delay between transmissions

            # Send the end marker
            self.send_float(self.end_marker)
            # print("End marker sent")
            time.sleep(0.5)  # Small delay after sending end marker

    # Function to set the active status
    def set_active(self, status):
        self.active = status

    def close(self):  # Gracefully close the I2C bus
        print("Closing I2C bus...")
        self.bus.close()

# Example usage
if __name__ == "__main__":
    sender = I2CSender()
    sender.set_active(True)  # Set active to True to start sending data

    # Replace this with actual message data to be sent
    while True:
        message_data = [1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7, 8.8, 9.9, 10.10, 11.11, 12.12, 13.13, 14, -999]  # Example float data
        sender.monitor_and_send(message_data)
        time.sleep(0.5)  # Delay between sets of transmissions
