import serial
import time
import struct
from dataclasses import dataclass

@dataclass
class IMUData:
    """Data structure to hold IMU readings."""
    Q_w: float
    Q_x: float
    Q_y: float
    Q_z: float
    a_x: float
    a_y: float
    a_z: float
    temperature: float
    pressure: float

class VN100IMU:
    """
    VN100IMU class to interface with the VectorNav VN-100 IMU sensor.
    
    This class provides methods to initialize the IMU, read data, validate messages,
    and parse data into structured format. It uses serial communication to receive
    data from the IMU.
    """
    
    def __init__(self, port='/dev/ttyAMA0', baudrate=115200, timeout=1):
        """
        Initializes the VN100IMU class with specified serial port parameters.
        
        :param port: Serial port to connect to the IMU (default: '/dev/ttyAMA0').
        :param baudrate: Communication speed (default: 115200).
        :param timeout: Read timeout in seconds (default: 1).
        """
        self.serialConnection = None
        self.currentData = None
        
        # Initialize serial connection
        self.serialConnection = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(2)  # Allow time for the serial connection to initialize
        self.initializeIMU()

    def initializeIMU(self):
        """
        Sends the initialization command to the IMU to configure its output.
        """
        self.serialConnection.write(b"$VNWRG,75,2,5,05,0110,0030*XX\r\n") ## See README
        time.sleep(1)
        print("IMU initialized, starting data monitoring")

    @staticmethod
    def calculateCRC(data, length):
        """
        Calculates the CRC (Cyclic Redundancy Check) for data integrity verification.

        :param data: The data byte array to calculate CRC on.
        :param length: The length of the data.
        :return: Computed CRC value as an integer.
        """
        crc = 0
        for i in range(length):
            crc = (crc >> 8) | (crc << 8) & 0xFFFF
            crc ^= data[i]
            crc ^= (crc & 0xFF) >> 4
            crc ^= (crc << 12) & 0xFFFF
            crc ^= ((crc & 0xFF) << 5) & 0xFFFF
        return crc

    def readData(self):
        """
        Reads data from the serial port, validates the message using CRC, 
        and parses it if valid. If data is valid, it stores it in currentData
        and prints it in a single row format.
        """
        if self.serialConnection.in_waiting > 0:  # Check if there is data in the buffer
            syncByte = self.serialConnection.read(1)  # Read the first byte

            if syncByte == b'\xFA':
                message = self.serialConnection.read(43)  # Read the next 43 bytes

                if len(message) == 43:
                    calculatedChecksum = self.calculateCRC(message[:41], 41)
                    receivedChecksum = int.from_bytes(message[41:43], 'big')

                    if calculatedChecksum == receivedChecksum:
                        self.currentData = self.parseMessage(message)

    def parseMessage(self, message):
        """
        Parses the raw message bytes into an IMUData structure.
        
        :param message: The raw byte message received from the IMU.
        :return: IMUData object containing parsed IMU values.
        """
        # Extract quaternion data
        Q_w = struct.unpack('<f', message[5:9])[0]
        Q_x = struct.unpack('<f', message[9:13])[0]
        Q_y = struct.unpack('<f', message[13:17])[0]
        Q_z = struct.unpack('<f', message[17:21])[0]

        # Extract acceleration data
        a_x = struct.unpack('<f', message[21:25])[0]
        a_y = struct.unpack('<f', message[25:29])[0]
        a_z = struct.unpack('<f', message[29:33])[0]

        # Extract temperature and pressure data
        temp = struct.unpack('<f', message[33:37])[0]
        pres = struct.unpack('<f', message[37:41])[0]

        return IMUData(Q_w=Q_w, Q_x=Q_x, Q_y=Q_y, Q_z=Q_z,
                       a_x=a_x, a_y=a_y, a_z=a_z, temperature=temp, pressure=pres)

    def printDataRow(self):
        """
        Prints the current IMU data in a single row format.
        """
        if self.currentData:
            data = self.currentData
            print(f"{data.Q_w:.4f}, {data.Q_x:.4f}, {data.Q_y:.4f}, {data.Q_z:.4f}, "
                  f"{data.a_x:.4f}, {data.a_y:.4f}, {data.a_z:.4f}, "
                  f"{data.temperature:.4f} °C, {data.pressure:.4f} hPa")

# Usage example
if __name__ == "__main__":
    imu = VN100IMU()
    try:
        while True:
            imu.readData()
            # Access currentData directly from other parts of the program if needed
            if imu.currentData:
                print(f"{imu.currentData.Q_w:.3f}, {imu.currentData.Q_x:.3f}, "
                      f"{imu.currentData.Q_y:.3f}, {imu.currentData.Q_z:.3f}, "
                      f"{imu.currentData.a_x:.3f}, {imu.currentData.a_y:.3f}, "
                      f"{imu.currentData.a_z:.3f}, {imu.currentData.temperature:.3f}, "
                      f"{imu.currentData.pressure:.3f}")
    except KeyboardInterrupt:
        print("Data monitoring interrupted.")
