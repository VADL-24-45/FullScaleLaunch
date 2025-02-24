import time
import smbus

class ADS1115:
    POINTER_CONVERSION = 0x00
    POINTER_CONFIGURATION = 0x01
    
    def __init__(self, i2c_bus=6, device_address=0x48):
        self.i2c_bus = i2c_bus
        self.device_address = device_address
        self.bus = smbus.SMBus(self.i2c_bus)

    def swap2Bytes(self, c):
        """Swap byte order for 16-bit values."""
        return (c >> 8 | c << 8) & 0xFFFF

    def LEtoBE(self, c):
        """Convert little-endian to big-endian (signed)."""
        c = self.swap2Bytes(c)
        # If the most significant bit is set, interpret as negative number
        if c >= 2**15:
            c -= 2**16
        return c

    def _read_ads1115(self):
        """Reads a raw value from channel 0 (AIN0) using single-shot mode."""
        # Configuration: Single-shot mode, AIN0, ±4.096V range, 128 SPS
        # Binary: 0b1100000110000011
        conf = self.swap2Bytes(0b1100000110000011)
        self.bus.write_word_data(self.device_address, self.POINTER_CONFIGURATION, conf)

        # Wait for conversion (at least 8ms for 128 SPS)
        time.sleep(0.01)

        # Read conversion result
        raw_value = self.bus.read_word_data(self.device_address, self.POINTER_CONVERSION)
        return self.LEtoBE(raw_value)

    def get_adc(self):
        """
        Public method to read the ADC value in volts from AIN0.
        
        Returns:
            float: The measured voltage on AIN0.
        """
        raw_adc_value = self._read_ads1115()
        # Per ADS1115 datasheet for ±4.096V range, 16-bit reading => 0.1875 mV per bit
        # Convert to volts
        real_adc_value = raw_adc_value * 0.1875 / 1000
        return real_adc_value

    def calc_vbat(self, adc_val):
        """
        Calculate battery voltage assuming a resistor divider from the battery
        to the ADC input.
        
        Args:
            adc_val (float): The measured voltage at the ADC input (V).
            
        Returns:
            float: The calculated battery voltage (V).
        """
        r1 = 330000  # 330 kΩ
        r2 = 100000  # 100 kΩ
        v_divider = r2 / (r1 + r2)
        return adc_val / v_divider
    
    def battery_voltage_to_percentage(self, v_bat):
        if v_bat >= 12.60:
            return 100.00
        elif v_bat <= 11.07:
            return 0.00
        else:
            return (0.00000 * v_bat**0) + (740.78596 * v_bat**1) + (-28.86639 * v_bat**2) + (-4653.88293)

    def cleanup(self):
        """Close the I2C bus."""
        self.bus.close()

if __name__ == "__main__":
    try:
        ads = ADS1115(i2c_bus=6, device_address=0x48)
        while True:
            adc_voltage = ads.get_adc()
            battery_voltage = ads.calc_vbat(adc_voltage)
            battery_percentage = ads.battery_voltage_to_percentage(ads.calc_vbat(ads.get_adc()))
            print(f"ADC Value: {adc_voltage:.6f} V    Battery Value: {battery_voltage:.6f} V Battery Percent: {battery_percentage:.2f}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        ads.cleanup()
