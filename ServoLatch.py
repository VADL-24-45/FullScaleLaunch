import time
import lgpio

class ServoController:
    """
    A class to control a PWM servo using lgpio on Raspberry Pi.
    """
    def __init__(self, servo_pin, pwm_frequency=50):
        """
        Initializes the servo controller.
        :param servo_pin: The GPIO pin number connected to the servo.
        :param pwm_frequency: The frequency of the PWM signal (default is 50 Hz for standard servos).
        """
        self.servo_pin = servo_pin
        self.pwm_frequency = pwm_frequency
        self.handle = lgpio.gpiochip_open(0)  # Open the default GPIO chip (0)

        # Claim the pin as output
        lgpio.gpio_claim_output(self.handle, self.servo_pin)

        # Constants for pulse width conversion
        self.min_duty_cycle = 1.0  # Minimum duty cycle (1 ms pulse width)
        self.max_duty_cycle = 2.0  # Maximum duty cycle (2 ms pulse width)

    def set_servo_angle(self, angle):
        """
        Sets the servo to a specific angle by generating a PWM signal.
        :param angle: The desired angle (0 to 180 degrees).
        """
        pulse_width = self.min_duty_cycle + (angle / 180.0) * (self.max_duty_cycle - self.min_duty_cycle)
        duty_cycle = (pulse_width / 20.0) * 100.0  # Convert pulse width to duty cycle percentage
        period = 1 / self.pwm_frequency  # Period in seconds (20 ms for 50 Hz)

        # Generate PWM signal manually for a short time to move the servo
        for _ in range(int(self.pwm_frequency * 0.5)):  # Run for 0.5 seconds
            lgpio.gpio_write(self.handle, self.servo_pin, 1)
            time.sleep(pulse_width / 1000.0)  # High time (pulse width in seconds)
            lgpio.gpio_write(self.handle, self.servo_pin, 0)
            time.sleep((period - pulse_width / 1000.0))  # Low time

    def release(self):
        """
        Releases the GPIO pin and closes the handle.
        """
        lgpio.gpiochip_close(self.handle)
        print("GPIO pin released and handle closed.")

# Usage example
if __name__ == "__main__":
    servo = ServoController(servo_pin=13)

    try:
        while True:
            for angle in [0, 90, 180]:  # Sweep the servo
                print(f"Setting servo to {angle} degrees")
                servo.set_servo_angle(angle)
                time.sleep(1)
    except KeyboardInterrupt:
        print("Stopped by user")
    finally:
        servo.release()
        print("Servo control ended.")
