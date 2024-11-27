# velocity_estimator.py

class VelocityEstimator:
    def __init__(self, sampling_rate, cutoff_freq):
        """
        Initializes the velocity estimator with the given sampling rate and cutoff frequency.

        Parameters:
        - sampling_rate: Sampling rate in Hz (e.g., 160)
        - cutoff_freq: Desired cutoff frequency in Hz (e.g., 0.001)
        """
        # Sampling interval
        self.T = 1.0 / sampling_rate  # Sampling interval T

        # Time constant tau
        self.tau = 1.0 / (2.0 * 3.141592653589793 * cutoff_freq)  # tau = 1 / (2 * pi * fc)

        # Precompute constants
        self.a0 = 2.0 / (self.T + 2.0 * self.tau)
        self.a1 = (self.T - 2.0 * self.tau) / (self.T + 2.0 * self.tau)

        # Initialize previous altitude and velocity
        self.prev_altitude = 0.0
        self.prev_velocity = 0.0

    def update(self, altitude):
        """
        Updates the velocity estimate based on the current altitude measurement.

        Parameters:
        - altitude: Current altitude measurement (float)

        Returns:
        - velocity: Estimated velocity (float)
        """
        # Compute the velocity using the difference equation
        velocity = (self.a0 * (altitude - self.prev_altitude)
                    - self.a1 * self.prev_velocity)

        # Update previous values for the next call
        self.prev_altitude = altitude
        self.prev_velocity = velocity

        return velocity
