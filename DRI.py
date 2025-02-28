import numpy as np
import pandas as pd
from scipy.interpolate import interp1d
import control  

class DRI:       
    def calc_DRI(self, t, accel, omega_n, zeta):
        """
        calc_DRI computes the Dynamic Response Index (DRI) given:
        t        = time vector, seconds
        accel    = vertical acceleration input, m/s^2
        omega_n  = natural frequency for the 2nd order system, rad/s
        zeta     = damping ratio

        Returns:
        DR: array of DRI values over time
        """

        # Ensure arrays are numpy arrays
        t = np.array(t)
        accel = np.array(accel)

        # Determine sampling interval (assuming uniform spacing)
        dt = t[1] - t[0]
        fs = 1 / dt  # sampling frequency (for reference if needed)

        # Optional: low-pass filter can be applied here if desired
        # We skip it:
        accel_f = accel

        # Define state-space matrices
        #   x' = A x + B u
        #   y  = C x + D u
        # Where x = [displacement; velocity]
        A = np.array([
            [0.0,       1.0],
            [-omega_n**2, -2.0 * zeta * omega_n]
        ])
        B = np.array([[0.0], [1.0]])
        C = np.array([[1.0, 0.0]])
        D = np.array([[0.0]])

        # Build state-space system
        sys = control.ss(A, B, C, D)

        # Simulate with zero initial conditions using forced_response
        # forced_response(sys, T, U, X0)
        T_out, y_out, x_out = control.forced_response(
            sys, T=t, U=accel_f, X0=[0, 0],
            return_x=True  # <-- This is the key
        )

        # x_out.shape -> (2, len(T_out)) => row 0 = displacement, row 1 = velocity
        displacement = x_out[0, :]

        # DRI formula: DRI(t) = (omega_n^2 / g) * displacement(t)
        g = 9.81
        DR = (omega_n**2 / g) * displacement

        return DR
