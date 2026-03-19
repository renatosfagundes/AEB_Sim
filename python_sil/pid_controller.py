"""
PID Brake Controller
=====================
Closed-loop controller that converts a target deceleration into a
brake command percentage [0-100%], with anti-windup and jerk limiting.

Requirements:
    FR-BRK-002: PID control law for brake actuation
    FR-BRK-004: Jerk limiting (|jerk| <= max_jerk)

References:
    - PMC6864679 Table 5 (gain-scheduled PI)
    - MDPI Applied Sciences 13(1):508
"""

import numpy as np

from config import PIDConfig


class PIDController:
    """PID controller for AEB brake actuation (FR-BRK-002).

    Computes a brake command in [0, 100] % from the error between
    target and actual deceleration magnitudes.  Includes integrator
    anti-windup clamping and output rate limiting (jerk control).

    Args:
        config: PIDConfig with gains, limits, and jerk parameters.
    """

    def __init__(self, config: PIDConfig):
        self.cfg = config

        # Internal state
        self.integral: float = 0.0
        self.prev_error: float = 0.0
        self.prev_output: float = 0.0

        # Diagnostic outputs (readable after each compute call)
        self.last_error: float = 0.0
        self.last_p_term: float = 0.0
        self.last_i_term: float = 0.0

    def compute(self, target_decel: float, actual_decel: float, dt: float) -> float:
        """Execute one PID cycle (FR-BRK-002).

        Both *target_decel* and *actual_decel* are positive magnitudes of
        deceleration [m/s^2].

        Args:
            target_decel: Desired deceleration magnitude [m/s^2].
            actual_decel: Measured deceleration magnitude [m/s^2].
            dt: Time step [s].  Must be > 0.

        Returns:
            Brake command in [0, 100] %.
        """
        if dt <= 0.0:
            return self.prev_output

        # Error: positive when we need more braking
        error = target_decel - actual_decel

        # --- Proportional term ---
        p_term = self.cfg.kp * error

        # --- Integral term with anti-windup clamping ---
        self.integral += self.cfg.ki * error * dt
        self.integral = float(np.clip(self.integral, self.cfg.integral_min, self.cfg.integral_max))
        i_term = self.integral

        # --- Derivative term ---
        d_term = self.cfg.kd * (error - self.prev_error) / dt

        # --- Store terms for diagnostics ---
        self.last_p_term = p_term
        self.last_i_term = i_term
        self.last_error = error

        # --- Raw output ---
        raw_output = p_term + i_term + d_term

        # Clamp to valid range
        clamped = float(np.clip(raw_output, self.cfg.output_min, self.cfg.output_max))

        # --- Jerk (rate) limiting (FR-BRK-004) ---
        max_change = self.cfg.max_decel_rate * dt  # max allowed change per step
        delta = clamped - self.prev_output
        if abs(delta) > max_change:
            clamped = self.prev_output + np.sign(delta) * max_change
        # Re-clamp after rate limiting
        clamped = float(np.clip(clamped, self.cfg.output_min, self.cfg.output_max))

        # --- Store state for next cycle ---
        self.prev_error = error
        self.prev_output = clamped

        return clamped

    def reset(self) -> None:
        """Reset internal state (integrator, previous error, previous output)."""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_output = 0.0
