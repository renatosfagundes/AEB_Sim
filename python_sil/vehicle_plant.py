"""
Vehicle Longitudinal Dynamics Plant Model
==========================================
1-DOF point-mass model for AEB software-in-the-loop simulation.

Equations of motion:

    m * dv/dt = -F_brake - F_drag - F_roll

where:
    F_brake = m * a_brake              (brake force from actuator)
    F_drag  = 0.5 * rho * Cd * Af * v^2   (aerodynamic drag)
    F_roll  = m * g * Cr               (rolling resistance)

The brake actuator is modelled as a first-order lag with dead time:

    G(s) = K * exp(-theta * s) / (tau * s + 1)

Discrete implementation uses a circular buffer for dead time and a
first-order difference equation for the lag.

Ref:
    - MDPI Applied Sciences 13(1):508
    - PMC6864679 (AEB-P Longitudinal Collision Avoidance)
    - MATLAB Vehicle Body 1DOF Longitudinal block
"""

import numpy as np
from collections import deque

from config import VehicleConfig, BrakeActuatorConfig


class BrakeActuator:
    """First-order lag + dead time brake actuator model.

    Transfer function (continuous):
        G(s) = K * exp(-theta * s) / (tau * s + 1)

    Discrete implementation:
        - Dead time: circular buffer of length ceil(dead_time / dt)
        - First-order lag: y(k) = y(k-1) + (dt / tau) * (u_delayed(k) - y(k-1))

    Parameters
    ----------
    config : BrakeActuatorConfig
        Actuator parameters (tau, dead_time, max_decel, cmd_to_decel_gain).
    dt : float
        Integration timestep [s].
    """

    def __init__(self, config: BrakeActuatorConfig, dt: float) -> None:
        self.tau = config.tau
        self.dead_time = config.dead_time
        self.max_decel = config.max_decel
        self.max_brake_cmd = config.max_brake_cmd
        self.cmd_to_decel_gain = config.cmd_to_decel_gain
        self.dt = dt

        # Circular buffer for dead time delay
        # Number of delay samples = ceil(dead_time / dt)
        self._n_delay = max(int(np.ceil(self.dead_time / self.dt)), 1)
        self._buffer: deque = deque([0.0] * self._n_delay, maxlen=self._n_delay)

        # First-order filter state (actual deceleration output) [m/s^2]
        self._y: float = 0.0

    def step(self, cmd_percent: float) -> float:
        """Advance the actuator model by one timestep.

        Parameters
        ----------
        cmd_percent : float
            Brake command in [0, 100] %.

        Returns
        -------
        float
            Actual braking deceleration [m/s^2] (positive value).

        Notes
        -----
        Pipeline:
            1. Clamp input to [0, max_brake_cmd].
            2. Convert command to desired deceleration via gain:
               u = cmd_percent * cmd_to_decel_gain
            3. Push u into the dead-time circular buffer; pop the delayed value.
            4. Apply first-order lag:
               y(k) = y(k-1) + (dt/tau) * (u_delayed - y(k-1))
            5. Clamp output to [0, max_decel].
        """
        # 1. Clamp input
        cmd_clamped = np.clip(cmd_percent, 0.0, self.max_brake_cmd)

        # 2. Convert to desired deceleration [m/s^2]
        u = cmd_clamped * self.cmd_to_decel_gain

        # 3. Dead time via circular buffer
        u_delayed = self._buffer[0]       # oldest sample = delayed output
        self._buffer.append(u)            # push new sample (auto-pops oldest)

        # 4. First-order lag
        alpha = self.dt / self.tau
        self._y = self._y + alpha * (u_delayed - self._y)

        # 5. Clamp output
        self._y = np.clip(self._y, 0.0, self.max_decel)

        return float(self._y)

    @property
    def output(self) -> float:
        """Current actuator output [m/s^2]."""
        return self._y

    def reset(self) -> None:
        """Reset actuator to zero state."""
        self._buffer = deque([0.0] * self._n_delay, maxlen=self._n_delay)
        self._y = 0.0


class VehiclePlant:
    """1-DOF longitudinal vehicle dynamics model.

    Equation of motion:
        m * dv/dt = -F_brake - F_drag - F_roll

    where:
        F_brake = m * a_brake
        F_drag  = 0.5 * rho * Cd * Af * v^2
        F_roll  = m * g * Cr

    State variables:
        x : position [m]
        v : velocity [m/s] (clamped >= 0, no reverse)
        a : acceleration [m/s^2]

    Integration: forward Euler at dt (default 1 kHz).

    Parameters
    ----------
    config : VehicleConfig
        Vehicle parameters (mass, cd, cr, frontal_area, air_density, g).
    brake_config : BrakeActuatorConfig
        Brake actuator parameters.
    dt : float, optional
        Integration timestep [s]. Default 0.001 (1 kHz).
    """

    def __init__(
        self,
        config: VehicleConfig,
        brake_config: BrakeActuatorConfig,
        dt: float = 0.001,
    ) -> None:
        # Vehicle parameters
        self._mass = config.mass
        self._cd = config.cd
        self._cr = config.cr
        self._af = config.frontal_area
        self._rho = config.air_density
        self._g = config.g
        self._v_min = config.v_min
        self._dt = dt

        # Brake actuator sub-model
        self._brake = BrakeActuator(brake_config, dt)

        # State
        self._x: float = 0.0   # position [m]
        self._v: float = 0.0   # velocity [m/s]
        self._a: float = 0.0   # acceleration [m/s^2]

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------
    @property
    def x(self) -> float:
        """Position [m]."""
        return self._x

    @property
    def v(self) -> float:
        """Velocity [m/s]."""
        return self._v

    @property
    def a(self) -> float:
        """Acceleration [m/s^2]."""
        return self._a

    # ------------------------------------------------------------------
    # Public methods
    # ------------------------------------------------------------------
    def apply_brake_command(self, cmd_percent: float, dt: float) -> float:
        """Pass a brake command through the actuator model.

        Parameters
        ----------
        cmd_percent : float
            Brake command [0-100] %.
        dt : float
            Timestep [s] (used for consistency; actuator uses its own dt).

        Returns
        -------
        float
            Actual braking deceleration [m/s^2] (positive value).
        """
        return self._brake.step(cmd_percent)

    def step(self, dt: float) -> None:
        """Integrate the vehicle dynamics by one timestep.

        Uses forward Euler integration:
            a(k)   = -(a_brake + a_drag + a_roll)
            v(k+1) = v(k) + a(k) * dt
            x(k+1) = x(k) + v(k) * dt

        The brake deceleration comes from the current actuator output
        (updated separately via :meth:`apply_brake_command`).

        Parameters
        ----------
        dt : float
            Integration timestep [s].

        Notes
        -----
        Forces:
            F_brake = m * a_brake           [N]
            F_drag  = 0.5 * rho * Cd * Af * v^2  [N]
            F_roll  = m * g * Cr            [N]

        All forces oppose forward motion, so:
            a_total = -(a_brake + F_drag/m + g*Cr)

        Velocity is clamped to >= 0 (vehicle cannot reverse under braking).
        """
        v = self._v
        a_brake = self._brake.output  # [m/s^2], positive

        # Aerodynamic drag deceleration: F_drag / m
        a_drag = 0.5 * self._rho * self._cd * self._af * v * v / self._mass

        # Rolling resistance deceleration: g * Cr
        a_roll = self._g * self._cr

        # Total acceleration (negative = decelerating)
        self._a = -(a_brake + a_drag + a_roll)

        # If vehicle is already stopped, no further deceleration
        if v <= self._v_min and self._a < 0.0:
            self._a = 0.0

        # Forward Euler integration
        self._x += v * dt
        self._v += self._a * dt

        # Clamp velocity (no reverse)
        if self._v < self._v_min:
            self._v = self._v_min

    def get_state(self) -> dict:
        """Return the current vehicle state as a dictionary.

        Returns
        -------
        dict
            Keys: ``x`` [m], ``v`` [m/s], ``a`` [m/s^2], ``brake_actual`` [m/s^2].
        """
        return {
            "x": self._x,
            "v": self._v,
            "a": self._a,
            "brake_actual": self._brake.output,
        }

    def reset(self, x0: float = 0.0, v0: float = 0.0) -> None:
        """Reset the vehicle state and actuator.

        Parameters
        ----------
        x0 : float
            Initial position [m].
        v0 : float
            Initial velocity [m/s].
        """
        self._x = x0
        self._v = v0
        self._a = 0.0
        self._brake.reset()
