"""
Sensor Models for AEB Simulation
=================================
Radar, LiDAR, and sensor fusion models with realistic noise,
latency, and plausibility checking.

Requirements covered:
    FR-PER-006  Plausibility checks on sensor data
    FR-PER-007  Fault detection (consecutive invalid readings)

References:
    - ZLY Tech 77 GHz radar datasheet
    - Velodyne VLP-16 datasheet
    - PMC9370944 (automotive radar/lidar fusion survey)
"""

from __future__ import annotations

import collections
from dataclasses import dataclass
from typing import Tuple

import numpy as np

from config import SensorConfig, LidarConfig, FusionConfig


# -------------------------------------------------------------------
# Data structure returned by PerceptionModule
# -------------------------------------------------------------------
@dataclass
class PerceptionData:
    """Fused perception output consumed by downstream TTC / FSM modules."""
    distance: float = 0.0
    v_ego: float = 0.0
    v_target: float = 0.0
    v_rel: float = 0.0
    brake_pedal: bool = False
    steering_angle: float = 0.0
    fault: bool = False
    confidence: float = 0.0


# ===================================================================
# 1. RadarSensor  (77 GHz automotive radar)
# ===================================================================
class RadarSensor:
    """Simulates a 77 GHz automotive radar with Gaussian noise and latency.

    Parameters
    ----------
    cfg : SensorConfig
        Noise standard deviations and latency parameters.
    update_period : float
        Radar update period in seconds (default 0.02 s = 50 Hz).
    """

    def __init__(self, cfg: SensorConfig, update_period: float = 0.02) -> None:
        self._cfg = cfg
        self._update_period = update_period

        # Latency buffer length (number of samples to delay)
        self._latency_samples = max(1, round(cfg.radar_latency / update_period))
        self._range_buf: collections.deque = collections.deque(
            maxlen=self._latency_samples
        )
        self._rate_buf: collections.deque = collections.deque(
            maxlen=self._latency_samples
        )

        # Initialise buffers with NaN (no valid measurement yet)
        for _ in range(self._latency_samples):
            self._range_buf.append(np.nan)
            self._rate_buf.append(np.nan)

        # Internal time accumulator & last-held output
        self._t_accum: float = 0.0
        self._last_range: float = np.nan
        self._last_rate: float = np.nan
        self._valid: bool = False

    # -----------------------------------------------------------------
    def update(self, true_range: float, true_range_rate: float, dt: float) -> None:
        """Advance the sensor model by *dt* seconds.

        The radar only produces a new noisy sample every ``update_period``
        seconds; between updates the last measurement is held.

        Parameters
        ----------
        true_range : float
            Ground-truth distance to target [m].
        true_range_rate : float
            Ground-truth range rate (closing speed positive) [m/s].
        dt : float
            Simulation time step [s].
        """
        self._t_accum += dt

        if self._t_accum >= self._update_period:
            self._t_accum -= self._update_period

            # Add Gaussian measurement noise
            noisy_range = true_range + np.random.normal(
                0.0, self._cfg.range_noise_std
            )
            noisy_rate = true_range_rate + np.random.normal(
                0.0, self._cfg.range_rate_noise_std
            )

            # Push into latency ring buffers
            self._range_buf.append(noisy_range)
            self._rate_buf.append(noisy_rate)

            # Pop the oldest (delayed) sample as current output
            self._last_range = self._range_buf[0]
            self._last_rate = self._rate_buf[0]
            self._valid = not (np.isnan(self._last_range) or np.isnan(self._last_rate))

    # -----------------------------------------------------------------
    def get_measurement(self) -> Tuple[float, float, bool]:
        """Return the current (possibly delayed) radar measurement.

        Returns
        -------
        range_meas : float
            Measured distance [m].
        range_rate_meas : float
            Measured range rate [m/s].
        valid : bool
            True when the sensor has produced at least one valid reading
            through its full latency pipeline.
        """
        return self._last_range, self._last_rate, self._valid


# ===================================================================
# 2. LidarSensor  (e.g. Velodyne VLP-16)
# ===================================================================
class LidarSensor:
    """Simulates a LiDAR sensor with high-accuracy range and finite-difference velocity.

    Parameters
    ----------
    cfg : LidarConfig
        Noise, update rate, and latency parameters.
    """

    def __init__(self, cfg: LidarConfig) -> None:
        self._cfg = cfg
        self._update_period = cfg.update_period

        # Latency buffer
        self._latency_samples = max(1, round(cfg.latency / cfg.update_period))
        self._range_buf: collections.deque = collections.deque(
            maxlen=self._latency_samples
        )
        for _ in range(self._latency_samples):
            self._range_buf.append(np.nan)

        # Internal state
        self._t_accum: float = 0.0
        self._last_range: float = np.nan
        self._prev_range: float = np.nan
        self._velocity: float = np.nan
        self._valid: bool = False

    # -----------------------------------------------------------------
    def update(self, true_range: float, dt: float) -> None:
        """Advance the LiDAR model by *dt* seconds.

        Parameters
        ----------
        true_range : float
            Ground-truth distance to target [m].
        dt : float
            Simulation time step [s].
        """
        self._t_accum += dt

        if self._t_accum >= self._update_period:
            self._t_accum -= self._update_period

            noisy_range = true_range + np.random.normal(0.0, self._cfg.range_noise_std)
            self._range_buf.append(noisy_range)

            # Delayed output
            delayed_range = self._range_buf[0]

            # Finite-difference velocity from consecutive delayed samples
            if not np.isnan(self._last_range) and not np.isnan(delayed_range):
                self._velocity = (delayed_range - self._last_range) / self._update_period

            self._prev_range = self._last_range
            self._last_range = delayed_range
            self._valid = not np.isnan(self._last_range)

    # -----------------------------------------------------------------
    def get_measurement(self) -> Tuple[float, bool]:
        """Return the current (possibly delayed) LiDAR range measurement.

        Returns
        -------
        range_meas : float
            Measured distance [m].
        valid : bool
            True when the sensor has warmed up through its latency pipeline.
        """
        return self._last_range, self._valid

    @property
    def velocity(self) -> float:
        """Velocity estimate from finite difference [m/s] (negative = closing)."""
        return self._velocity


# ===================================================================
# 3. SensorFusion  (2-state Kalman filter)
# ===================================================================
class SensorFusion:
    """Fuses radar and LiDAR using a linear discrete Kalman filter.

    State vector: x = [distance, v_rel]^T

    Prediction model (constant-velocity):
        d(k+1)     = d(k) + v_rel(k) * dt
        v_rel(k+1) = v_rel(k)

    Parameters
    ----------
    cfg : FusionConfig
        Process noise and operational flags.
    sensor_cfg : SensorConfig
        Radar noise parameters (for R_radar).
    lidar_cfg : LidarConfig
        LiDAR noise parameters (for R_lidar).
    """

    def __init__(
        self,
        cfg: FusionConfig,
        sensor_cfg: SensorConfig,
        lidar_cfg: LidarConfig,
    ) -> None:
        self._cfg = cfg

        # State and covariance
        self.x: np.ndarray = np.array([100.0, 0.0])  # [d, v_rel]
        self.P: np.ndarray = np.diag([10.0, 5.0])     # initial uncertainty

        # Process noise Q (tunable)
        self._Q_base = np.diag([cfg.process_noise_pos, cfg.process_noise_vel])

        # Measurement noise covariances
        self._R_radar = np.diag([
            sensor_cfg.range_noise_std ** 2,
            sensor_cfg.range_rate_noise_std ** 2,
        ])
        self._R_lidar = np.array([[lidar_cfg.range_noise_std ** 2]])

        # Observation matrices
        self._H_radar = np.array([[1.0, 0.0],
                                   [0.0, 1.0]])   # measures d and v_rel
        self._H_lidar = np.array([[1.0, 0.0]])     # measures d only

        # Sensor availability tracking
        self._radar_active: bool = False
        self._lidar_active: bool = False

    # -----------------------------------------------------------------
    def predict(self, dt: float) -> None:
        """Propagate state and covariance forward by *dt* seconds.

        Parameters
        ----------
        dt : float
            Time step [s].
        """
        # State transition matrix (constant velocity model)
        F = np.array([[1.0, dt],
                       [0.0, 1.0]])

        # Scale process noise with dt
        Q = self._Q_base * dt

        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    # -----------------------------------------------------------------
    def update_radar(self, d_meas: float, v_meas: float) -> None:
        """Incorporate a radar measurement (distance + range rate).

        Parameters
        ----------
        d_meas : float
            Measured distance [m].
        v_meas : float
            Measured range rate [m/s].
        """
        self._radar_active = True
        z = np.array([d_meas, v_meas])
        H = self._H_radar
        R = self._R_radar

        y = z - H @ self.x                          # innovation
        S = H @ self.P @ H.T + R                    # innovation covariance
        K = self.P @ H.T @ np.linalg.inv(S)         # Kalman gain

        self.x = self.x + K @ y
        I = np.eye(2)
        self.P = (I - K @ H) @ self.P

    # -----------------------------------------------------------------
    def update_lidar(self, d_meas: float) -> None:
        """Incorporate a LiDAR measurement (distance only).

        Parameters
        ----------
        d_meas : float
            Measured distance [m].
        """
        self._lidar_active = True
        z = np.array([d_meas])
        H = self._H_lidar
        R = self._R_lidar

        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        I = np.eye(2)
        self.P = (I - K @ H) @ self.P

    # -----------------------------------------------------------------
    def get_fused_state(self) -> Tuple[float, float, float]:
        """Return the current fused estimate and confidence level.

        Returns
        -------
        d : float
            Fused distance estimate [m].
        v_rel : float
            Fused relative velocity estimate [m/s].
        confidence : float
            1.0 if both sensors active, 0.7 if single sensor, 0.0 if none.
        """
        if self._radar_active and self._lidar_active:
            confidence = 1.0
        elif self._radar_active or self._lidar_active:
            confidence = 0.7
        else:
            confidence = 0.0

        return float(self.x[0]), float(self.x[1]), confidence

    # -----------------------------------------------------------------
    def reset_activity_flags(self) -> None:
        """Clear per-cycle sensor activity flags (call once per fusion cycle)."""
        self._radar_active = False
        self._lidar_active = False


# ===================================================================
# 4. PerceptionModule  (FR-PER-006, FR-PER-007)
# ===================================================================
class PerceptionModule:
    """Top-level perception wrapper that orchestrates radar, LiDAR, and fusion.

    Performs plausibility checks and fault detection on every update cycle.

    Parameters
    ----------
    sensor_cfg : SensorConfig
        Radar / ego-speed noise and plausibility limits.
    lidar_cfg : LidarConfig
        LiDAR noise and timing parameters.
    fusion_cfg : FusionConfig
        Kalman filter tuning and operational flags.
    """

    def __init__(
        self,
        sensor_cfg: SensorConfig,
        lidar_cfg: LidarConfig,
        fusion_cfg: FusionConfig,
    ) -> None:
        self._sensor_cfg = sensor_cfg
        self._lidar_cfg = lidar_cfg
        self._fusion_cfg = fusion_cfg

        # Sub-models
        self._radar = RadarSensor(sensor_cfg, update_period=0.02)
        self._lidar = LidarSensor(lidar_cfg)
        self._fusion = SensorFusion(fusion_cfg, sensor_cfg, lidar_cfg)

        # Plausibility & fault counters
        self._consecutive_invalid: int = 0
        self._fault: bool = False

        # Previous-cycle values for rate-of-change checks
        self._prev_distance: float = np.nan
        self._prev_v_ego: float = np.nan

        # Output cache
        self._data = PerceptionData()

    # -----------------------------------------------------------------
    @staticmethod
    def _in_range(value: float, lo: float, hi: float) -> bool:
        """Return True if *value* lies within [lo, hi]."""
        return lo <= value <= hi

    # -----------------------------------------------------------------
    def _plausibility_check(
        self, distance: float, v_ego: float, dt: float
    ) -> bool:
        """Apply plausibility checks per FR-PER-006.

        Checks:
            - Range within [range_min, range_max]
            - Speed within [speed_min, speed_max]
            - Rate-of-change of distance within limits
            - Rate-of-change of speed within limits

        Returns
        -------
        bool
            True if all checks pass.
        """
        cfg = self._sensor_cfg

        # Static bounds
        if not self._in_range(distance, cfg.range_min, cfg.range_max):
            return False
        if not self._in_range(v_ego, cfg.speed_min, cfg.speed_max):
            return False

        # Rate-of-change bounds (skip on first cycle)
        if dt > 0.0 and not np.isnan(self._prev_distance):
            range_rate = abs(distance - self._prev_distance) / dt
            if range_rate > cfg.max_range_rate_of_change:
                return False

        if dt > 0.0 and not np.isnan(self._prev_v_ego):
            speed_rate = abs(v_ego - self._prev_v_ego) / dt
            if speed_rate > cfg.max_speed_rate_of_change:
                return False

        return True

    # -----------------------------------------------------------------
    def update(
        self,
        true_distance: float,
        true_v_rel: float,
        true_v_ego: float,
        brake_pedal: bool,
        steering_angle: float,
        dt: float,
    ) -> None:
        """Run one perception cycle.

        Parameters
        ----------
        true_distance : float
            Ground-truth distance to lead vehicle [m].
        true_v_rel : float
            Ground-truth relative velocity (closing positive) [m/s].
        true_v_ego : float
            Ground-truth ego vehicle speed [m/s].
        brake_pedal : bool
            Driver brake pedal state.
        steering_angle : float
            Current steering wheel angle [deg].
        dt : float
            Simulation time step [s].
        """
        # --- Ego speed with noise -------------------------------------------
        v_ego_meas = true_v_ego + np.random.normal(
            0.0, self._sensor_cfg.ego_speed_noise_std
        )

        # --- Sensor updates --------------------------------------------------
        self._radar.update(true_distance, true_v_rel, dt)
        self._lidar.update(true_distance, dt)

        radar_range, radar_rate, radar_valid = self._radar.get_measurement()
        lidar_range, lidar_valid = self._lidar.get_measurement()

        # --- Fusion ----------------------------------------------------------
        self._fusion.reset_activity_flags()
        self._fusion.predict(dt)

        if radar_valid:
            self._fusion.update_radar(radar_range, radar_rate)
        if lidar_valid:
            self._fusion.update_lidar(lidar_range)

        fused_d, fused_v_rel, confidence = self._fusion.get_fused_state()

        # --- Plausibility check (FR-PER-006) ---------------------------------
        plausible = self._plausibility_check(fused_d, v_ego_meas, dt)

        # --- Fault detection (FR-PER-007) ------------------------------------
        if not plausible:
            self._consecutive_invalid += 1
        else:
            self._consecutive_invalid = 0

        if self._consecutive_invalid >= self._sensor_cfg.fault_cycles_threshold:
            self._fault = True
        else:
            self._fault = False

        # --- Store previous values for next rate-of-change check -------------
        self._prev_distance = fused_d
        self._prev_v_ego = v_ego_meas

        # --- Compute target speed from ego speed and relative velocity -------
        v_target = v_ego_meas - fused_v_rel

        # --- Pack output ------------------------------------------------------
        self._data = PerceptionData(
            distance=fused_d,
            v_ego=v_ego_meas,
            v_target=v_target,
            v_rel=fused_v_rel,
            brake_pedal=brake_pedal,
            steering_angle=steering_angle,
            fault=self._fault,
            confidence=confidence,
        )

    # -----------------------------------------------------------------
    def reset(self) -> None:
        """Reset all sensor models and fusion state for a new scenario."""
        self._radar = RadarSensor(self._sensor_cfg, update_period=0.02)
        self._lidar = LidarSensor(self._lidar_cfg)
        self._fusion = SensorFusion(self._fusion_cfg, self._sensor_cfg, self._lidar_cfg)
        self._consecutive_invalid = 0
        self._fault = False
        self._prev_distance = np.nan
        self._prev_v_ego = np.nan
        self._data = PerceptionData()

    # -----------------------------------------------------------------
    def get_data(self) -> PerceptionData:
        """Return the latest fused perception data.

        Returns
        -------
        PerceptionData
            Dataclass containing distance, velocities, driver inputs,
            fault flag, and fusion confidence.
        """
        return self._data

    # -----------------------------------------------------------------
    def get_raw_sensor_data(self) -> dict:
        """Return raw sensor readings and Kalman filter state for diagnostics.

        Returns
        -------
        dict
            Keys: radar_distance, radar_rate, radar_valid,
                  lidar_distance, lidar_valid,
                  kalman_P_trace (trace of covariance matrix).
        """
        radar_range, radar_rate, radar_valid = self._radar.get_measurement()
        lidar_range, lidar_valid = self._lidar.get_measurement()
        P = self._fusion.P
        return {
            "radar_distance": radar_range if radar_valid else float('nan'),
            "radar_rate": radar_rate if radar_valid else float('nan'),
            "radar_valid": radar_valid,
            "lidar_distance": lidar_range if lidar_valid else float('nan'),
            "lidar_valid": lidar_valid,
            "kalman_P_trace": float(P[0, 0] + P[1, 1]),
        }
