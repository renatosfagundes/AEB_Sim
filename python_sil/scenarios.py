"""
Euro NCAP AEB C2C Test Scenarios
=================================
Based on Euro NCAP AEB C2C Test Protocol v4.3.1 (Feb 2024)
and UNECE R152 test conditions.

Three scenarios:
  CCRs - Car-to-Car Rear Stationary
  CCRm - Car-to-Car Rear Moving
  CCRb - Car-to-Car Rear Braking
"""
from dataclasses import dataclass
from typing import Callable, Optional
import numpy as np


@dataclass
class ScenarioConfig:
    """Configuration for a single test scenario."""
    name: str
    description: str

    # Ego vehicle initial conditions
    ego_x0: float          # Initial position [m]
    ego_v0: float          # Initial velocity [m/s]

    # Target vehicle initial conditions
    target_x0: float       # Initial position [m]
    target_v0: float       # Initial velocity [m/s]

    # Target behavior
    target_accel: float = 0.0       # Constant acceleration [m/s^2] (negative = braking)
    target_brake_onset: float = 0.0  # Time at which target starts braking [s]

    # Simulation
    t_max: float = 15.0    # Max simulation time [s]

    # Pass criteria (NFR-VAL-001..003)
    max_residual_speed: Optional[float] = None  # Max impact speed for pass [m/s]
    min_speed_reduction: Optional[float] = None  # Min speed reduction [m/s]


class TargetVehicle:
    """
    Target vehicle model (non-controlled, follows predefined trajectory).

    For CCRs: stationary (v=0, a=0)
    For CCRm: constant velocity
    For CCRb: constant velocity then brakes at specified time
    """

    def __init__(self, x0: float, v0: float, accel: float = 0.0,
                 brake_onset: float = 0.0):
        self.x = x0
        self.v = v0
        self.v0 = v0
        self.accel = accel
        self.brake_onset = brake_onset
        self._braking = False

    def step(self, t: float, dt: float) -> None:
        """Advance target vehicle by one timestep."""
        # Check if braking should start
        if self.accel < 0 and t >= self.brake_onset:
            self._braking = True

        a = self.accel if self._braking else 0.0

        # Integrate velocity
        self.v = max(0.0, self.v + a * dt)

        # Integrate position
        self.x += self.v * dt

    def get_state(self) -> dict:
        return {"x": self.x, "v": self.v, "braking": self._braking}

    def reset(self, x0: float, v0: float):
        self.x = x0
        self.v = v0
        self._braking = False


# ============================================================
# PREDEFINED SCENARIOS
# ============================================================

def get_ccrs_scenario(ego_speed_kmh: float = 40.0,
                      initial_gap: float = 100.0) -> ScenarioConfig:
    """
    CCRs: Car-to-Car Rear Stationary
    Euro NCAP: VUT at 10-50 km/h, GVT stationary.

    Pass: Full stop OR residual velocity < 5 km/h (NFR-VAL-001)
    """
    v0 = ego_speed_kmh / 3.6  # Convert to m/s
    return ScenarioConfig(
        name=f"CCRs_{ego_speed_kmh:.0f}kmh",
        description=f"Ego at {ego_speed_kmh} km/h approaching stationary target",
        ego_x0=0.0,
        ego_v0=v0,
        target_x0=initial_gap,
        target_v0=0.0,
        target_accel=0.0,
        t_max=15.0,
        max_residual_speed=5.0 / 3.6,  # 5 km/h in m/s
    )


def get_ccrm_scenario(ego_speed_kmh: float = 50.0,
                      target_speed_kmh: float = 20.0,
                      initial_gap: float = 100.0) -> ScenarioConfig:
    """
    CCRm: Car-to-Car Rear Moving
    Euro NCAP: VUT at 30-80 km/h, GVT at 20 km/h.

    Pass: Collision avoided OR impact speed reduced by >= 20 km/h (NFR-VAL-002)
    """
    v_ego = ego_speed_kmh / 3.6
    v_target = target_speed_kmh / 3.6
    return ScenarioConfig(
        name=f"CCRm_{ego_speed_kmh:.0f}vs{target_speed_kmh:.0f}kmh",
        description=(f"Ego at {ego_speed_kmh} km/h approaching target "
                     f"at {target_speed_kmh} km/h"),
        ego_x0=0.0,
        ego_v0=v_ego,
        target_x0=initial_gap,
        target_v0=v_target,
        target_accel=0.0,
        t_max=15.0,
        min_speed_reduction=20.0 / 3.6,  # 20 km/h in m/s
    )


def get_ccrb_scenario(ego_speed_kmh: float = 50.0,
                      target_speed_kmh: float = 50.0,
                      target_decel: float = -2.0,
                      initial_gap: float = 40.0,
                      brake_onset: float = 2.0) -> ScenarioConfig:
    """
    CCRb: Car-to-Car Rear Braking
    Euro NCAP: Both at 50 km/h, GVT decelerates at -2 or -6 m/s^2.
    Initial headway: 12m or 40m.
    GVT deceleration reached within 1.0 s of onset.

    Pass: Collision avoided OR impact speed < 15 km/h (NFR-VAL-003)
    """
    v_ego = ego_speed_kmh / 3.6
    v_target = target_speed_kmh / 3.6
    return ScenarioConfig(
        name=f"CCRb_{target_decel:.0f}ms2_gap{initial_gap:.0f}m",
        description=(f"Both at {ego_speed_kmh} km/h, target brakes at "
                     f"{target_decel} m/s^2 after {brake_onset}s, gap={initial_gap}m"),
        ego_x0=0.0,
        ego_v0=v_ego,
        target_x0=initial_gap,
        target_v0=v_target,
        target_accel=target_decel,
        target_brake_onset=brake_onset,
        t_max=15.0,
        max_residual_speed=15.0 / 3.6,  # 15 km/h in m/s
    )


def get_all_scenarios() -> list:
    """Returns the standard Euro NCAP test matrix."""
    scenarios = []

    # CCRs: ego at 20, 30, 40, 50 km/h
    for v in [20, 30, 40, 50]:
        scenarios.append(get_ccrs_scenario(ego_speed_kmh=v))

    # CCRm: ego at 50 km/h, target at 20 km/h
    scenarios.append(get_ccrm_scenario(50, 20))

    # CCRb: -2 m/s^2 @ 12m and 40m, -6 m/s^2 @ 12m and 40m
    for decel in [-2.0, -6.0]:
        for gap in [12.0, 40.0]:
            scenarios.append(get_ccrb_scenario(
                target_decel=decel, initial_gap=gap, brake_onset=2.0
            ))

    return scenarios


def evaluate_scenario_result(scenario: ScenarioConfig,
                             collision: bool,
                             impact_speed: float,
                             initial_speed: float,
                             final_distance: float) -> dict:
    """
    Evaluate whether a scenario passed or failed.

    Returns dict with: passed, collision, impact_speed, speed_reduction,
                       speed_reduction_pct, final_distance
    """
    speed_reduction = initial_speed - impact_speed if collision else initial_speed
    speed_reduction_pct = (speed_reduction / initial_speed * 100.0
                          if initial_speed > 0 else 100.0)

    passed = False
    reason = ""

    if not collision:
        passed = True
        reason = "Collision avoided"
    elif scenario.max_residual_speed is not None:
        if impact_speed <= scenario.max_residual_speed:
            passed = True
            reason = f"Impact speed {impact_speed*3.6:.1f} km/h <= {scenario.max_residual_speed*3.6:.1f} km/h"
        else:
            reason = f"Impact speed {impact_speed*3.6:.1f} km/h > {scenario.max_residual_speed*3.6:.1f} km/h"
    elif scenario.min_speed_reduction is not None:
        if speed_reduction >= scenario.min_speed_reduction:
            passed = True
            reason = f"Speed reduction {speed_reduction*3.6:.1f} km/h >= {scenario.min_speed_reduction*3.6:.1f} km/h"
        else:
            reason = f"Speed reduction {speed_reduction*3.6:.1f} km/h < {scenario.min_speed_reduction*3.6:.1f} km/h"

    return {
        "passed": passed,
        "reason": reason,
        "collision": collision,
        "impact_speed_ms": impact_speed,
        "impact_speed_kmh": impact_speed * 3.6,
        "speed_reduction_ms": speed_reduction,
        "speed_reduction_kmh": speed_reduction * 3.6,
        "speed_reduction_pct": speed_reduction_pct,
        "final_distance_m": final_distance,
    }
