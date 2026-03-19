"""
TTC (Time-To-Collision) Calculator Module
==========================================
Implements collision risk assessment based on kinematic equations.

Requirements:
    FR-DEC-001: TTC = d / v_rel, valid when v_rel > 0.5 m/s
    FR-DEC-002: Braking distance d_brake = v^2 / (2 * a_max)

References:
    - GB/T 33577-2017 (TTC thresholds)
    - MDPI Applied Sciences 13(1):508
    - PMC6864679 Table 4
"""

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

from config import TTCConfig


@dataclass
class TTCResult:
    """Result of a TTC evaluation cycle.

    Attributes:
        ttc: Time-to-collision [s]. float('inf') if not closing.
        d_brake: Braking distance at current ego speed [m].
        is_closing: True if ego vehicle is closing on the target.
        risk_level: Qualitative risk label ('none', 'low', 'medium', 'high', 'critical').
    """
    ttc: float
    d_brake: float
    is_closing: bool
    risk_level: str


def compute_ttc(distance: float, v_rel: float) -> float:
    """Compute constant-velocity TTC (FR-DEC-001).

    TTC = distance / v_rel, valid only when v_rel > 0.5 m/s (closing).
    Result is clamped to [0, 10] seconds.

    Args:
        distance: Gap to target [m]. Must be >= 0.
        v_rel: Relative closing speed [m/s]. Positive means closing.

    Returns:
        TTC in seconds, clamped to [0, 10]. Returns float('inf') if not closing.
    """
    if v_rel <= 0.5:
        return float('inf')

    ttc = distance / v_rel
    return float(np.clip(ttc, 0.0, 10.0))


def compute_braking_distance(v_ego: float, a_max: float = 6.0) -> float:
    """Compute minimum braking distance (FR-DEC-002).

    d_brake = v_ego^2 / (2 * a_max)

    Args:
        v_ego: Ego vehicle speed [m/s].
        a_max: Maximum deceleration magnitude [m/s^2]. Default 6.0.

    Returns:
        Braking distance [m].
    """
    if v_ego <= 0.0 or a_max <= 0.0:
        return 0.0
    return (v_ego ** 2) / (2.0 * a_max)


def compute_ttc_with_accel(distance: float, v_rel: float, a_rel: float) -> float:
    """Compute TTC with relative acceleration (quadratic model).

    Solves: distance + v_rel * t + 0.5 * a_rel * t^2 = 0
    for the smallest positive root.

    Convention: v_rel > 0 means closing, a_rel < 0 means target is decelerating
    (closing faster).

    Args:
        distance: Gap to target [m]. Must be >= 0.
        v_rel: Relative closing speed [m/s]. Positive = closing.
        a_rel: Relative acceleration [m/s^2].

    Returns:
        TTC in seconds (smallest positive root), or float('inf') if no collision.
    """
    # Quadratic: 0.5*a_rel*t^2 + v_rel*t + distance = 0
    # But we want distance to reach zero, so:
    # 0.5*a_rel*t^2 + (-v_rel)*t + distance = 0
    # because positive v_rel means gap is shrinking.
    a = 0.5 * a_rel
    b = -v_rel
    c = distance

    if abs(a) < 1e-9:
        # Linear case
        return compute_ttc(distance, v_rel)

    discriminant = b ** 2 - 4.0 * a * c

    if discriminant < 0:
        return float('inf')

    sqrt_disc = math.sqrt(discriminant)
    t1 = (-b - sqrt_disc) / (2.0 * a)
    t2 = (-b + sqrt_disc) / (2.0 * a)

    # Collect positive roots
    candidates = []
    if t1 > 1e-9:
        candidates.append(t1)
    if t2 > 1e-9:
        candidates.append(t2)

    if not candidates:
        return float('inf')

    ttc = min(candidates)
    return float(np.clip(ttc, 0.0, 10.0))


class TTCCalculator:
    """TTC calculator with configurable thresholds (FR-DEC-001, FR-DEC-002).

    Wraps the module-level functions and provides a unified ``evaluate``
    method that returns a :class:`TTCResult` dataclass.

    Args:
        config: TTCConfig instance with calibration parameters.
    """

    def __init__(self, config: TTCConfig):
        self.config = config

    def compute_ttc(self, distance: float, v_rel: float) -> float:
        """Compute constant-velocity TTC. See module-level :func:`compute_ttc`."""
        if v_rel <= self.config.v_rel_min:
            return float('inf')
        ttc = distance / v_rel
        return float(np.clip(ttc, 0.0, self.config.ttc_max))

    def compute_braking_distance(self, v_ego: float) -> float:
        """Compute braking distance using configured a_max. See :func:`compute_braking_distance`."""
        return compute_braking_distance(v_ego, self.config.a_max_braking)

    def compute_ttc_with_accel(self, distance: float, v_rel: float, a_rel: float) -> float:
        """Compute quadratic TTC. See module-level :func:`compute_ttc_with_accel`."""
        return compute_ttc_with_accel(distance, v_rel, a_rel)

    def evaluate(self, distance: float, v_ego: float, v_target: float) -> TTCResult:
        """Full TTC evaluation for one sensor cycle (FR-DEC-001, FR-DEC-002).

        Args:
            distance: Gap to target [m].
            v_ego: Ego vehicle speed [m/s].
            v_target: Target vehicle speed [m/s].

        Returns:
            TTCResult with ttc, braking distance, closing flag, and risk level.
        """
        v_rel = v_ego - v_target
        is_closing = v_rel > self.config.v_rel_min

        ttc = self.compute_ttc(distance, v_rel)
        d_brake = self.compute_braking_distance(v_ego)

        # Risk classification
        if not is_closing:
            risk_level = 'none'
        elif ttc > 4.0:
            risk_level = 'low'
        elif ttc > 3.0:
            risk_level = 'medium'
        elif ttc > 1.8:
            risk_level = 'high'
        else:
            risk_level = 'critical'

        return TTCResult(
            ttc=ttc,
            d_brake=d_brake,
            is_closing=is_closing,
            risk_level=risk_level,
        )
