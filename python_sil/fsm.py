"""
AEB Finite State Machine (7-state)
===================================
Implements the decision logic that maps TTC and braking distance
assessments to intervention levels.

Requirements:
    FR-FSM-001: 7-state machine (OFF, STANDBY, WARNING, BRAKE_L1-L3, POST_BRAKE)
    FR-FSM-002: TTC-based state transitions with defined thresholds
    FR-FSM-003: Escalation is immediate; de-escalation uses debounce
    FR-FSM-004: De-escalation debounce of 200 ms
    FR-FSM-005: Fault -> OFF
    FR-DEC-006: Brake pedal override -> STANDBY
    FR-DEC-007: Steering override (|angle| > 5 deg) -> STANDBY
    FR-DEC-008: Speed range [10, 60] km/h = [2.78, 16.67] m/s
    FR-ALR-003: WARNING must last >= 800 ms before BRAKE_L1
    FR-BRK-005: POST_BRAKE holds brake for 2.0 s then -> STANDBY

References:
    - GB/T 33577-2017
    - MDPI Applied Sciences 13(1):508
    - Euro NCAP AEB C2C v4.3.1
"""

from dataclasses import dataclass
from enum import IntEnum

from config import FSMConfig


class AEBState(IntEnum):
    """AEB FSM states (FR-FSM-001)."""
    OFF = 0
    STANDBY = 1
    WARNING = 2
    BRAKE_L1 = 3
    BRAKE_L2 = 4
    BRAKE_L3 = 5
    POST_BRAKE = 6


@dataclass
class FSMOutput:
    """Output of one FSM update cycle.

    Attributes:
        state: Current AEB state.
        target_decel: Commanded deceleration magnitude [m/s^2] (>= 0).
        alert_visual: True if visual alert should be active.
        alert_audible: True if audible alert should be active.
        brake_active: True if braking is commanded.
    """
    state: AEBState
    target_decel: float
    alert_visual: bool
    alert_audible: bool
    brake_active: bool


# Mapping from state to target deceleration — filled from config at runtime.
_BRAKE_STATES = {AEBState.BRAKE_L1, AEBState.BRAKE_L2, AEBState.BRAKE_L3}


class AEBStateMachine:
    """7-state AEB finite state machine (FR-FSM-001 through FR-FSM-005).

    Args:
        config: FSMConfig with all thresholds and timing parameters.
    """

    def __init__(self, config: FSMConfig):
        self.cfg = config

        # Current / previous state
        self.current_state: AEBState = AEBState.STANDBY
        self.previous_state: AEBState = AEBState.STANDBY

        # Timers
        self.state_timer: float = 0.0          # Time spent in current state [s]
        self.warning_entry_time: float = 0.0   # Accumulated time in WARNING [s]
        self.debounce_timer: float = 0.0       # De-escalation debounce accumulator [s]
        self.post_brake_timer: float = 0.0     # POST_BRAKE hold timer [s]

        # Deceleration look-up keyed by state
        self._decel_map = {
            AEBState.OFF: 0.0,
            AEBState.STANDBY: 0.0,
            AEBState.WARNING: self.cfg.decel_warning,
            AEBState.BRAKE_L1: self.cfg.decel_brake_l1,
            AEBState.BRAKE_L2: self.cfg.decel_brake_l2,
            AEBState.BRAKE_L3: self.cfg.decel_brake_l3,
            AEBState.POST_BRAKE: self.cfg.decel_brake_l3,  # hold last decel until stop
        }

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _ttc_to_desired_state(self, ttc: float, d_brake: float, distance: float) -> AEBState:
        """Map TTC (and braking distance criterion) to desired FSM state (FR-FSM-002).

        The braking-distance criterion escalates the state if d_brake >= distance
        regardless of TTC.
        """
        # Braking-distance override: if we can't stop in time, go to at least BRAKE_L1
        bd_escalate = d_brake >= distance

        if ttc <= self.cfg.ttc_brake_l3:
            return AEBState.BRAKE_L3
        if ttc <= self.cfg.ttc_brake_l2 or (bd_escalate and ttc <= self.cfg.ttc_brake_l1):
            return AEBState.BRAKE_L2
        if ttc <= self.cfg.ttc_brake_l1:
            return AEBState.BRAKE_L1
        if ttc <= self.cfg.ttc_warning:
            if bd_escalate:
                return AEBState.BRAKE_L1
            return AEBState.WARNING
        # TTC > warning threshold
        if bd_escalate:
            return AEBState.WARNING
        return AEBState.STANDBY

    def _set_state(self, new_state: AEBState) -> None:
        """Transition to *new_state*, resetting timers as needed."""
        if new_state != self.current_state:
            self.previous_state = self.current_state
            self.current_state = new_state
            self.state_timer = 0.0
            self.debounce_timer = 0.0
            if new_state == AEBState.WARNING:
                self.warning_entry_time = 0.0
            if new_state == AEBState.POST_BRAKE:
                self.post_brake_timer = 0.0

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def update(
        self,
        ttc: float,
        d_brake: float,
        distance: float,
        v_ego: float,
        brake_pedal: bool,
        steering_angle: float,
        fault: bool,
        dt: float,
    ) -> FSMOutput:
        """Execute one FSM cycle.

        Args:
            ttc: Current time-to-collision [s].
            d_brake: Current braking distance [m].
            distance: Current gap to target [m].
            v_ego: Ego vehicle speed [m/s].
            brake_pedal: True if driver is pressing the brake.
            steering_angle: Current steering wheel angle [deg].
            fault: True if a system fault is active.
            dt: Time step [s].

        Returns:
            FSMOutput with state, deceleration command, and alert flags.
        """
        # Advance timers
        self.state_timer += dt

        # ---- Priority 1: Fault -> OFF (FR-FSM-005) ----
        if fault:
            self._set_state(AEBState.OFF)
            return self._build_output()

        # ---- Priority 2: Speed range check (FR-DEC-008) ----
        if v_ego < self.cfg.v_ego_min or v_ego > self.cfg.v_ego_max:
            self._set_state(AEBState.STANDBY)
            return self._build_output()

        # ---- Priority 3: Driver override (FR-DEC-006, FR-DEC-007) ----
        if brake_pedal or abs(steering_angle) > self.cfg.steering_override_deg:
            self._set_state(AEBState.STANDBY)
            return self._build_output()

        # ---- POST_BRAKE logic (FR-BRK-005) ----
        if self.current_state == AEBState.POST_BRAKE:
            self.post_brake_timer += dt
            if self.post_brake_timer >= self.cfg.post_brake_hold_time:
                self._set_state(AEBState.STANDBY)
            return self._build_output()

        # ---- Vehicle stopped while braking -> POST_BRAKE ----
        if self.current_state in _BRAKE_STATES and v_ego <= 0.05:
            self._set_state(AEBState.POST_BRAKE)
            return self._build_output()

        # ---- TTC-based state determination ----
        desired = self._ttc_to_desired_state(ttc, d_brake, distance)

        is_escalation = desired > self.current_state
        is_deescalation = desired < self.current_state

        if is_escalation:
            # Escalation is immediate (FR-FSM-003), except:
            # WARNING must last >= 800 ms before transitioning to BRAKE_L1 (FR-ALR-003)
            if self.current_state == AEBState.WARNING and desired >= AEBState.BRAKE_L1:
                self.warning_entry_time += dt
                if self.warning_entry_time < self.cfg.warning_to_brake_min:
                    # Stay in WARNING until minimum time elapsed
                    return self._build_output()
            self.debounce_timer = 0.0
            self._set_state(desired)

        elif is_deescalation:
            # De-escalation requires debounce (FR-FSM-004)
            self.debounce_timer += dt
            if self.debounce_timer >= self.cfg.hysteresis_time:
                self._set_state(desired)
            # else: stay in current (higher) state

        else:
            # Same state — reset debounce, accumulate warning time
            self.debounce_timer = 0.0
            if self.current_state == AEBState.WARNING:
                self.warning_entry_time += dt

        return self._build_output()

    def _build_output(self) -> FSMOutput:
        """Construct FSMOutput from current state."""
        state = self.current_state
        target_decel = self._decel_map.get(state, 0.0)
        brake_active = state in _BRAKE_STATES or state == AEBState.POST_BRAKE
        alert_visual = state in (AEBState.WARNING, *_BRAKE_STATES)
        alert_audible = state in (AEBState.WARNING, *_BRAKE_STATES)

        return FSMOutput(
            state=state,
            target_decel=target_decel,
            alert_visual=alert_visual,
            alert_audible=alert_audible,
            brake_active=brake_active,
        )

    def reset(self) -> None:
        """Reset FSM to initial STANDBY state."""
        self.current_state = AEBState.STANDBY
        self.previous_state = AEBState.STANDBY
        self.state_timer = 0.0
        self.warning_entry_time = 0.0
        self.debounce_timer = 0.0
        self.post_brake_timer = 0.0
