"""
Alert Module
=============
Manages visual and audible driver alerts based on the current FSM state.

Requirements:
    FR-ALR-001: Visual alert in WARNING and BRAKE states
    FR-ALR-002: Audible alert in WARNING and BRAKE states
    FR-ALR-003: Buzzer pattern — intermittent in WARNING, continuous in BRAKE
    FR-ALR-004: All alerts off in STANDBY and OFF states

References:
    - Euro NCAP AEB C2C Test Protocol v4.3.1
    - GB/T 33577-2017
"""

from dataclasses import dataclass

from fsm import AEBState


@dataclass
class AlertOutput:
    """Output of one alert update cycle.

    Attributes:
        visual: True if the visual indicator (e.g., HUD icon) is active.
        audible: True if the audible indicator (e.g., buzzer) is active.
        buzzer_pattern: Description of the buzzer behaviour —
            'off', 'intermittent', or 'continuous'.
    """
    visual: bool
    audible: bool
    buzzer_pattern: str


# States that require alerts
_ALERT_STATES = {
    AEBState.WARNING,
    AEBState.BRAKE_L1,
    AEBState.BRAKE_L2,
    AEBState.BRAKE_L3,
}

_BRAKE_STATES = {
    AEBState.BRAKE_L1,
    AEBState.BRAKE_L2,
    AEBState.BRAKE_L3,
}

# Intermittent buzzer toggles every 250 ms (FR-ALR-003)
_BUZZER_TOGGLE_PERIOD: float = 0.250


class AlertModule:
    """Driver alert manager (FR-ALR-001 through FR-ALR-004).

    Tracks visual and audible alert state and generates a buzzer
    pattern appropriate for the current FSM state.
    """

    def __init__(self):
        self.visual_active: bool = False
        self.audible_active: bool = False
        self.alert_start_time: float = 0.0
        self._buzzer_on: bool = False

    def update(self, fsm_state: AEBState, t: float) -> AlertOutput:
        """Update alert outputs based on current FSM state.

        Args:
            fsm_state: Current AEB state from the FSM.
            t: Current simulation time [s], used for intermittent pattern timing.

        Returns:
            AlertOutput with visual/audible flags and buzzer pattern string.
        """
        if fsm_state in _ALERT_STATES:
            self.visual_active = True

            if fsm_state == AEBState.WARNING:
                # Intermittent buzzer: toggle every 250 ms (FR-ALR-003)
                # Use time-based toggling for deterministic behaviour
                cycle_pos = t % (_BUZZER_TOGGLE_PERIOD * 2)
                self._buzzer_on = cycle_pos < _BUZZER_TOGGLE_PERIOD
                self.audible_active = self._buzzer_on
                buzzer_pattern = 'intermittent'
            else:
                # BRAKE states: continuous buzzer (FR-ALR-003)
                self.audible_active = True
                self._buzzer_on = True
                buzzer_pattern = 'continuous'
        else:
            # OFF, STANDBY, POST_BRAKE: all alerts off (FR-ALR-004)
            self.visual_active = False
            self.audible_active = False
            self._buzzer_on = False
            buzzer_pattern = 'off'

        return AlertOutput(
            visual=self.visual_active,
            audible=self.audible_active,
            buzzer_pattern=buzzer_pattern,
        )
