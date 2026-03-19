"""
AEB System Integrator
======================
Main integration module that connects all subsystems and runs the
10ms execution cycle as specified in the sequence diagram.

Execution order per cycle (FR-COD-001, Section 3.2.2):
  1. Read sensors (perception update)
  2. Validate data (plausibility checks)
  3. Compute TTC and braking distance
  4. Evaluate FSM state transition
  5. Compute brake command via PID
  6. Output brake command + alerts
  7. Transmit CAN messages

Ref: AEB_Requisitos_Projeto_final.tex, Section 3.2.2 Execution Cycle
"""
import numpy as np
from dataclasses import dataclass, field
from typing import List, Optional

from config import AEBConfig
from vehicle_plant import VehiclePlant, BrakeActuator
from sensor_model import PerceptionModule
from ttc_calculator import TTCCalculator
from fsm import AEBStateMachine, AEBState
from pid_controller import PIDController
from alert import AlertModule
from can_bus import CANBus, pack_ego_vehicle, pack_radar_target, pack_brake_cmd, pack_fsm_state, pack_alert
from scenarios import TargetVehicle, ScenarioConfig, evaluate_scenario_result


@dataclass
class SimulationStep:
    """Data recorded at each controller cycle."""
    t: float = 0.0

    # Ego vehicle
    ego_x: float = 0.0
    ego_v: float = 0.0
    ego_a: float = 0.0

    # Target vehicle
    target_x: float = 0.0
    target_v: float = 0.0

    # Perception (fused)
    distance: float = 0.0
    v_rel: float = 0.0
    v_ego_sensed: float = 0.0
    sensor_confidence: float = 0.0
    sensor_fault: bool = False

    # Perception — raw sensor readings (for fusion analysis)
    ground_truth_distance: float = 0.0
    radar_distance: float = 0.0
    lidar_distance: float = 0.0
    radar_valid: bool = False
    lidar_valid: bool = False
    kalman_P_trace: float = 0.0  # trace(P) = P[0,0] + P[1,1]

    # TTC
    ttc: float = 0.0
    d_brake: float = 0.0
    is_closing: bool = False

    # FSM
    fsm_state: int = 0
    target_decel: float = 0.0

    # Controller
    brake_cmd: float = 0.0
    brake_actual: float = 0.0

    # PID internals (for controller analysis)
    pid_error: float = 0.0
    pid_p_term: float = 0.0
    pid_i_term: float = 0.0
    jerk: float = 0.0  # d(brake_actual)/dt [m/s³]

    # Alerts
    alert_visual: bool = False
    alert_audible: bool = False

    # CAN
    can_bus_load: float = 0.0


class AEBSimulation:
    """
    Complete AEB simulation integrating all subsystems.

    Architecture:
        [Target Vehicle] --ground truth--> [Sensors+Fusion] --> [Perception]
        [Ego Vehicle]    --ground truth--> [Sensors+Fusion] --> [Perception]
        [Perception] --> [TTC Calculator] --> [FSM] --> [PID] --> [Brake Actuator] --> [Ego Vehicle]
        [FSM] --> [Alert Module]
        All --> [CAN Bus]
    """

    def __init__(self, config: AEBConfig = None):
        self.cfg = config or AEBConfig()

        # Plant models
        self.ego = VehiclePlant(self.cfg.vehicle, self.cfg.brake_actuator)
        self.target = None  # Set per scenario

        # Perception (sensors + fusion)
        self.perception = PerceptionModule(
            self.cfg.sensor, self.cfg.lidar, self.cfg.fusion
        )

        # Decision
        self.ttc_calc = TTCCalculator(self.cfg.ttc)
        self.fsm = AEBStateMachine(self.cfg.fsm)

        # Execution
        self.pid = PIDController(self.cfg.pid)
        self.alert = AlertModule()

        # CAN bus
        self.can = CANBus(self.cfg.can)
        self._register_can_messages()

        # Simulation state
        self.t = 0.0
        self.log: List[SimulationStep] = []
        self._prev_brake_actual: float = 0.0

    def _register_can_messages(self):
        """Register all AEB CAN messages."""
        for msg_id, dlc, period_ms, name in self.cfg.can.messages:
            self.can.register_message(msg_id, dlc, period_ms, name)

    def setup_scenario(self, scenario: ScenarioConfig):
        """Initialize ego and target vehicles for a test scenario."""
        self.ego.reset(scenario.ego_x0, scenario.ego_v0)
        self.target = TargetVehicle(
            scenario.target_x0, scenario.target_v0,
            scenario.target_accel, scenario.target_brake_onset
        )
        self.fsm.reset()
        self.pid.reset()
        self.perception.reset()
        self.t = 0.0
        self.log = []
        self._prev_brake_actual = 0.0

    def run(self, scenario: ScenarioConfig) -> dict:
        """
        Run a complete scenario simulation.

        Returns dict with: log, result, scenario
        """
        self.setup_scenario(scenario)

        dt_plant = self.cfg.sim.dt              # 1 ms
        dt_ctrl = self.cfg.sim.dt_controller    # 10 ms
        t_max = scenario.t_max

        ctrl_accumulator = 0.0
        collision = False
        impact_speed = 0.0

        while self.t < t_max:
            # --- Plant step (1 kHz) ---
            self.target.step(self.t, dt_plant)
            self.ego.step(dt_plant)
            self.t += dt_plant
            ctrl_accumulator += dt_plant

            # --- Check collision ---
            gap = self.target.x - self.ego.x
            if gap <= 0.0 and self.ego.v > 0.0:
                collision = True
                impact_speed = self.ego.v
                break

            # --- Check if ego stopped ---
            if self.ego.v <= 0.001 and self.fsm.state in (
                AEBState.BRAKE_L1, AEBState.BRAKE_L2, AEBState.BRAKE_L3,
                AEBState.POST_BRAKE
            ):
                # Vehicle stopped during braking
                if self.fsm.state == AEBState.POST_BRAKE:
                    # Already in post-brake, check hold time
                    pass
                # Let FSM handle transition to POST_BRAKE

            # --- Controller cycle (100 Hz) ---
            if ctrl_accumulator >= dt_ctrl:
                ctrl_accumulator = 0.0
                step = self._controller_cycle(dt_ctrl)
                self.log.append(step)

        # --- Evaluate result ---
        result = evaluate_scenario_result(
            scenario=scenario,
            collision=collision,
            impact_speed=impact_speed,
            initial_speed=scenario.ego_v0,
            final_distance=self.target.x - self.ego.x if not collision else 0.0,
        )

        return {
            "log": self.log,
            "result": result,
            "scenario": scenario,
        }

    def _controller_cycle(self, dt: float) -> SimulationStep:
        """
        Execute one 10ms controller cycle.
        This is the core AEB algorithm matching the sequence diagram.
        """
        step = SimulationStep(t=self.t)

        # 1. Get ground truth
        true_distance = max(0.0, self.target.x - self.ego.x)
        true_v_rel = self.ego.v - self.target.v  # positive = closing
        true_v_ego = self.ego.v

        # 2. Perception update (sensors + fusion + plausibility)
        self.perception.update(
            true_distance=true_distance,
            true_v_rel=true_v_rel,
            true_v_ego=true_v_ego,
            brake_pedal=False,  # No driver override in simulation
            steering_angle=0.0,
            dt=dt,
        )
        pdata = self.perception.get_data()

        step.ego_x = self.ego.x
        step.ego_v = self.ego.v
        step.ego_a = self.ego.a
        step.target_x = self.target.x
        step.target_v = self.target.v
        step.distance = pdata.distance
        step.v_rel = pdata.v_rel
        step.v_ego_sensed = pdata.v_ego
        step.sensor_confidence = pdata.confidence
        step.sensor_fault = pdata.fault

        # Raw sensor data for fusion analysis
        step.ground_truth_distance = true_distance
        raw = self.perception.get_raw_sensor_data()
        step.radar_distance = raw["radar_distance"]
        step.lidar_distance = raw["lidar_distance"]
        step.radar_valid = raw["radar_valid"]
        step.lidar_valid = raw["lidar_valid"]
        step.kalman_P_trace = raw["kalman_P_trace"]

        # 3. TTC calculation
        ttc_result = self.ttc_calc.evaluate(
            pdata.distance, pdata.v_ego, pdata.v_ego - pdata.v_rel
        )
        step.ttc = ttc_result.ttc
        step.d_brake = ttc_result.d_brake
        step.is_closing = ttc_result.is_closing

        # 4. FSM update
        fsm_output = self.fsm.update(
            ttc=ttc_result.ttc,
            d_brake=ttc_result.d_brake,
            distance=pdata.distance,
            v_ego=pdata.v_ego,
            brake_pedal=pdata.brake_pedal,
            steering_angle=pdata.steering_angle,
            fault=pdata.fault,
            dt=dt,
        )
        step.fsm_state = fsm_output.state.value
        step.target_decel = fsm_output.target_decel

        # 5. PID controller
        if fsm_output.brake_active and fsm_output.target_decel > 0:
            brake_cmd = self.pid.compute(
                target_decel=fsm_output.target_decel,
                actual_decel=abs(self.ego.a),
                dt=dt,
            )
        else:
            brake_cmd = 0.0
            self.pid.reset()

        step.brake_cmd = brake_cmd
        step.pid_error = self.pid.last_error
        step.pid_p_term = self.pid.last_p_term
        step.pid_i_term = self.pid.last_i_term

        # 6. Apply brake command to actuator
        prev_brake_actual = self._prev_brake_actual
        actual_decel = self.ego.apply_brake_command(brake_cmd, dt)
        step.brake_actual = actual_decel
        step.jerk = (actual_decel - prev_brake_actual) / dt if dt > 0 else 0.0
        self._prev_brake_actual = actual_decel

        # 7. Alert update
        alert_output = self.alert.update(fsm_output.state, self.t)
        step.alert_visual = alert_output.visual
        step.alert_audible = alert_output.audible

        # 8. CAN bus transmission
        self._transmit_can(step)
        self.can.tick(self.t, dt)
        step.can_bus_load = self.can.get_bus_load()

        return step

    def _transmit_can(self, step: SimulationStep):
        """Pack and transmit all CAN messages."""
        try:
            # Ego vehicle data (0x100, 10ms)
            ego_data = pack_ego_vehicle(
                step.v_ego_sensed, step.ego_a, 0.0, 0.0
            )
            self.can.transmit(0x100, ego_data, self.t)

            # Radar target data (0x120, 20ms)
            radar_data = pack_radar_target(
                step.distance, step.v_rel, step.ttc
            )
            self.can.transmit(0x120, radar_data, self.t)

            # Brake command (0x080, 10ms)
            brake_data = pack_brake_cmd(
                brake_request=step.brake_cmd > 0,
                brake_pressure=step.brake_cmd,
                mode=step.fsm_state,
                alive=int(self.t / 0.01) % 16,
                crc=0,
            )
            self.can.transmit(0x080, brake_data, self.t)

            # FSM state (0x200, 50ms)
            fsm_data = pack_fsm_state(
                step.fsm_state, 0, step.brake_cmd > 0, step.ttc
            )
            self.can.transmit(0x200, fsm_data, self.t)

            # Alert (0x300, event)
            if step.alert_visual or step.alert_audible:
                alert_data = pack_alert(
                    alert_type=4 if (step.alert_visual and step.alert_audible) else 1,
                    alert_active=True,
                    buzzer_cmd=2 if step.fsm_state >= 3 else 1,
                )
                self.can.transmit(0x300, alert_data, self.t)
        except Exception:
            pass  # CAN errors don't stop the controller
