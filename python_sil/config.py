"""
AEB System Configuration Parameters
====================================
All calibration parameters separated from code (NFR-POR-002).
Based on:
  - MDPI Applied Sciences 13(1):508 (AEB Simulink modeling)
  - PMC6864679 (AEB-P Longitudinal Collision Avoidance)
  - Euro NCAP AEB C2C Test Protocol v4.3.1
  - GB/T 33577-2017 (TTC thresholds)
  - ISO 11898 (CAN bus)
"""
from dataclasses import dataclass, field
from typing import Dict

# ============================================================
# 1. SIMULATION PARAMETERS
# ============================================================
@dataclass
class SimConfig:
    dt: float = 0.001          # Plant integration step [s] (1 kHz)
    dt_controller: float = 0.01  # Controller cycle [s] (100 Hz = 10 ms) (NFR-PERF-001)
    dt_sensor: float = 0.02    # Radar update period [s] (50 Hz)
    t_max: float = 15.0        # Maximum simulation time [s]


# ============================================================
# 2. VEHICLE PLANT PARAMETERS (1-DOF Longitudinal)
# ============================================================
@dataclass
class VehicleConfig:
    """
    1-DOF point-mass longitudinal model:
        m * dv/dt = F_brake + F_drag + F_roll

    Ref: MATLAB Vehicle Body 1DOF Longitudinal block
         PMC6864679 Table 2
    """
    mass: float = 1500.0          # Vehicle mass [kg]
    frontal_area: float = 2.73    # Frontal area [m^2]
    cd: float = 0.32              # Drag coefficient [-]
    cr: float = 0.015             # Rolling resistance coefficient [-]
    wheelbase: float = 2.75       # Wheelbase [m]
    cg_height: float = 0.55       # CG height [m]
    wheel_radius: float = 0.32    # Wheel radius [m]
    air_density: float = 1.225    # Air density [kg/m^3]
    g: float = 9.81               # Gravity [m/s^2]
    v_min: float = 0.0            # Minimum velocity [m/s] (can't go negative)


# ============================================================
# 3. BRAKE ACTUATOR PARAMETERS
# ============================================================
@dataclass
class BrakeActuatorConfig:
    """
    First-order lag + dead time model:
        G(s) = K * e^(-theta*s) / (tau*s + 1)

    Ref: x-engineer.org ABS modeling
         MDPI EVS 15(10):433
    """
    tau: float = 0.05             # Hydraulic time constant [s]
    dead_time: float = 0.03       # System dead time [s]
    max_decel: float = 10.0       # Maximum achievable deceleration [m/s^2]
    max_brake_cmd: float = 100.0  # Maximum brake command [%]
    # Mapping: brake_cmd [0-100%] -> deceleration [0 - max_decel m/s^2]
    cmd_to_decel_gain: float = 0.1  # 100% -> 10 m/s^2


# ============================================================
# 4. SENSOR MODEL PARAMETERS
# ============================================================
@dataclass
class SensorConfig:
    """
    77 GHz radar sensor model with Gaussian noise.

    Ref: ZLY Tech 77GHz radar datasheet
         PMC9370944 (automotive radar survey)
    """
    # Noise standard deviations
    range_noise_std: float = 0.15      # Distance noise [m]
    range_rate_noise_std: float = 0.05  # Range rate noise [m/s]
    ego_speed_noise_std: float = 0.02   # Ego speed noise [m/s] (wheel speed sensor)

    # Latency
    radar_latency: float = 0.04        # Radar processing latency [s]

    # Plausibility check limits (FR-PER-006)
    range_min: float = 0.5             # Minimum valid range [m]
    range_max: float = 300.0           # Maximum valid range [m]
    speed_min: float = 0.0             # Minimum valid speed [m/s]
    speed_max: float = 50.0            # Maximum valid speed [m/s] (~180 km/h)
    max_range_rate_of_change: float = 50.0  # Max |dd/dt| [m/s] (plausibility)
    max_speed_rate_of_change: float = 15.0  # Max |dv/dt| [m/s^2] (plausibility)

    # Fault detection (FR-PER-007)
    fault_cycles_threshold: int = 3    # Consecutive invalid cycles to set fault


@dataclass
class LidarConfig:
    """
    LiDAR sensor model (e.g., Velodyne VLP-16 or similar).
    Higher accuracy but lower update rate than radar.

    Ref: Velodyne VLP-16 datasheet, PMC9370944
    """
    range_noise_std: float = 0.03      # Distance noise [m] (much better than radar)
    range_max: float = 200.0           # Maximum range [m]
    update_period: float = 0.05        # 20 Hz update rate [s]
    latency: float = 0.05             # Processing latency [s]
    fov_horizontal: float = 360.0      # Horizontal FOV [deg]
    fov_vertical: float = 30.0         # Vertical FOV [deg]
    angular_resolution: float = 0.2    # Angular resolution [deg]


@dataclass
class FusionConfig:
    """
    Sensor fusion parameters (radar + lidar).
    Weighted average based on measurement covariance (Kalman-like).

    Ref: PMC9370944 (Survey on Automotive Radar/Lidar Fusion)
    """
    # Fusion weights (derived from noise covariance: w = 1/sigma^2)
    # Higher weight = more trust in that sensor
    use_lidar: bool = True
    use_radar: bool = True

    # Kalman filter process noise
    process_noise_pos: float = 0.1     # Position process noise [m^2]
    process_noise_vel: float = 0.5     # Velocity process noise [(m/s)^2]

    # Confidence threshold for fused output
    min_confidence: float = 0.5        # Minimum fusion confidence [0-1]

    # If one sensor fails, fallback to the other
    single_sensor_degraded: bool = True


# ============================================================
# 5. TTC CALCULATOR PARAMETERS
# ============================================================
@dataclass
class TTCConfig:
    """
    TTC = d / (v_ego - v_target), valid when v_ego > v_target

    Ref: GB/T 33577-2017
         MDPI Applied Sciences 13(1):508
         PMC6864679 Table 4
    """
    ttc_max: float = 10.0             # Maximum TTC to compute [s]
    v_rel_min: float = 0.5            # Minimum closing speed to compute TTC [m/s]

    # Braking distance: d_brake = v^2 / (2 * a_max) (FR-DEC-002)
    a_max_braking: float = 6.0        # Max deceleration for d_brake calc [m/s^2]


# ============================================================
# 6. FSM PARAMETERS
# ============================================================
@dataclass
class FSMConfig:
    """
    7-state FSM: OFF, STANDBY, WARNING, BRAKE_L1, BRAKE_L2, BRAKE_L3, POST_BRAKE

    TTC Thresholds (FR-DEC-004, Table: Intervention Levels):
        STANDBY:  TTC > 4.0 s
        WARNING:  4.0 >= TTC > 3.0 s    -> 0 m/s^2
        BRAKE_L1: 3.0 >= TTC > 2.2 s    -> -2 m/s^2
        BRAKE_L2: 2.2 >= TTC > 1.8 s    -> -4 m/s^2
        BRAKE_L3: TTC <= 1.8 s          -> -6 m/s^2

    Ref: GB/T 33577-2017, MDPI Art.
    """
    # TTC thresholds [s]
    ttc_warning: float = 4.0
    ttc_brake_l1: float = 3.0
    ttc_brake_l2: float = 2.2
    ttc_brake_l3: float = 1.8

    # Target decelerations [m/s^2] (positive values, applied as negative)
    decel_warning: float = 0.0
    decel_brake_l1: float = 2.0
    decel_brake_l2: float = 4.0
    decel_brake_l3: float = 6.0

    # Speed range [m/s] (FR-DEC-008: 10-60 km/h)
    v_ego_min: float = 2.78            # 10 km/h in m/s
    v_ego_max: float = 16.67           # 60 km/h in m/s

    # Hysteresis / debounce (FR-DEC-010, FR-FSM-004)
    hysteresis_time: float = 0.2       # De-escalation debounce [s]
    ttc_hysteresis: float = 0.15       # TTC hysteresis band [s]

    # Driver override (FR-DEC-007)
    steering_override_deg: float = 5.0  # Steering angle threshold [deg]

    # Post-brake hold (FR-BRK-005)
    post_brake_hold_time: float = 2.0  # Hold brake after stop [s]

    # Warning-to-brake minimum time (FR-ALR-003)
    warning_to_brake_min: float = 0.8  # Minimum 800 ms


# ============================================================
# 7. PID CONTROLLER PARAMETERS
# ============================================================
@dataclass
class PIDConfig:
    """
    PI controller for brake actuation (FR-BRK-002).
    e(t) = a_desired - a_actual
    u(t) = Kp * e(t) + Ki * integral(e(t))

    Ref: PMC6864679 Table 5 (gain-scheduled PI)
    """
    kp: float = 4.0                    # Proportional gain
    ki: float = 0.05                   # Integral gain (1/Ti, Ti~20)
    kd: float = 0.0                    # Derivative gain (PI only)

    # Anti-windup
    integral_max: float = 50.0         # Integrator saturation [%]
    integral_min: float = 0.0

    # Output limits
    output_min: float = 0.0            # Minimum brake command [%]
    output_max: float = 100.0          # Maximum brake command [%]

    # Jerk limiting (FR-BRK-004: |jerk| <= 10 m/s^3)
    max_jerk: float = 10.0             # Maximum jerk [m/s^3]
    max_decel_rate: float = 10.0       # Max deceleration rate of change [m/s^3]


# ============================================================
# 8. CAN BUS PARAMETERS
# ============================================================
@dataclass
class CANConfig:
    """
    CAN 2.0B communication model.

    Ref: ISO 11898-1:2015
         CSS Electronics CAN bus guides
    """
    baud_rate: int = 500_000           # 500 kbps
    max_bus_load: float = 0.30         # 30% target for ASIL B

    # Message definitions: (CAN_ID, DLC, period_ms, name)
    messages: list = field(default_factory=lambda: [
        (0x080, 4, 10,  "AEB_BrakeCmd"),      # Highest priority
        (0x100, 8, 10,  "AEB_EgoVehicle"),
        (0x120, 8, 20,  "AEB_RadarTarget"),
        (0x200, 4, 50,  "AEB_FSMState"),
        (0x300, 2, 100, "AEB_Alert"),          # Event-driven, ~10 Hz avg
    ])

    # Stuff bit multiplier (industry standard approximation)
    stuff_bit_factor: float = 1.20


# ============================================================
# 9. TIRE-ROAD FRICTION (Burckhardt model)
# ============================================================
@dataclass
class TireConfig:
    """
    mu(s) = A * (B * (1 - exp(-C*s)) - D*s)

    Ref: x-engineer.org ABS modeling
    """
    # Dry concrete coefficients
    A: float = 0.9
    B: float = 1.07
    C: float = 0.2773
    D: float = 0.0026
    mu_max: float = 0.9               # Maximum friction coefficient


# ============================================================
# MASTER CONFIG
# ============================================================
@dataclass
class AEBConfig:
    sim: SimConfig = field(default_factory=SimConfig)
    vehicle: VehicleConfig = field(default_factory=VehicleConfig)
    brake_actuator: BrakeActuatorConfig = field(default_factory=BrakeActuatorConfig)
    sensor: SensorConfig = field(default_factory=SensorConfig)
    lidar: LidarConfig = field(default_factory=LidarConfig)
    fusion: FusionConfig = field(default_factory=FusionConfig)
    ttc: TTCConfig = field(default_factory=TTCConfig)
    fsm: FSMConfig = field(default_factory=FSMConfig)
    pid: PIDConfig = field(default_factory=PIDConfig)
    can: CANConfig = field(default_factory=CANConfig)
    tire: TireConfig = field(default_factory=TireConfig)
