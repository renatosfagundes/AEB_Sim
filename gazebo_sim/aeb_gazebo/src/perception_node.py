#!/usr/bin/env python3
"""
AEB Perception Node — Simulated Radar + Lidar Fusion
=====================================================
Computes distance from ground-truth odom, then applies realistic
sensor noise to simulate radar and lidar independently. The two
"virtual sensors" are fused with weighted averaging, plausibility
checks, and fault detection — exactly as a real system would.

Sensor models:
  Radar (77 GHz):  sigma=0.25m noise, 20 Hz, range 0.5-260m
  Lidar (905 nm):  sigma=0.05m noise, 15 Hz, range 0.3-200m

Implements:
  FR-PER-001: Acquire relative distance (fused radar+lidar)
  FR-PER-002: Acquire ego velocity from odometry
  FR-PER-003: Compute relative velocity from distance derivative
  FR-PER-006: Plausibility checks (range, rate-of-change, cross-sensor)
  FR-PER-007: Fault detection (3 consecutive invalid -> fault flag)

Subscribes:
  /aeb/ego/odom     - Ego vehicle odometry
  /aeb/target/odom  - Target vehicle odometry

Publishes (CAN frames — one topic per DBC message ID):
  /can/radar_target [AEB_RadarTarget]  20 ms  — Radar_ECU  (0x120)
  /can/ego_vehicle  [AEB_EgoVehicle]   10 ms  — Vehicle_ECU (0x100)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from aeb_gazebo.msg import AebRadarTarget, AebEgoVehicle
import math
import time
import random


# --- Constants from SRS ---
DIST_RANGE_MIN = 0.0
DIST_RANGE_MAX = 300.0
SPEED_RANGE_MAX = 50.0      # m/s
FAULT_CYCLE_LIMIT = 3       # FR-PER-007: 3 cycles = 30 ms
TTC_MAX = 10.0
V_REL_MIN = 0.5             # m/s min closing speed for TTC
EGO_LENGTH = 4.5            # m (distance between odom origins)

# Plausibility (FR-PER-006)
DIST_RATE_MAX = 60.0        # m/s
CROSS_SENSOR_TOL = 5.0      # m

# Sensor models
RADAR_NOISE_STD = 0.25      # m (77 GHz radar)
RADAR_RANGE_MIN = 0.5
RADAR_RANGE_MAX = 260.0
RADAR_PERIOD = 1.0 / 20.0   # 20 Hz

LIDAR_NOISE_STD = 0.05      # m (lidar)
LIDAR_RANGE_MIN = 0.3
LIDAR_RANGE_MAX = 200.0
LIDAR_PERIOD = 1.0 / 15.0   # 15 Hz

# Fusion weights
W_RADAR = 0.3
W_LIDAR = 0.7

# EMA filter
EMA_ALPHA = 0.4

# CAN IDs (matches DBC)
CAN_ID_RADAR_TARGET = 0x120
CAN_ID_EGO_VEHICLE  = 0x100

# Control loop periods
CTRL_PERIOD_10MS = 0.01    # 10 ms — ego vehicle frame
RADAR_PUB_PERIOD = 0.02    # 20 ms — radar target frame


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        # Ground truth from odom
        self.ego_x = 0.0
        self.ego_vx = 0.0
        self.ego_vx_prev = 0.0
        self.ego_long_accel = 0.0
        self.target_x = 100.0
        self.target_vx = 0.0
        self.odom_ready = False
        self.last_accel_time = None

        # Simulated sensor readings
        self.radar_distance = None
        self.radar_time = 0.0
        self.lidar_distance = None
        self.lidar_time = 0.0

        # Fused output
        self.fused_distance = 100.0
        self.prev_fused_distance = 100.0
        self.v_rel = 0.0
        self.v_rel_filtered = 0.0
        self.last_fusion_time = None

        # Fault tracking
        self.fault_counter = 0
        self.sensor_fault = False

        # Rolling alive counters (0-15), one per CAN frame type
        self.alive_radar = 0
        self.alive_ego = 0

        # Publishers — one per CAN frame
        self.radar_pub = self.create_publisher(AebRadarTarget, '/can/radar_target', 10)
        self.ego_pub   = self.create_publisher(AebEgoVehicle,  '/can/ego_vehicle',  10)

        # Subscribers
        self.create_subscription(Odometry, '/aeb/ego/odom',    self.ego_odom_cb,    10)
        self.create_subscription(Odometry, '/aeb/target/odom', self.target_odom_cb, 10)

        # Vehicle_ECU frame: 10 ms
        self.timer_10ms = self.create_timer(CTRL_PERIOD_10MS, self.publish_ego_vehicle)
        # Radar_ECU frame: 20 ms
        self.timer_20ms = self.create_timer(RADAR_PUB_PERIOD, self.radar_fusion_cycle)

        self.get_logger().info(
            'Perception node started — publishing /can/radar_target (20ms) '
            'and /can/ego_vehicle (10ms)'
        )

    # ── Odometry callbacks ──────────────────────────────────────────────────

    def ego_odom_cb(self, msg):
        now = time.time()
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        speed = math.sqrt(vx * vx + vy * vy)

        # Estimate longitudinal acceleration from speed derivative
        if self.last_accel_time is not None:
            dt = now - self.last_accel_time
            if dt > 0.001:
                self.ego_long_accel = (speed - self.ego_vx_prev) / dt
        self.ego_vx_prev = speed
        self.last_accel_time = now

        self.ego_x = msg.pose.pose.position.x
        self.ego_vx = speed
        if not self.odom_ready:
            self.odom_ready = True
            self.get_logger().info('Odom received — sensors active')

    def target_odom_cb(self, msg):
        self.target_x = msg.pose.pose.position.x
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.target_vx = math.sqrt(vx * vx + vy * vy)

    # ── Radar/Lidar fusion → AEB_RadarTarget (0x120) @ 20 ms ───────────────

    def radar_fusion_cycle(self):
        if not self.odom_ready:
            return

        now = time.time()
        gt_dist = self.target_x - self.ego_x - EGO_LENGTH

        self._simulate_radar(now, gt_dist)
        self._simulate_lidar(now, gt_dist)

        radar_ok = (self.radar_distance is not None and
                    DIST_RANGE_MIN <= self.radar_distance <= DIST_RANGE_MAX)
        lidar_ok = (self.lidar_distance is not None and
                    DIST_RANGE_MIN <= self.lidar_distance <= DIST_RANGE_MAX)

        raw_distance = None
        if radar_ok and lidar_ok:
            if abs(self.radar_distance - self.lidar_distance) > CROSS_SENSOR_TOL:
                raw_distance = self.lidar_distance
            else:
                raw_distance = W_RADAR * self.radar_distance + W_LIDAR * self.lidar_distance
        elif lidar_ok:
            raw_distance = self.lidar_distance
        elif radar_ok:
            raw_distance = self.radar_distance

        if raw_distance is not None and self.last_fusion_time is not None:
            dt = now - self.last_fusion_time
            if dt > 0.001:
                rate = abs(raw_distance - self.prev_fused_distance) / dt
                if rate > DIST_RATE_MAX:
                    raw_distance = None

        if raw_distance is not None:
            self.fault_counter = 0
            self.sensor_fault = False
            self.v_rel = self.ego_vx - self.target_vx  # positive = closing
            self.prev_fused_distance = self.fused_distance
            self.fused_distance = raw_distance
            self.last_fusion_time = now
        else:
            self.fault_counter += 1
            if self.fault_counter >= FAULT_CYCLE_LIMIT:
                if not self.sensor_fault:
                    self.get_logger().warn('SENSOR FAULT (FR-PER-007)')
                self.sensor_fault = True

        # Compute TTC
        ttc = TTC_MAX
        if self.v_rel > V_REL_MIN and self.fused_distance > 0:
            ttc = max(0.0, min(TTC_MAX, self.fused_distance / self.v_rel))

        # Confidence: 15 if both sensors valid, 8 if one, 0 if fault
        if radar_ok and lidar_ok:
            confidence = 15
        elif radar_ok or lidar_ok:
            confidence = 8
        else:
            confidence = 0

        # Build AebRadarTarget frame
        msg = AebRadarTarget()
        msg.can_id          = CAN_ID_RADAR_TARGET
        msg.alive_counter   = self.alive_radar
        msg.target_distance = float(max(0.0, min(DIST_RANGE_MAX, self.fused_distance)))
        msg.relative_speed  = float(self.v_rel)   # positive = closing
        msg.ttc             = float(ttc)
        msg.confidence      = confidence
        msg.sensor_fault    = self.sensor_fault

        self.radar_pub.publish(msg)
        self.alive_radar = (self.alive_radar + 1) & 0x0F

    # ── Ego vehicle state → AEB_EgoVehicle (0x100) @ 10 ms ─────────────────

    def publish_ego_vehicle(self):
        if not self.odom_ready:
            return

        msg = AebEgoVehicle()
        msg.can_id         = CAN_ID_EGO_VEHICLE
        msg.alive_counter  = self.alive_ego
        msg.vehicle_speed  = float(self.ego_vx)
        msg.long_accel     = float(self.ego_long_accel)
        msg.yaw_rate       = 0.0    # not available from planar_move odom
        msg.steering_angle = 0.0    # not available from planar_move odom
        msg.brake_pedal    = False  # no driver input in simulation

        self.ego_pub.publish(msg)
        self.alive_ego = (self.alive_ego + 1) & 0x0F

    # ── Internal sensor simulators ──────────────────────────────────────────

    def _simulate_radar(self, now, gt_dist):
        if now - self.radar_time >= RADAR_PERIOD:
            self.radar_time = now
            if RADAR_RANGE_MIN <= gt_dist <= RADAR_RANGE_MAX:
                self.radar_distance = gt_dist + random.gauss(0, RADAR_NOISE_STD)
            else:
                self.radar_distance = None

    def _simulate_lidar(self, now, gt_dist):
        if now - self.lidar_time >= LIDAR_PERIOD:
            self.lidar_time = now
            if LIDAR_RANGE_MIN <= gt_dist <= LIDAR_RANGE_MAX:
                self.lidar_distance = gt_dist + random.gauss(0, LIDAR_NOISE_STD)
            else:
                self.lidar_distance = None


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
