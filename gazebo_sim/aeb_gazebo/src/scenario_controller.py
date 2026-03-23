#!/usr/bin/env python3
"""
Scenario Controller Node
========================
Controls both ego and target vehicle velocities to execute
Euro NCAP CCR test scenarios (CCRs, CCRm, CCRb).

Subscribes to:
  /aeb/ego/odom           - Ego vehicle odometry
  /aeb/target/odom        - Target vehicle odometry
  /can/brake_cmd          [AEB_BrakeCmd]    - Brake command from controller
  /can/radar_target       [AEB_RadarTarget] - Distance / TTC from perception

Publishes:
  /aeb/ego/cmd_vel        - Ego vehicle velocity command
  /aeb/target/cmd_vel     - Target vehicle velocity command
  /aeb/scenario_status    - Scenario state (running/passed/failed)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from aeb_gazebo.msg import AebBrakeCmd, AebRadarTarget, AebEgoVehicle
import math
import time


class ScenarioController(Node):
    def __init__(self):
        super().__init__('scenario_controller')

        # Declare parameters
        self.declare_parameter('scenario', 'ccrs_40')
        self.declare_parameter('ego_speed_kmh', 40.0)
        self.declare_parameter('target_speed_kmh', 0.0)
        self.declare_parameter('initial_gap_m', 100.0)
        self.declare_parameter('target_decel', 0.0)
        self.declare_parameter('target_brake_time', 0.0)
        self.declare_parameter('skip_teleport', False)
        self.declare_parameter('aeb_enabled', True)

        # Get parameters
        self.scenario_name = self.get_parameter('scenario').value
        self.ego_speed_ms = self.get_parameter('ego_speed_kmh').value / 3.6
        self.target_speed_ms = self.get_parameter('target_speed_kmh').value / 3.6
        self.initial_gap = self.get_parameter('initial_gap_m').value
        self.target_decel = self.get_parameter('target_decel').value
        self.target_brake_time = self.get_parameter('target_brake_time').value
        self.skip_teleport = self.get_parameter('skip_teleport').value
        self.aeb_enabled = self.get_parameter('aeb_enabled').value

        # State
        self.ego_x = 0.0
        self.ego_vx = 0.0
        self.ego_vx_cmd = 0.0
        self.target_x = self.initial_gap
        self.target_vx = self.target_speed_ms
        self.target_vx_cmd = self.target_speed_ms
        self.brake_cmd_pct = 0.0       # 0-100 % from AEB_BrakeCmd
        self.start_time = None
        self.scenario_running = False
        self.scenario_ended = False
        self.collision_time = None
        self.collision_detected = False
        self.max_decel_achieved = 0.0
        self.ego_has_braked = False
        self.perceived_distance = 100.0
        self.perceived_ego_speed_ms = 0.0

        # Publishers
        self.ego_cmd_pub    = self.create_publisher(Twist,  '/aeb/ego/cmd_vel',     10)
        self.target_cmd_pub = self.create_publisher(Twist,  '/aeb/target/cmd_vel',  10)
        self.status_pub     = self.create_publisher(String, '/aeb/scenario_status', 10)

        # Subscribers
        self.create_subscription(Odometry, '/aeb/ego/odom',    self.ego_odom_cb,    10)
        self.create_subscription(Odometry, '/aeb/target/odom', self.target_odom_cb, 10)
        self.create_subscription(Float64,  '/aeb/restart',     self.restart_cb,     10)

        # CAN frame subscribers
        self.create_subscription(AebBrakeCmd,    '/can/brake_cmd',     self.brake_cmd_cb,    10)
        self.create_subscription(AebRadarTarget, '/can/radar_target',  self.radar_target_cb, 10)
        self.create_subscription(AebEgoVehicle,  '/can/ego_vehicle',   self.ego_vehicle_cb,  10)

        # Main loop at 100 Hz
        self.timer = self.create_timer(0.01, self.control_loop)

        self.gazebo_ready = False
        self.odom_received = False
        self.target_teleported = False

        # Service client to teleport target vehicle
        self.set_state_client = self.create_client(
            SetEntityState, '/aeb/set_entity_state'
        )

        self.get_logger().info(f'=== Scenario: {self.scenario_name} ===')
        self.get_logger().info(
            f'Ego: {self.ego_speed_ms*3.6:.0f} km/h, '
            f'Target: {self.target_speed_ms*3.6:.0f} km/h, '
            f'Gap: {self.initial_gap:.0f} m'
        )
        self.get_logger().info('Waiting for Gazebo to be ready...')

    # ── CAN frame callbacks ─────────────────────────────────────────────────

    def brake_cmd_cb(self, msg: AebBrakeCmd):
        """Decode brake command from AEB_BrakeCmd frame (0x080)."""
        self.brake_cmd_pct = (msg.brake_pressure / 0.1) * (10.0 / 100.0) * 100.0
        # Simpler: brake_pressure [bar] → decel [%]: pressure * 10 = %
        # brake_pressure = brake_pct * 0.1  →  brake_pct = brake_pressure / 0.1
        self.brake_cmd_pct = float(msg.brake_pressure) / 0.1

    def radar_target_cb(self, msg: AebRadarTarget):
        """Decode distance from AEB_RadarTarget frame (0x120)."""
        self.perceived_distance = msg.target_distance

    def ego_vehicle_cb(self, msg: AebEgoVehicle):
        """Decode ego speed from AEB_EgoVehicle frame (0x100)."""
        self.perceived_ego_speed_ms = msg.vehicle_speed

    # ── Standard odometry callbacks ─────────────────────────────────────────

    def restart_cb(self, msg):
        """Reset scenario to initial conditions."""
        self.get_logger().info('=== RESTARTING SCENARIO ===')
        self.ego_vx_cmd = 0.0
        self.target_vx_cmd = self.target_speed_ms
        self.brake_cmd_pct = 0.0
        self.scenario_running = False
        self.scenario_ended = False
        self.collision_time = None
        self.collision_detected = False
        self.max_decel_achieved = 0.0
        self.ego_has_braked = False
        self.perceived_distance = 100.0
        self.perceived_ego_speed_ms = 0.0
        self.publish_vel(self.ego_cmd_pub, 0.0)
        self.publish_vel(self.target_cmd_pub, 0.0)
        self.gazebo_ready = True
        self.odom_received = True
        self.create_timer(2.0, self._start_scenario_once)
        self.publish_status('')

    def _teleport_target(self):
        self.get_logger().info(
            f'Teleporting target_vehicle to x={self.initial_gap:.1f} m ...'
        )
        if not self.set_state_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                'set_entity_state service not available — using world default position'
            )
            self.target_teleported = True
            self.get_logger().info('Starting scenario in 2 seconds...')
            self.create_timer(2.0, self._start_scenario_once)
            return

        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = 'target_vehicle'
        req.state.pose = Pose(
            position=Point(x=self.initial_gap, y=-1.85, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )
        future = self.set_state_client.call_async(req)
        future.add_done_callback(self._teleport_done)

    def _teleport_done(self, future):
        try:
            result = future.result()
            if result.success:
                self.get_logger().info(
                    f'Target teleported to gap={self.initial_gap:.1f} m'
                )
            else:
                self.get_logger().warn('Teleport failed — using default position')
        except Exception as e:
            self.get_logger().warn(f'Teleport service error: {e}')
        self.target_teleported = True
        self.get_logger().info('Starting scenario in 2 seconds...')
        self.create_timer(2.0, self._start_scenario_once)

    def _start_scenario_once(self):
        if not self.scenario_running and not self.scenario_ended:
            self.scenario_running = True
            self.start_time = time.time()
            self.get_logger().info('>>> Scenario STARTED <<<')

    def ego_odom_cb(self, msg):
        self.ego_x = msg.pose.pose.position.x
        self.ego_vx = msg.twist.twist.linear.x
        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info('Odom received — Gazebo is ready')

    def target_odom_cb(self, msg):
        self.target_x = msg.pose.pose.position.x
        self.target_vx = msg.twist.twist.linear.x

    # ── Main control loop ───────────────────────────────────────────────────

    def control_loop(self):
        if self.scenario_ended:
            # After a collision, keep commanding ego speed for 2 s so Gazebo
            # physics can resolve the impact (push) before stopping both vehicles.
            if self.collision_detected and self.collision_time is not None:
                if time.time() - self.collision_time < 2.0:
                    self.publish_vel(self.ego_cmd_pub, self.ego_vx_cmd)
                    return
            self.publish_vel(self.ego_cmd_pub, 0.0)
            self.publish_vel(self.target_cmd_pub, 0.0)
            return

        if not self.scenario_running:
            if self.odom_received and not self.gazebo_ready:
                self.gazebo_ready = True
                if self.skip_teleport:
                    # Target already spawns at the correct gap in the world file.
                    # Calling set_entity_state to the same x gives a physics impulse
                    # that makes the target drift forward for ~3 s, delaying WARNING.
                    self.get_logger().info(
                        f'Skipping teleport — target spawns at gap={self.initial_gap:.0f} m'
                    )
                    self.target_teleported = True
                    self.create_timer(2.0, self._start_scenario_once)
                elif not self.target_teleported:
                    self._teleport_target()
                else:
                    self.get_logger().info('Starting scenario in 2 seconds...')
                    self.create_timer(2.0, self._start_scenario_once)
            return

        elapsed = time.time() - self.start_time
        dt = 0.01

        # Target vehicle motion profile
        if self.target_decel != 0.0 and elapsed >= self.target_brake_time:
            self.target_vx_cmd = max(0.0, self.target_vx_cmd + self.target_decel * dt)
        else:
            self.target_vx_cmd = self.target_speed_ms

        self.publish_vel(self.target_cmd_pub, self.target_vx_cmd)

        # Ego: apply AEB braking (0-100% maps to 0-10 m/s²)
        brake_decel = (self.brake_cmd_pct / 100.0) * 10.0

        if self.aeb_enabled and self.brake_cmd_pct > 1.0:
            self.ego_vx_cmd = max(0.0, self.ego_vx_cmd - brake_decel * dt)
            self.ego_has_braked = True
        elif not self.ego_has_braked:
            self.ego_vx_cmd = self.ego_speed_ms

        self.publish_vel(self.ego_cmd_pub, self.ego_vx_cmd)

        if brake_decel > self.max_decel_achieved:
            self.max_decel_achieved = brake_decel

        # End-condition checks (use CAN-decoded perception data)
        distance = self.perceived_distance
        ego_speed_kmh = self.perceived_ego_speed_ms * 3.6

        if distance <= 1.0 and ego_speed_kmh > 5.0:
            self.collision_detected = True
            self.scenario_ended = True
            if self.collision_time is None:
                self.collision_time = time.time()
            self.get_logger().warn(
                f'!!! COLLISION at v_impact = {ego_speed_kmh:.1f} km/h !!!'
            )
            self.publish_status(f'COLLISION: v_impact={ego_speed_kmh:.1f} km/h')

        elif distance <= 1.0 and ego_speed_kmh <= 5.0 and elapsed > 2.0:
            self.scenario_ended = True
            self.get_logger().info(
                f'*** NEAR STOP: gap={distance:.1f}m v={ego_speed_kmh:.1f}km/h ***'
            )
            self.publish_status(f'STOPPED: gap={distance:.1f}m v={ego_speed_kmh:.1f}km/h')

        elif self.ego_vx_cmd < 0.15 and elapsed > 2.0:
            self.scenario_ended = True
            self.get_logger().info(
                f'*** STOPPED: distance remaining = {distance:.1f} m ***'
            )
            self.publish_status(f'STOPPED: gap={distance:.1f}m')

        elif elapsed > 60.0:
            self.scenario_ended = True
            self.get_logger().info('Scenario timed out (60s)')
            self.publish_status('TIMEOUT')

        if int(elapsed * 100) % 100 == 0:
            self.get_logger().info(
                f't={elapsed:.1f}s  d={distance:.1f}m  '
                f'v_ego={ego_speed_kmh:.1f}km/h  '
                f'brake={self.brake_cmd_pct:.0f}%'
            )

    def publish_vel(self, publisher, speed_ms):
        msg = Twist()
        msg.linear.x = speed_ms
        publisher.publish(msg)

    def publish_status(self, status_text):
        msg = String()
        msg.data = status_text
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScenarioController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
