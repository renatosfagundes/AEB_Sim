"""
AEB SIL Simulation Runner
===========================
Runs all Euro NCAP scenarios and generates plots.

Usage:
    python main_sim.py              # Run all scenarios (save PNGs only)
    python main_sim.py --scenario CCRs_40kmh  # Run specific scenario
    python main_sim.py --plot       # Run and show interactive plots per scenario
"""
import sys
import os
import numpy as np

# Backend selection: interactive when --plot, headless otherwise
_SHOW_PLOTS = "--plot" in sys.argv
if not _SHOW_PLOTS:
    import matplotlib
    matplotlib.use('Agg')

import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

from config import AEBConfig
from aeb_system import AEBSimulation
from scenarios import (
    get_all_scenarios, get_ccrs_scenario, get_ccrm_scenario,
    get_ccrb_scenario, ScenarioConfig
)


# ===================================================================
# Helper: extract all signals from log into a dict of numpy arrays
# ===================================================================
def _extract_signals(log):
    """Convert a list of SimulationStep into a dict of numpy arrays."""
    fields = [
        't', 'ego_x', 'ego_v', 'ego_a', 'target_x', 'target_v',
        'distance', 'v_rel', 'v_ego_sensed', 'sensor_confidence', 'sensor_fault',
        'ground_truth_distance', 'radar_distance', 'lidar_distance',
        'radar_valid', 'lidar_valid', 'kalman_P_trace',
        'ttc', 'd_brake', 'is_closing',
        'fsm_state', 'target_decel',
        'brake_cmd', 'brake_actual',
        'pid_error', 'pid_p_term', 'pid_i_term', 'jerk',
        'alert_visual', 'alert_audible',
        'can_bus_load',
    ]
    return {f: np.array([getattr(s, f) for s in log]) for f in fields}


# Common constants
_STATE_NAMES = ['OFF', 'STANDBY', 'WARNING', 'BRAKE_L1', 'BRAKE_L2',
                'BRAKE_L3', 'POST_BRAKE']
_STATE_COLORS = ['gray', 'green', 'orange', '#ff6600', '#ff3300',
                 'red', 'blue']


# ===================================================================
# Window 1 (original): Overview — 6 subplots
# ===================================================================
def plot_scenario_results(log, scenario: ScenarioConfig, result: dict,
                          save_path: str = None):
    """
    Generate a 6-subplot figure for a single scenario.

    Subplots:
      1. Position (ego vs target) and gap distance
      2. Velocity (ego vs target)
      3. TTC and braking distance
      4. FSM state and brake command
      5. Deceleration (target vs actual)
      6. Sensor confidence and CAN bus load
    """
    s = _extract_signals(log)
    t = s['t']

    fig = plt.figure(figsize=(16, 20))
    fig.canvas.manager.set_window_title(f"Overview — {scenario.name}")
    fig.suptitle(f"AEB Scenario: {scenario.name}\n{scenario.description}\n"
                 f"Result: {'PASS' if result['passed'] else 'FAIL'} - {result['reason']}",
                 fontsize=14, fontweight='bold')
    gs = GridSpec(6, 1, hspace=0.35)

    # 1. Position and gap
    ax1 = fig.add_subplot(gs[0])
    ax1.plot(t, s['ego_x'], 'b-', label='Ego position', linewidth=1.5)
    ax1.plot(t, s['target_x'], 'r--', label='Target position', linewidth=1.5)
    ax1b = ax1.twinx()
    ax1b.plot(t, s['distance'], 'g-', alpha=0.7, label='Gap distance')
    ax1b.fill_between(t, 0, s['distance'], alpha=0.1, color='green')
    ax1.set_ylabel('Position [m]')
    ax1b.set_ylabel('Gap [m]', color='green')
    ax1.legend(loc='upper left')
    ax1b.legend(loc='upper right')
    ax1.set_title('Vehicle Positions and Gap Distance')
    ax1.grid(True, alpha=0.3)

    # 2. Velocity
    ax2 = fig.add_subplot(gs[1])
    ax2.plot(t, s['ego_v'] * 3.6, 'b-', label='Ego velocity', linewidth=1.5)
    ax2.plot(t, s['target_v'] * 3.6, 'r--', label='Target velocity', linewidth=1.5)
    ax2.set_ylabel('Velocity [km/h]')
    ax2.legend()
    ax2.set_title('Vehicle Velocities')
    ax2.grid(True, alpha=0.3)

    # 3. TTC and braking distance
    ax3 = fig.add_subplot(gs[2])
    ttc_plot = np.clip(s['ttc'], 0, 10)
    ax3.plot(t, ttc_plot, 'b-', label='TTC', linewidth=1.5)
    ax3.axhline(y=4.0, color='orange', linestyle='--', alpha=0.7, label='WARNING (4.0s)')
    ax3.axhline(y=3.0, color='#ff6600', linestyle='--', alpha=0.7, label='BRAKE_L1 (3.0s)')
    ax3.axhline(y=2.2, color='#ff3300', linestyle='--', alpha=0.7, label='BRAKE_L2 (2.2s)')
    ax3.axhline(y=1.8, color='red', linestyle='--', alpha=0.7, label='BRAKE_L3 (1.8s)')
    ax3b = ax3.twinx()
    ax3b.plot(t, s['d_brake'], 'g--', alpha=0.5, label='d_brake')
    ax3b.plot(t, s['distance'], 'g-', alpha=0.5, label='Gap')
    ax3.set_ylabel('TTC [s]')
    ax3b.set_ylabel('Distance [m]', color='green')
    ax3.legend(loc='upper left', fontsize=8)
    ax3b.legend(loc='upper right', fontsize=8)
    ax3.set_title('Time-to-Collision and Safety Distance')
    ax3.set_ylim(0, 8)
    ax3.grid(True, alpha=0.3)

    # 4. FSM state and brake command
    ax4 = fig.add_subplot(gs[3])
    for i, name in enumerate(_STATE_NAMES):
        mask = s['fsm_state'] == i
        if np.any(mask):
            ax4.fill_between(t, 0, 1, where=mask,
                             alpha=0.3, color=_STATE_COLORS[i], label=name)
    ax4b = ax4.twinx()
    ax4b.plot(t, s['brake_cmd'], 'k-', linewidth=1.5, label='Brake cmd [%]')
    ax4.set_ylabel('FSM State')
    ax4.set_yticks([])
    ax4b.set_ylabel('Brake Command [%]')
    ax4b.set_ylim(0, 110)
    ax4.legend(loc='upper left', fontsize=7, ncol=4)
    ax4b.legend(loc='upper right')
    ax4.set_title('FSM State and Brake Command')
    ax4.grid(True, alpha=0.3)

    # 5. Deceleration
    ax5 = fig.add_subplot(gs[4])
    ax5.plot(t, s['target_decel'], 'r--', label='Target decel', linewidth=1.5)
    ax5.plot(t, s['brake_actual'], 'b-', label='Actual decel', linewidth=1.5)
    ax5.set_ylabel('Deceleration [m/s²]')
    ax5.legend()
    ax5.set_title('Commanded vs Actual Deceleration')
    ax5.grid(True, alpha=0.3)

    # 6. Sensor confidence and CAN bus load
    ax6 = fig.add_subplot(gs[5])
    ax6.plot(t, s['sensor_confidence'], 'b-', label='Sensor confidence', linewidth=1.5)
    ax6b = ax6.twinx()
    ax6b.plot(t, s['can_bus_load'] * 100, 'r-', alpha=0.7, label='CAN bus load [%]')
    ax6b.axhline(y=30, color='r', linestyle='--', alpha=0.5, label='30% limit')
    ax6.set_ylabel('Confidence [0-1]')
    ax6b.set_ylabel('Bus Load [%]', color='red')
    ax6.set_xlabel('Time [s]')
    ax6.legend(loc='upper left')
    ax6b.legend(loc='upper right')
    ax6.set_title('Sensor Fusion Confidence and CAN Bus Load')
    ax6.grid(True, alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.95])

    if save_path:
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"  Plot saved: {save_path}")

    if not _SHOW_PLOTS:
        plt.close()


# ===================================================================
# Window 2: Sensor Fusion Analysis
# ===================================================================
def plot_sensor_fusion(log, scenario: ScenarioConfig, result: dict,
                       save_path: str = None):
    """
    Sensor fusion diagnostics — 3 subplots:
      1. Ground truth vs Radar vs LiDAR vs Kalman fused distance
      2. Measurement error (fused - ground truth)
      3. Kalman filter covariance trace (uncertainty)
    """
    s = _extract_signals(log)
    t = s['t']

    fig, axes = plt.subplots(3, 1, figsize=(14, 12), sharex=True)
    fig.canvas.manager.set_window_title(f"Sensor Fusion — {scenario.name}")
    fig.suptitle(f"Sensor Fusion Analysis — {scenario.name}\n"
                 f"{'PASS' if result['passed'] else 'FAIL'} — {result['reason']}",
                 fontsize=13, fontweight='bold')

    # --- 1. Distance comparison ---
    ax = axes[0]
    ax.plot(t, s['ground_truth_distance'], 'k-', linewidth=2.0,
            label='Ground truth', zorder=5)
    ax.plot(t, s['distance'], 'b-', linewidth=1.5,
            label='Kalman fused', alpha=0.9)
    ax.plot(t, s['radar_distance'], 'r.', markersize=2, alpha=0.4,
            label='Radar raw')
    ax.plot(t, s['lidar_distance'], 'g.', markersize=2, alpha=0.4,
            label='LiDAR raw')
    ax.set_ylabel('Distance [m]')
    ax.set_title('Sensor Readings vs Kalman Fused Output')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.3)

    # --- 2. Fusion error ---
    ax = axes[1]
    error = s['distance'] - s['ground_truth_distance']
    ax.plot(t, error, 'b-', linewidth=1.0, label='Fused error (fused − truth)')
    ax.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    ax.fill_between(t, error, 0, alpha=0.15, color='blue')
    # Compute RMS in a rolling window
    win = min(50, len(error))
    if win > 1:
        cumsum = np.cumsum(np.insert(error ** 2, 0, 0))
        rms = np.sqrt((cumsum[win:] - cumsum[:-win]) / win)
        t_rms = t[win - 1:]
        ax.plot(t_rms, rms, 'r-', linewidth=1.5, alpha=0.8,
                label=f'Rolling RMS (n={win})')
    ax.set_ylabel('Error [m]')
    ax.set_title('Fusion Error (Fused Distance − Ground Truth)')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.3)

    # --- 3. Kalman covariance trace ---
    ax = axes[2]
    ax.plot(t, s['kalman_P_trace'], 'purple', linewidth=1.5)
    ax.set_ylabel('trace(P)')
    ax.set_xlabel('Time [s]')
    ax.set_title('Kalman Filter Uncertainty — trace(P) = P₀₀ + P₁₁')
    ax.set_yscale('log')
    ax.grid(True, alpha=0.3, which='both')

    plt.tight_layout(rect=[0, 0, 1, 0.93])

    if save_path:
        base, ext = os.path.splitext(save_path)
        path = base + '_fusion' + ext
        plt.savefig(path, dpi=150, bbox_inches='tight')
        print(f"  Plot saved: {path}")

    if not _SHOW_PLOTS:
        plt.close()


# ===================================================================
# Window 3: PID Controller Analysis
# ===================================================================
def plot_pid_analysis(log, scenario: ScenarioConfig, result: dict,
                      save_path: str = None):
    """
    PID controller diagnostics — 4 subplots:
      1. PID error signal (target_decel - actual_decel)
      2. P-term and I-term contributions
      3. Brake command vs actual deceleration (shows actuator lag)
      4. Jerk (d(actual_decel)/dt) vs limit
    """
    s = _extract_signals(log)
    t = s['t']

    fig, axes = plt.subplots(4, 1, figsize=(14, 14), sharex=True)
    fig.canvas.manager.set_window_title(f"PID Controller — {scenario.name}")
    fig.suptitle(f"PID Controller Analysis — {scenario.name}\n"
                 f"{'PASS' if result['passed'] else 'FAIL'} — {result['reason']}",
                 fontsize=13, fontweight='bold')

    # --- 1. PID error ---
    ax = axes[0]
    ax.plot(t, s['pid_error'], 'b-', linewidth=1.2, label='PID error (target − actual)')
    ax.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    ax.fill_between(t, s['pid_error'], 0, alpha=0.15, color='blue')
    ax.set_ylabel('Error [m/s²]')
    ax.set_title('PID Error Signal')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)

    # --- 2. P-term and I-term ---
    ax = axes[1]
    ax.plot(t, s['pid_p_term'], 'r-', linewidth=1.2, label='P-term (Kp·e)')
    ax.plot(t, s['pid_i_term'], 'g-', linewidth=1.2, label='I-term (integral)')
    ax.plot(t, s['brake_cmd'], 'k--', linewidth=1.0, alpha=0.6,
            label='Brake cmd [%] (output)')
    ax.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    ax.set_ylabel('Value [%]')
    ax.set_title('PID Term Contributions')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)

    # --- 3. Command vs actual (actuator response) ---
    ax = axes[2]
    # Convert brake_cmd to equivalent deceleration for comparison
    brake_cmd_decel = s['brake_cmd'] * 0.1  # gain = 0.1 m/s² per %
    ax.plot(t, s['target_decel'], 'r--', linewidth=1.5,
            label='FSM target decel')
    ax.plot(t, brake_cmd_decel, 'b--', linewidth=1.0, alpha=0.7,
            label='Cmd equivalent decel')
    ax.plot(t, s['brake_actual'], 'b-', linewidth=1.5,
            label='Actual decel (actuator out)')
    ax.set_ylabel('Deceleration [m/s²]')
    ax.set_title('Actuator Response — Command vs Actual (dead time + lag)')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)

    # --- 4. Jerk ---
    ax = axes[3]
    ax.plot(t, s['jerk'], 'b-', linewidth=1.0, label='Jerk (da/dt)')
    ax.axhline(y=10.0, color='r', linestyle='--', alpha=0.7, label='Limit +10 m/s³')
    ax.axhline(y=-10.0, color='r', linestyle='--', alpha=0.7, label='Limit −10 m/s³')
    ax.set_ylabel('Jerk [m/s³]')
    ax.set_xlabel('Time [s]')
    ax.set_title('Jerk (Rate of Change of Deceleration) — FR-BRK-004')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.93])

    if save_path:
        base, ext = os.path.splitext(save_path)
        path = base + '_pid' + ext
        plt.savefig(path, dpi=150, bbox_inches='tight')
        print(f"  Plot saved: {path}")

    if not _SHOW_PLOTS:
        plt.close()


# ===================================================================
# Window 4: Phase Plane (Distance vs Relative Velocity)
# ===================================================================
def plot_phase_plane(log, scenario: ScenarioConfig, result: dict,
                     save_path: str = None):
    """
    Phase-plane plot: Distance vs Relative Velocity.
    TTC iso-lines are straight lines through the origin (d = TTC * v_rel).
    Trajectory is color-coded by FSM state.
    """
    s = _extract_signals(log)

    fig, ax = plt.subplots(figsize=(12, 9))
    fig.canvas.manager.set_window_title(f"Phase Plane — {scenario.name}")
    fig.suptitle(f"Phase Plane — {scenario.name}\n"
                 f"{'PASS' if result['passed'] else 'FAIL'} — {result['reason']}",
                 fontsize=13, fontweight='bold')

    v_rel = s['v_rel']
    dist = s['distance']
    fsm = s['fsm_state']

    # Draw TTC iso-lines
    v_range = np.linspace(0.01, max(np.nanmax(v_rel) * 1.2, 1.0), 200)
    ttc_thresholds = [(4.0, 'WARNING', 'orange'),
                      (3.0, 'BRAKE_L1', '#ff6600'),
                      (2.2, 'BRAKE_L2', '#ff3300'),
                      (1.8, 'BRAKE_L3', 'red')]
    for ttc_val, label, color in ttc_thresholds:
        d_line = ttc_val * v_range
        ax.plot(v_range, d_line, '--', color=color, alpha=0.6, linewidth=1.2,
                label=f'TTC = {ttc_val}s ({label})')

    # Plot trajectory segments colored by FSM state
    for i in range(len(v_rel) - 1):
        state_idx = int(fsm[i])
        ax.plot(v_rel[i:i + 2], dist[i:i + 2],
                color=_STATE_COLORS[state_idx], linewidth=2.0, alpha=0.8)

    # Start and end markers
    ax.plot(v_rel[0], dist[0], 'ko', markersize=10, zorder=10, label='Start')
    end_marker = 'rx' if result['collision'] else 'gs'
    end_label = 'Collision' if result['collision'] else 'Stopped / Safe'
    ax.plot(v_rel[-1], dist[-1], end_marker, markersize=12, markeredgewidth=3,
            zorder=10, label=end_label)

    # Legend entries for FSM state colors
    for i, name in enumerate(_STATE_NAMES):
        if np.any(fsm == i):
            ax.plot([], [], '-', color=_STATE_COLORS[i], linewidth=3, label=name)

    ax.set_xlabel('Relative Velocity (closing) [m/s]')
    ax.set_ylabel('Gap Distance [m]')
    ax.set_title('Safety Trajectory in Phase Space')
    ax.legend(loc='upper left', fontsize=8, ncol=2)
    ax.grid(True, alpha=0.3)
    ax.set_xlim(left=-0.5)
    ax.set_ylim(bottom=-1)

    plt.tight_layout(rect=[0, 0, 1, 0.93])

    if save_path:
        base, ext = os.path.splitext(save_path)
        path = base + '_phase' + ext
        plt.savefig(path, dpi=150, bbox_inches='tight')
        print(f"  Plot saved: {path}")

    if not _SHOW_PLOTS:
        plt.close()


# ===================================================================
# Window 5: Energy & Safety Margins
# ===================================================================
def plot_energy_safety(log, scenario: ScenarioConfig, result: dict,
                       config: AEBConfig, save_path: str = None):
    """
    Energy and safety analysis — 3 subplots:
      1. Kinetic energy over time (energy dissipation by braking)
      2. Safety margin (gap - d_brake): positive = safe, negative = can't stop
      3. Speed reduction percentage over time (Euro NCAP scoring metric)
    """
    s = _extract_signals(log)
    t = s['t']
    mass = config.vehicle.mass

    fig, axes = plt.subplots(3, 1, figsize=(14, 12), sharex=True)
    fig.canvas.manager.set_window_title(f"Energy & Safety — {scenario.name}")
    fig.suptitle(f"Energy & Safety Margins — {scenario.name}\n"
                 f"{'PASS' if result['passed'] else 'FAIL'} — {result['reason']}",
                 fontsize=13, fontweight='bold')

    # --- 1. Kinetic energy ---
    ax = axes[0]
    ke = 0.5 * mass * s['ego_v'] ** 2  # [J]
    ke_kj = ke / 1000.0  # [kJ]
    ax.plot(t, ke_kj, 'b-', linewidth=1.5, label='Ego kinetic energy')
    ax.fill_between(t, 0, ke_kj, alpha=0.15, color='blue')
    # Energy dissipated
    ke_dissipated = ke_kj[0] - ke_kj
    ax.plot(t, ke_dissipated, 'r--', linewidth=1.2,
            label='Energy dissipated by braking')
    ax.set_ylabel('Energy [kJ]')
    ax.set_title(f'Kinetic Energy — Vehicle mass = {mass} kg')
    ax.legend(loc='right')
    ax.grid(True, alpha=0.3)

    # --- 2. Safety margin ---
    ax = axes[1]
    margin = s['distance'] - s['d_brake']
    ax.plot(t, margin, 'b-', linewidth=1.5, label='Safety margin (gap − d_brake)')
    ax.axhline(y=0, color='r', linestyle='-', linewidth=1.5, alpha=0.7,
               label='Critical threshold (margin = 0)')
    ax.fill_between(t, margin, 0,
                    where=(margin >= 0), alpha=0.15, color='green',
                    interpolate=True)
    ax.fill_between(t, margin, 0,
                    where=(margin < 0), alpha=0.25, color='red',
                    interpolate=True)
    ax.set_ylabel('Margin [m]')
    ax.set_title('Safety Margin — Can the vehicle still stop in time?')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)

    # --- 3. Speed reduction ---
    ax = axes[2]
    v0 = s['ego_v'][0]
    if v0 > 0:
        speed_reduction_pct = (1.0 - s['ego_v'] / v0) * 100.0
    else:
        speed_reduction_pct = np.zeros_like(s['ego_v'])
    ax.plot(t, speed_reduction_pct, 'b-', linewidth=1.5,
            label='Speed reduction [%]')
    ax.axhline(y=100, color='g', linestyle='--', alpha=0.5, label='Full stop')
    # Show pass threshold if applicable
    if scenario.min_speed_reduction is not None and v0 > 0:
        threshold_pct = scenario.min_speed_reduction / v0 * 100.0
        ax.axhline(y=threshold_pct, color='orange', linestyle='--', alpha=0.7,
                    label=f'Pass threshold ({threshold_pct:.0f}%)')
    ax.set_ylabel('Reduction [%]')
    ax.set_xlabel('Time [s]')
    ax.set_title('Cumulative Speed Reduction — Euro NCAP Scoring Metric')
    ax.set_ylim(-5, 110)
    ax.legend(loc='lower right')
    ax.grid(True, alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.93])

    if save_path:
        base, ext = os.path.splitext(save_path)
        path = base + '_energy' + ext
        plt.savefig(path, dpi=150, bbox_inches='tight')
        print(f"  Plot saved: {path}")

    if not _SHOW_PLOTS:
        plt.close()


# ===================================================================
# Generate all plot windows for one scenario
# ===================================================================
def plot_all_windows(log, scenario, result, config, save_path=None):
    """Generate all 5 plot windows for a single scenario."""
    plot_scenario_results(log, scenario, result, save_path=save_path)
    plot_sensor_fusion(log, scenario, result, save_path=save_path)
    plot_pid_analysis(log, scenario, result, save_path=save_path)
    plot_phase_plane(log, scenario, result, save_path=save_path)
    plot_energy_safety(log, scenario, result, config, save_path=save_path)


# ===================================================================
# Runners
# ===================================================================
def run_all_scenarios(show_plots: bool = False):
    """Run all Euro NCAP scenarios and generate report."""
    config = AEBConfig()
    sim = AEBSimulation(config)
    scenarios = get_all_scenarios()

    results = []
    output_dir = os.path.join(os.path.dirname(__file__), '..', 'results')
    os.makedirs(output_dir, exist_ok=True)

    print("=" * 70)
    print("AEB SIL SIMULATION - Euro NCAP Test Matrix")
    print("=" * 70)
    print(f"Controller cycle: {config.sim.dt_controller*1000:.0f} ms")
    print(f"Plant step: {config.sim.dt*1000:.1f} ms")
    print(f"Sensor fusion: Radar (50 Hz) + LiDAR (20 Hz)")
    print(f"CAN bus: {config.can.baud_rate/1000:.0f} kbps")
    print("=" * 70)

    for i, scenario in enumerate(scenarios):
        print(f"\n[{i+1}/{len(scenarios)}] Running: {scenario.name}")
        print(f"  {scenario.description}")

        sim_result = sim.run(scenario)
        result = sim_result['result']
        results.append((scenario, result))

        status = "PASS" if result['passed'] else "FAIL"
        print(f"  Result: {status} - {result['reason']}")
        if result['collision']:
            print(f"  Impact speed: {result['impact_speed_kmh']:.1f} km/h")
            print(f"  Speed reduction: {result['speed_reduction_pct']:.1f}%")
        else:
            print(f"  Final gap: {result['final_distance_m']:.2f} m")

        # Generate all plots
        plot_path = os.path.join(output_dir, f"{scenario.name}.png")
        plot_all_windows(
            sim_result['log'], scenario, result, config, save_path=plot_path
        )

        if show_plots:
            plt.show()

    # Summary
    print("\n" + "=" * 70)
    print("SUMMARY")
    print("=" * 70)
    passed = sum(1 for _, r in results if r['passed'])
    total = len(results)
    print(f"Passed: {passed}/{total}")
    print()
    for scenario, result in results:
        status = "PASS" if result['passed'] else "FAIL"
        print(f"  [{status}] {scenario.name}: {result['reason']}")

    # CAN bus report
    print(f"\nCAN Bus Load: {sim.can.get_bus_load()*100:.2f}%")
    stats = sim.can.get_statistics()
    print(f"CAN Statistics: {stats}")

    return results


def run_single_scenario(name: str):
    """Run a single named scenario."""
    config = AEBConfig()
    sim = AEBSimulation(config)

    # Parse scenario name
    if name.startswith("CCRs"):
        speed = float(name.split("_")[1].replace("kmh", ""))
        scenario = get_ccrs_scenario(speed)
    elif name.startswith("CCRm"):
        scenario = get_ccrm_scenario()
    elif name.startswith("CCRb"):
        scenario = get_ccrb_scenario()
    else:
        print(f"Unknown scenario: {name}")
        return

    print(f"Running: {scenario.name} - {scenario.description}")
    result = sim.run(scenario)

    output_dir = os.path.join(os.path.dirname(__file__), '..', 'results')
    plot_path = os.path.join(output_dir, f"{scenario.name}.png")
    plot_all_windows(
        result['log'], scenario, result['result'], config, save_path=plot_path
    )

    r = result['result']
    print(f"Result: {'PASS' if r['passed'] else 'FAIL'} - {r['reason']}")

    if _SHOW_PLOTS:
        plt.show()


if __name__ == "__main__":
    args = sys.argv[1:]

    if "--scenario" in args:
        idx = args.index("--scenario")
        if idx + 1 < len(args):
            run_single_scenario(args[idx + 1])
        else:
            print("Usage: python main_sim.py --scenario CCRs_40kmh")
    else:
        run_all_scenarios(show_plots=_SHOW_PLOTS)
