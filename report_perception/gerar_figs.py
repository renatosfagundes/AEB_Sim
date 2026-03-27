"""
gerar_figs.py  —  Figures for AEB Perception Layer report (v3 — physics-correct)

Sensor physics modeled correctly:
  Radar  (FMCW) : measures range + radial velocity (Doppler)
  LiDAR  (ToF)  : measures range ONLY (velocity derived by finite difference)
  WheelIMU      : measures ego speed (wheel speed sensor + IMU)
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(SCRIPT_DIR, 'figs')
os.makedirs(OUT_DIR, exist_ok=True)
DPI = 180

C_RADAR  = '#2E6FBF'
C_LIDAR  = '#2E9F4E'
C_WHEEL  = '#8B4FA0'
C_FUSION = '#E07B20'
C_FAULT  = '#C0392B'
C_TRUE   = '#222222'
C_BORDER = '#1E3C64'
C_LIGHT  = '#EEF4FB'


# ─────────────────────────────────────────────────────────────────────────────
# FIG 1 — Physics-correct architecture
# ─────────────────────────────────────────────────────────────────────────────
def fig_arquitetura():
    fig, ax = plt.subplots(figsize=(14, 6))
    ax.set_xlim(0, 14); ax.set_ylim(0, 6); ax.axis('off')
    fig.patch.set_facecolor('white')
    ax.set_title('Arquitetura da Camada de Percepção AEB (Modelos Fisicamente Corretos)',
                 fontsize=13, fontweight='bold', color=C_BORDER, pad=10)

    def box(x, y, w, h, title, sub='', fc='#D0E4F7', ec='#2E5B8A', fs=10):
        ax.add_patch(FancyBboxPatch((x, y), w, h,
            boxstyle='round,pad=0.1', lw=1.8, edgecolor=ec, facecolor=fc))
        ax.text(x+w/2, y+h/2+(0.15 if sub else 0), title,
                ha='center', va='center', fontsize=fs, fontweight='bold', color=C_BORDER)
        if sub:
            ax.text(x+w/2, y+h/2-0.3, sub,
                    ha='center', va='center', fontsize=7.5, color='#555555')

    def arrow(x0, y0, x1, y1, lbl='', lc='#333333'):
        ax.annotate('', xy=(x1,y1), xytext=(x0,y0),
                    arrowprops=dict(arrowstyle='->', color=lc, lw=1.5))
        if lbl:
            mx,my=(x0+x1)/2,(y0+y1)/2
            ax.text(mx, my+0.18, lbl, ha='center', fontsize=7.5, color='#555555', style='italic')

    # Plant
    box(0.2, 2.2, 1.5, 1.6, 'Planta AEB', 'AEB_Plant.slx', '#E8F0E8', '#2E5B2E', 9)

    # Radar
    box(2.8, 4.0, 2.4, 1.2, 'Radar (FMCW)',
        'Saída: d [m] + v_rel [m/s]\n(distância + Doppler direto)',
        C_RADAR+'22', C_RADAR, 9)

    # LiDAR
    box(2.8, 2.4, 2.4, 1.2, 'LiDAR (ToF)',
        'Saída: d [m] apenas\n(velocidade NÃO medida)',
        C_LIDAR+'22', C_LIDAR, 9)

    # WheelIMU
    box(2.8, 0.8, 2.4, 1.2, 'Wheel/IMU',
        'Saída: v_ego [m/s]\n(NÃO vem do radar/lidar)',
        C_WHEEL+'22', C_WHEEL, 9)

    # Fault detect
    box(6.5, 3.5, 2.2, 1.0, 'Detecção\nRadar', 'range + Δv', '#FDE8E8', C_FAULT, 9)
    box(6.5, 2.2, 2.2, 1.0, 'Detecção\nLiDAR', 'range only', '#FDE8E8', C_FAULT, 9)

    # Kalman Filter
    box(9.5, 2.0, 2.6, 2.2, 'Filtro de\nKalman (2D)',
        'x=[d; v_rel]\nH_r=I₂  H_l=[1,0]\nAtual. sequencial',
        '#FFF3D0', C_FUSION, 10)

    # Controller
    box(12.5, 2.4, 1.3, 1.4, 'Controlador\nAEB', '', '#E8F0E8', '#2E5B2E', 9)

    # Connections from plant
    arrow(1.7, 3.2, 2.8, 4.6, 'd_true, v_rel')
    arrow(1.7, 3.0, 2.8, 3.0, 'd_true')
    arrow(1.7, 2.8, 2.8, 1.4, 'v_ego')

    # Sensor → Fault detect
    arrow(5.2, 4.6, 6.5, 4.0)
    arrow(5.2, 3.0, 6.5, 2.7)

    # Sensor + Fault → KF
    arrow(5.2, 4.6, 9.5, 3.5, 'd_r, vr_r')
    arrow(5.2, 3.0, 9.5, 3.0, 'd_l')
    arrow(5.2, 1.4, 9.5, 2.5, 'v_ego')
    arrow(8.7, 4.0, 9.5, 3.8, 'r_fault')
    arrow(8.7, 2.7, 9.5, 2.8, 'l_fault')

    # KF → Controller
    arrow(12.1, 3.1, 12.5, 3.1)

    # Physics notes
    ax.text(4.0, 0.35,
        '★  Radar: Efeito Doppler → v_rel direta  |  LiDAR: tempo-de-voo → apenas distância  |  v_ego: sensor de roda + IMU',
        ha='center', fontsize=8.5, color='#555555',
        bbox=dict(boxstyle='round', fc='#FFFBE8', ec='#CCAA00', alpha=0.9))

    handles = [
        mpatches.Patch(fc=C_RADAR+'33', ec=C_RADAR, label='Radar — d + v_rel'),
        mpatches.Patch(fc=C_LIDAR+'33', ec=C_LIDAR, label='LiDAR — d apenas'),
        mpatches.Patch(fc=C_WHEEL+'33', ec=C_WHEEL, label='Wheel/IMU — v_ego'),
        mpatches.Patch(fc='#FFF3D0',    ec=C_FUSION, label='Kalman Filter 2D'),
    ]
    ax.legend(handles=handles, loc='lower right', ncol=2, fontsize=8.5,
              framealpha=0.9, bbox_to_anchor=(1.0, 0.0))

    plt.tight_layout()
    plt.savefig(os.path.join(OUT_DIR,'fig_arquitetura.png'), dpi=DPI, bbox_inches='tight')
    plt.close(); print('  fig_arquitetura.png OK')


# ─────────────────────────────────────────────────────────────────────────────
# FIG 2 — Sensor measurements (physics-correct)
#   Radar   : noisy d + noisy v_rel (Doppler, direct)
#   LiDAR   : noisy d only; derived v_rel via finite diff (much noisier)
#   WheelIMU: v_ego directly (separate sensor)
# ─────────────────────────────────────────────────────────────────────────────
def fig_ruido_sensor():
    np.random.seed(42)
    dt = 0.001; T = 8.0
    t  = np.arange(0, T, dt)
    v_rel = 10.0; v_ego = 13.89

    d_true  = np.maximum(0.0, 60.0 - v_rel * t)
    vr_true = v_rel * np.ones_like(t)
    ve_true = v_ego * np.ones_like(t)

    # Radar: range + Doppler velocity (both direct, 20 ms ZOH)
    RADAR_RATE = 0.020; RADAR_LAT = 0.015
    tr = np.arange(0, T, RADAR_RATE)
    d_r_s  = np.interp(tr, t - RADAR_LAT, d_true,  left=d_true[0])  + np.random.normal(0, 0.30, len(tr))
    vr_r_s = np.interp(tr, t - RADAR_LAT, vr_true, left=vr_true[0]) + np.random.normal(0, 0.10, len(tr))
    d_radar  = np.interp(t, tr, np.maximum(0.5, d_r_s))
    vr_radar = np.interp(t, tr, vr_r_s)

    # LiDAR: range only (50 ms ZOH, better range noise)
    LIDAR_RATE = 0.050; LIDAR_LAT = 0.010
    tl = np.arange(0, T, LIDAR_RATE)
    d_l_s  = np.interp(tl, t - LIDAR_LAT, d_true, left=d_true[0]) + np.random.normal(0, 0.05, len(tl))
    d_lidar = np.interp(t, tl, np.maximum(1.0, d_l_s))
    # Velocity DERIVED from lidar by finite difference — very noisy
    vr_lidar_derived = -np.gradient(d_lidar, dt)  # negative because d decreases when approaching

    # Wheel/IMU: ego speed
    WHEEL_RATE = 0.010
    tw = np.arange(0, T, WHEEL_RATE)
    ve_w_s = v_ego + np.random.normal(0, 0.05, len(tw))
    ve_wheel = np.interp(t, tw, ve_w_s)

    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    fig.patch.set_facecolor('white')
    fig.suptitle('Modelos de Sensor Fisicamente Corretos: Medições vs. Verdade',
                 fontsize=13, fontweight='bold', color=C_BORDER)

    # Panel 1: Distance measurements
    ax = axes[0]
    ax.plot(t, d_true,  '-',  color=C_TRUE,   lw=2.2, label='d verdadeiro', zorder=4)
    ax.plot(t, d_radar, '--', color=C_RADAR,   lw=1.2, label=f'Radar: d_r (σ=0.30 m, 20 ms)', alpha=0.8)
    ax.plot(t, d_lidar, '-',  color=C_LIDAR,   lw=1.5, label=f'LiDAR: d_l (σ=0.05 m, 50 ms)', alpha=0.8)
    ax.set_ylabel('Distância [m]', fontsize=11)
    ax.set_title('Distância: ambos os sensores medem, LiDAR mais preciso', fontsize=11)
    ax.legend(fontsize=9); ax.grid(True, alpha=0.4); ax.set_xlim(0,T)

    # Panel 2: Relative speed measurements
    ax = axes[1]
    ax.plot(t, vr_true,          '-',  color=C_TRUE,   lw=2.2, label='v_rel verdadeiro', zorder=4)
    ax.plot(t, vr_radar,         '--', color=C_RADAR,   lw=1.5,
            label=f'Radar: vr_r via Doppler (σ=0.10 m/s) ← direto', alpha=0.9)
    ax.plot(t, vr_lidar_derived, '-',  color=C_LIDAR,   lw=1.0,
            label=f'LiDAR: ΔdL/Δt derivado (σ≈{np.std(vr_lidar_derived-v_rel):.2f} m/s) ← muito ruidoso', alpha=0.7)
    ax.set_ylabel('Velocidade relativa [m/s]', fontsize=11)
    ax.set_title('Vel. relativa: Radar por Doppler (preciso) vs. LiDAR por diferença finita (ruidoso)',
                 fontsize=11)
    ax.legend(fontsize=9); ax.grid(True, alpha=0.4); ax.set_xlim(0,T)
    ax.text(0.5, 0.05,
            '★  LiDAR (ToF) NÃO mede velocidade diretamente — derivada de Δd/Δt resulta em ruído ≈10× maior',
            transform=ax.transAxes, ha='center', fontsize=8.5,
            bbox=dict(boxstyle='round', fc='#FFF3D0', ec=C_FAULT, alpha=0.85))

    # Panel 3: Ego speed
    ax = axes[2]
    ax.plot(t, ve_true,  '-',  color=C_TRUE,  lw=2.2, label='v_ego verdadeiro')
    ax.plot(t, ve_wheel, '--', color=C_WHEEL, lw=1.5,
            label='Wheel/IMU (σ=0.05 m/s) ← sensor de roda + IMU')
    ax.set_ylabel('Velocidade ego [m/s]', fontsize=11)
    ax.set_xlabel('Tempo [s]', fontsize=11)
    ax.set_title('Velocidade do ego: sensor de roda + IMU (NÃO vem do radar ou LiDAR)', fontsize=11)
    ax.legend(fontsize=9); ax.grid(True, alpha=0.4); ax.set_xlim(0,T)

    plt.tight_layout()
    plt.savefig(os.path.join(OUT_DIR,'fig_ruido_sensor.png'), dpi=DPI, bbox_inches='tight')
    plt.close(); print('  fig_ruido_sensor.png OK')


# ─────────────────────────────────────────────────────────────────────────────
# FIG 3 — Fault detection logic
# ─────────────────────────────────────────────────────────────────────────────
def fig_deteccao_falha():
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    fig.patch.set_facecolor('white')
    fig.suptitle('Lógica de Detecção de Falha por Sensor', fontsize=13,
                 fontweight='bold', color=C_BORDER)

    for ax, title, signals, color in [
        (axes[0], 'RadarFault — verifica d_r E vr_r',
         ['d_r ∈ [0.5, 200] m', '|vr_r| ≤ 50 m/s', '|Δd| ≤ 10 m/ciclo', '|Δvr| ≤ 2 m/s/ciclo'],
         C_RADAR),
        (axes[1], 'LidarFault — verifica apenas d_l',
         ['d_l ∈ [1.0, 100] m', '|Δd| ≤ 10 m/ciclo',
          '(sem verificação de velocidade)', '(LiDAR não mede velocidade)'],
         C_LIDAR),
    ]:
        ax.set_xlim(0, 8); ax.set_ylim(0, 6); ax.axis('off')
        ax.set_facecolor('white')
        ax.set_title(title, fontsize=11, fontweight='bold', color=color)

        # Check boxes
        for i, sig in enumerate(signals):
            y = 4.8 - i * 1.0
            fc = '#FFFBE8' if '(sem' in sig or '(LiDAR' in sig else '#FEF0E0'
            ec = '#CCAA00' if '(sem' in sig or '(LiDAR' in sig else color
            ax.add_patch(FancyBboxPatch((0.3, y-0.35), 5.0, 0.65,
                boxstyle='round,pad=0.1', lw=1.5, edgecolor=ec, facecolor=fc))
            ax.text(0.8, y, sig, va='center', fontsize=10,
                    color='#888888' if '(sem' in sig or '(LiDAR' in sig else C_BORDER)

        # OR gate
        ax.add_patch(FancyBboxPatch((5.7, 2.0), 1.8, 0.7,
            boxstyle='round,pad=0.1', lw=1.8, edgecolor=C_FAULT, facecolor='#FDE8E8'))
        ax.text(6.6, 2.35, 'OR\nfault_ciclo', ha='center', va='center',
                fontsize=9, fontweight='bold', color=C_FAULT)

        # Counter
        ax.add_patch(FancyBboxPatch((5.7, 0.8), 1.8, 0.7,
            boxstyle='round,pad=0.1', lw=1.8, edgecolor='#A04000', facecolor='#FFE0D0'))
        ax.text(6.6, 1.15, 'Contador ≥ 3?', ha='center', va='center',
                fontsize=9, fontweight='bold', color='#A04000')

        ax.annotate('', xy=(5.7, 2.35), xytext=(5.3, 4.8),
                    arrowprops=dict(arrowstyle='->', color=C_FAULT, lw=1.2))
        ax.annotate('', xy=(5.7, 2.35), xytext=(5.3, 3.8),
                    arrowprops=dict(arrowstyle='->', color=C_FAULT, lw=1.2))
        ax.annotate('', xy=(6.6, 2.0), xytext=(6.6, 1.5),
                    arrowprops=dict(arrowstyle='->', color=C_FAULT, lw=1.2))
        ax.text(6.6, 0.35, 'sensor_fault', ha='center', fontsize=9,
                fontweight='bold', color=C_FAULT)

    plt.tight_layout()
    plt.savefig(os.path.join(OUT_DIR,'fig_deteccao_falha.png'), dpi=DPI, bbox_inches='tight')
    plt.close(); print('  fig_deteccao_falha.png OK')


# ─────────────────────────────────────────────────────────────────────────────
# FIG 4 — Fault injection scenario
# ─────────────────────────────────────────────────────────────────────────────
def fig_injecao_falha():
    np.random.seed(7)
    dt = 0.001; T = 12.0
    t  = np.arange(0, T, dt); N = len(t)
    d_true = np.maximum(0.0, 80.0 - 10.0 * t)

    d_radar = d_true + np.random.normal(0, 0.30, N)
    fi_mask = (t >= 4.0) & (t < 6.0)
    d_radar[fi_mask] += np.random.uniform(15, 25, fi_mask.sum())
    d_radar = np.maximum(0.5, d_radar)

    ROC_MAX = 10.0
    fault_cycle = np.zeros(N); prev = d_radar[0]
    for i in range(1, N):
        if abs(d_radar[i] - prev) > ROC_MAX: fault_cycle[i] = 1
        prev = d_radar[i]

    counter = np.zeros(N)
    for i in range(1, N):
        counter[i] = counter[i-1] + 1 if fault_cycle[i] else 0

    fault_flag = (counter >= 3).astype(float)
    confidence = 1.0 - fault_flag

    fig, axes = plt.subplots(4, 1, figsize=(11, 9), sharex=True)
    fig.patch.set_facecolor('white')
    fig.suptitle('Injeção de Falha no Radar (t = 4–6 s)\nDetecção por taxa de variação |Δd| > 10 m',
                 fontsize=12, fontweight='bold', color=C_BORDER)

    for ax, y_data, ylabel, clr, extra in [
        (axes[0], (d_true, d_radar),      'Distância [m]',  C_RADAR, None),
        (axes[1], (np.abs(np.diff(d_radar, prepend=d_radar[0])),), '|Δd| [m]', C_FUSION, ROC_MAX),
        (axes[2], (counter,),             'Contador',        '#8B4513', 3),
        (axes[3], (fault_flag, confidence),'Saídas',         C_FAULT, None),
    ]:
        if ax is axes[0]:
            ax.plot(t, y_data[0], '-', color=C_TRUE,  lw=2.0, label='d_true')
            ax.plot(t, y_data[1], '-', color=C_RADAR, lw=1.0, label='d_radar (com falha)', alpha=0.7)
            ax.legend(fontsize=9)
        elif ax is axes[1]:
            ax.plot(t, y_data[0], '-', color=clr, lw=1.2)
            ax.axhline(extra, color=C_FAULT, ls='--', lw=1.5, label=f'Limiar = {extra} m')
            ax.legend(fontsize=9)
        elif ax is axes[2]:
            ax.plot(t, y_data[0], '-', color=clr, lw=1.2)
            ax.axhline(extra, color=C_FAULT, ls='--', lw=1.5, label=f'Limiar latch = {extra}')
            ax.legend(fontsize=9)
        else:
            ax.fill_between(t, y_data[0], step='post', color=C_FAULT, alpha=0.35, label='fault_flag')
            ax.plot(t, y_data[1], '-', color=C_LIDAR, lw=1.8, label='confiança')
            ax.set_ylim(-0.1, 1.3); ax.legend(fontsize=9)
            ax.set_xlabel('Tempo [s]', fontsize=11)

        ax.axvspan(4.0, 6.0, alpha=0.10, color=C_FAULT)
        ax.set_ylabel(ylabel, fontsize=9); ax.grid(True, alpha=0.4)

    plt.tight_layout()
    plt.savefig(os.path.join(OUT_DIR,'fig_injecao_falha.png'), dpi=DPI, bbox_inches='tight')
    plt.close(); print('  fig_injecao_falha.png OK')


# ─────────────────────────────────────────────────────────────────────────────
# FIG 5 — Latency comparison
# ─────────────────────────────────────────────────────────────────────────────
def fig_latencia():
    np.random.seed(99); dt = 0.001; T = 1.0
    t = np.arange(0, T, dt); step_t = 0.10
    d_true = np.where(t >= step_t, 50.0, 60.0)

    def zoh(d, t, rate, lat, sigma, seed=0):
        np.random.seed(seed)
        delayed = np.interp(t - lat, t, d, left=d[0])
        sr = np.arange(0, T, rate)
        samp = np.interp(sr, t, delayed) + np.random.normal(0, sigma, len(sr))
        return np.interp(t, sr, samp)

    d_r = zoh(d_true, t, 0.020, 0.015, 0.30, 1)
    d_l = zoh(d_true, t, 0.050, 0.010, 0.05, 2)

    fig, ax = plt.subplots(figsize=(10, 4.5))
    fig.patch.set_facecolor('white')
    ax.plot(t*1000, d_true, '-',  color=C_TRUE,  lw=2.2, label='Verdadeiro (degrau)')
    ax.plot(t*1000, d_r,    '--', color=C_RADAR, lw=1.6, label='Radar  (σ=0.30 m, lat=15 ms, rate=20 ms)')
    ax.plot(t*1000, d_l,    '-',  color=C_LIDAR, lw=1.6, label='LiDAR  (σ=0.05 m, lat=10 ms, rate=50 ms)')
    ax.axvline(step_t*1000, color='grey', lw=0.8, ls='--', alpha=0.5)

    ax.annotate('LiDAR mais preciso\npara distância',
                xy=(160, 52), xytext=(200, 56),
                arrowprops=dict(arrowstyle='->', color=C_LIDAR), color=C_LIDAR, fontsize=9)

    ax.set_xlabel('Tempo [ms]', fontsize=11)
    ax.set_ylabel('Distância [m]', fontsize=11)
    ax.set_title('Latência e Precisão de Distância: Radar vs. LiDAR\n'
                 '(Radar compensa imprecisão de range com medição de velocidade Doppler)',
                 fontsize=11, fontweight='bold')
    ax.legend(fontsize=9, loc='lower right'); ax.grid(True, alpha=0.4)
    ax.set_xlim(80, 400); ax.set_ylim(45, 65)

    plt.tight_layout()
    plt.savefig(os.path.join(OUT_DIR,'fig_latencia.png'), dpi=DPI, bbox_inches='tight')
    plt.close(); print('  fig_latencia.png OK')


# ─────────────────────────────────────────────────────────────────────────────
# FIG 6 — Kalman Filter 2D with sequential updates
# ─────────────────────────────────────────────────────────────────────────────
def fig_fusao():
    np.random.seed(55)
    dt = 0.001; T = 8.0
    t  = np.arange(0, T, dt); N = len(t)

    v_rel = 10.0; d0 = 80.0
    d_true  = np.maximum(0.0, d0 - v_rel * t)
    vr_true = v_rel * np.ones(N)

    # Radar: d + vr (Doppler)
    tr = np.arange(0, T, 0.020)
    d_r_s  = np.interp(tr, t, d_true) + np.random.normal(0, 0.30, len(tr))
    vr_r_s = v_rel + np.random.normal(0, 0.10, len(tr))
    d_radar  = np.interp(t, tr, np.maximum(0.5, d_r_s))
    vr_radar = np.interp(t, tr, vr_r_s)

    # LiDAR: d only
    tl = np.arange(0, T, 0.050)
    d_l_s  = np.interp(tl, t, d_true) + np.random.normal(0, 0.05, len(tl))
    d_lidar = np.interp(t, tl, np.maximum(1.0, d_l_s))

    # Kalman Filter 2D
    DT = dt; F = np.array([[1, -DT],[0, 1]])
    Q = np.diag([1e-4, 1e-5])
    H_l = np.array([[1, 0]]); R_l = np.array([[0.0025]])
    H_r = np.eye(2); R_r = np.diag([0.09, 0.01])

    x = np.array([d_radar[0], vr_radar[0]]); P = np.diag([1.0, 0.25])
    d_kf = np.zeros(N); vr_kf = np.zeros(N)

    for k in range(N):
        xp = F @ x; Pp = F @ P @ F.T + Q
        x_u = xp.copy(); P_u = Pp.copy()
        # LiDAR update (range)
        S_l = H_l @ P_u @ H_l.T + R_l
        K_l = P_u @ H_l.T @ np.linalg.inv(S_l)
        x_u = x_u + K_l.flatten() * (d_lidar[k] - H_l @ x_u).item()
        P_u = (np.eye(2) - K_l @ H_l) @ P_u
        # Radar update (range + velocity)
        S_r = H_r @ P_u @ H_r.T + R_r
        K_r = P_u @ H_r.T @ np.linalg.inv(S_r)
        z_r = np.array([d_radar[k], vr_radar[k]])
        x_u = x_u + K_r @ (z_r - H_r @ x_u)
        P_u = (np.eye(2) - K_r @ H_r) @ P_u
        x = x_u; P = P_u
        x[0] = max(0.5, min(300, x[0])); x[1] = max(-50, min(50, x[1]))
        d_kf[k] = x[0]; vr_kf[k] = x[1]

    rmse = lambda a,b: np.sqrt(np.mean((a-b)**2))

    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    fig.patch.set_facecolor('white')
    fig.suptitle('Filtro de Kalman 2D: Atualização Sequencial LiDAR→Radar',
                 fontsize=13, fontweight='bold', color=C_BORDER)

    ax = axes[0]
    ax.plot(t, d_true,  '-',  color=C_TRUE,   lw=2.2, label='d verdadeiro')
    ax.plot(t, d_radar, '--', color=C_RADAR,   lw=1.0, label=f'Radar d_r (RMSE={rmse(d_radar,d_true):.3f} m)', alpha=0.6)
    ax.plot(t, d_lidar, ':',  color=C_LIDAR,   lw=1.2, label=f'LiDAR d_l (RMSE={rmse(d_lidar,d_true):.3f} m)', alpha=0.8)
    ax.plot(t, d_kf,    '-',  color=C_FUSION,  lw=2.0, label=f'KF d_fused (RMSE={rmse(d_kf,d_true):.3f} m)')
    ax.set_ylabel('Distância [m]', fontsize=11)
    ax.set_title('Distância — KF funde range do LiDAR (preciso) com range+vel do Radar', fontsize=11)
    ax.legend(fontsize=9); ax.grid(True, alpha=0.4); ax.set_xlim(0,T)

    ax = axes[1]
    ax.plot(t, vr_true,  '-',  color=C_TRUE,   lw=2.2, label='v_rel verdadeiro')
    ax.plot(t, vr_radar, '--', color=C_RADAR,   lw=1.0, label=f'Radar vr_r via Doppler (RMSE={rmse(vr_radar,vr_true):.3f} m/s)', alpha=0.8)
    ax.plot(t, vr_kf,    '-',  color=C_FUSION,  lw=2.0, label=f'KF vr_fused (RMSE={rmse(vr_kf,vr_true):.4f} m/s)')
    ax.set_ylabel('v_rel [m/s]', fontsize=11)
    ax.set_xlabel('Tempo [s]', fontsize=11)
    ax.set_title('Velocidade relativa — LiDAR NÃO contribui (sem Doppler); apenas Radar atualiza v_rel', fontsize=11)
    ax.text(0.5, 0.08,
            '★  Radar via Doppler: única fonte direta de v_rel  |  LiDAR: só corrige distância no KF',
            transform=ax.transAxes, ha='center', fontsize=9,
            bbox=dict(boxstyle='round', fc='#FFF3D0', ec=C_FUSION, alpha=0.9))
    ax.legend(fontsize=9); ax.grid(True, alpha=0.4); ax.set_xlim(0,T)

    plt.tight_layout()
    plt.savefig(os.path.join(OUT_DIR,'fig_fusao.png'), dpi=DPI, bbox_inches='tight')
    plt.close(); print('  fig_fusao.png OK')


# ─────────────────────────────────────────────────────────────────────────────
# FIG 7 — Parameter table
# ─────────────────────────────────────────────────────────────────────────────
def fig_parametros():
    fig, ax = plt.subplots(figsize=(13, 6.5))
    fig.patch.set_facecolor('white'); ax.axis('off')
    ax.set_title('Parâmetros dos Modelos de Sensor (Fisicamente Justificados)',
                 fontsize=13, fontweight='bold', color=C_BORDER, pad=8)

    cols = ['Parâmetro', 'Radar (FMCW)', 'LiDAR (ToF)', 'Wheel/IMU', 'Unidade', 'Justificativa']
    rows = [
        ['Ruído de distância (σ)', '0,30', '0,05', '—', 'm', 'LiDAR ~6× mais preciso em range (ToF vs FMCW)'],
        ['Medição de velocidade', 'Doppler direto', 'NÃO MEDE', '—', '—', 'Diferença fundamental de física de sensor'],
        ['Ruído velocidade relativa', '0,10 m/s', 'N/A direto\n(Δd/Δt≈±1 m/s)', '—', 'm/s', 'Radar: resolução Doppler; LiDAR: derivada numérica'],
        ['Ruído velocidade ego', '—', '—', '0,05 m/s', 'm/s', 'Sensor de roda + integração IMU'],
        ['Taxa de atualização', '20', '50', '10', 'ms', 'DSP FMCW / varredura mecânica / loop ESC'],
        ['Latência', '15', '10', '5', 'ms', 'CAN stack + DSP / point cloud proc / CAN'],
        ['Alcance mínimo', '0,5', '1,0', '—', 'm', 'Blind zone próximo'],
        ['Alcance máximo', '200', '100', '—', 'm', 'AEB: TTC relevante < 100m a 50 km/h'],
        ['KF: Modelo H',  'eye(2): d+vr', '[1,0]: d only', 'pass-through', '—', 'LiDAR não tem coluna de velocidade em H'],
        ['KF: Variância R', 'diag([0.09, 0.01])', '0.0025', '—', 'm²/m²s²', 'σ_r=0.3, σ_v=0.1; σ_l=0.05'],
    ]
    row_colors = [['#EEF4FB' if i%2==0 else 'white']*6 for i in range(len(rows))]
    tbl = ax.table(cellText=rows, colLabels=cols, cellLoc='center',
                   loc='center', cellColours=row_colors)
    tbl.auto_set_font_size(False); tbl.set_fontsize(8.2); tbl.scale(1.0, 1.55)
    for j in range(len(cols)):
        tbl[(0,j)].set_facecolor(C_BORDER)
        tbl[(0,j)].set_text_props(color='white', fontweight='bold')
    tbl.auto_set_column_width(list(range(len(cols))))

    plt.tight_layout()
    plt.savefig(os.path.join(OUT_DIR,'fig_parametros.png'), dpi=DPI, bbox_inches='tight')
    plt.close(); print('  fig_parametros.png OK')


# ─────────────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    print(f'Gerando figuras em: {OUT_DIR}')
    fig_arquitetura()
    fig_ruido_sensor()
    fig_deteccao_falha()
    fig_injecao_falha()
    fig_latencia()
    fig_fusao()
    fig_parametros()
    print('\nTodas as figuras geradas com sucesso.')
