"""
gerar_figs.py
Gera todas as figuras para relatorio_uds.tex
Figuras produzidas (pasta figs/):
  fig_uds_architecture.pdf    -- Diagrama de camadas UDS / ISO 14229
  fig_session_states.pdf      -- Máquina de estados de sessão UDS
  fig_security_access.pdf     -- Fluxo seed-key (SID 0x27) com LFSR
  fig_dtc_lifecycle.pdf       -- Ciclo de vida DTC: inactive→pending→confirmed→cleared
  fig_did_read.pdf            -- Timeline ReadDataByIdentifier (SID 0x22)
  fig_calibration_write.pdf   -- Sequência WriteDID com security unlock
  fig_key_algorithm.pdf       -- Visualização do algoritmo seed-key
"""

import os
import struct
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
from matplotlib.lines import Line2D

script_dir = os.path.dirname(os.path.abspath(__file__))
fig_dir = os.path.join(script_dir, 'figs')
os.makedirs(fig_dir, exist_ok=True)

plt.rcParams.update({
    'font.family': 'DejaVu Sans',
    'font.size': 9,
    'axes.titlesize': 10,
    'axes.labelsize': 9,
    'legend.fontsize': 8,
    'xtick.labelsize': 8,
    'ytick.labelsize': 8,
    'figure.dpi': 150,
    'savefig.dpi': 200,
    'savefig.bbox': 'tight',
    'savefig.pad_inches': 0.1,
})

BLUE   = '#3a7abf'
RED    = '#cc3322'
GREEN  = '#2a9a4a'
PURPLE = '#7a30c0'
ORANGE = '#e06000'
GRAY   = '#888888'
CYAN   = '#1ab0b0'
DKBLUE = '#1a3a6f'
LGRAY  = '#dddddd'

# ──────────────────────────────────────────────────────────────────────────────
# Helpers de criptografia (Python mirror do MATLAB)
# ──────────────────────────────────────────────────────────────────────────────
MASK32 = 0xFFFFFFFF

def lfsr32(x):
    """Galois LFSR 32 bits -- polinomio 0x80200003."""
    POLY = 0x80200003
    if x & 1:
        return ((x >> 1) ^ POLY) & MASK32
    return (x >> 1) & MASK32

def rotl32(x, n):
    return ((x << n) | (x >> (32 - n))) & MASK32

def compute_key(seed):
    """key = ROTL32(seed ^ 0xA55A3CC3, 7) + 0x12345678 mod 2^32"""
    SECRET_XOR = 0xA55A3CC3
    SECRET_ADD = 0x12345678
    masked  = (seed ^ SECRET_XOR) & MASK32
    rotated = rotl32(masked, 7)
    return (rotated + SECRET_ADD) & MASK32

# ──────────────────────────────────────────────────────────────────────────────
# Simula ciclos UDS para geração de gráficos
# ──────────────────────────────────────────────────────────────────────────────
DT   = 0.1
TEND = 30.0
S3   = 5.0
DEB_FAULT = 3
DEB_ALIVE = 5
TEMP_MAX  = 85.0

def simulate_uds(scenario='dtc_lifecycle'):
    t = np.arange(0, TEND + DT, DT)
    N = len(t)

    # System inputs
    timeout_err = np.zeros(N)
    crc_err     = np.zeros(N)
    alive_err   = np.zeros(N)
    act_err     = np.zeros(N)
    ecu_temp    = np.full(N, 25.0)

    if scenario == 'dtc_lifecycle':
        timeout_err[int(5/DT):int(25/DT)] = 1.0
        crc_err[int(13/DT):int(14/DT)]    = 1.0
    elif scenario == 'calibration':
        pass  # no faults
    elif scenario == 'session_timeout':
        pass

    # Request sequence: list of (t_idx, SID, bytes...)
    req_seq = []
    if scenario == 'dtc_lifecycle':
        req_seq = [
            (int(1/DT),  0x10, [0x03]),           # Programming session
            (int(2/DT),  0x19, [0x02, 0xFF]),      # ReadDTC before fault
            (int(8/DT),  0x19, [0x02, 0xFF]),      # After fault confirmed
            (int(14/DT), 0x19, [0x02, 0xFF]),      # C1004 also
            (int(25/DT), 0x14, [0xFF, 0xFF, 0xFF]),# ClearDTC
            (int(27/DT), 0x19, [0x02, 0xFF]),      # Confirm cleared
        ]
    elif scenario == 'calibration':
        seed = lfsr32(0xABCD1234)
        key  = compute_key(seed)
        kb   = list(struct.pack('<I', key))
        ttc_bytes = list(struct.pack('<f', 3.5))
        req_seq = [
            (int(1/DT),  0x10, [0x03]),
            (int(3/DT),  0x27, [0x01]),            # RequestSeed
            (int(4/DT),  0x27, [0x02] + kb),       # SendKey
            (int(5/DT),  0x2E, [0xF2, 0x00] + ttc_bytes),  # WriteDID
        ]
    elif scenario == 'session_timeout':
        req_seq = [
            (int(1/DT),  0x10, [0x02]),
            (int(2/DT),  0x27, [0x01]),
            (int(10/DT), 0x10, [0x02]),
            (int(11/DT), 0x3E, [0x00]),
        ]

    # State
    sess      = 1
    s3_timer  = 0.0
    seed_v    = 0xABCD1234
    unlocked  = False
    dtc       = [0] * 7
    dtc_ctr   = [0] * 7
    ttc_w     = 4.0; ttc_l1 = 3.0; kp = 10.0
    aeb_en    = 1

    # Output arrays
    out_sess    = np.zeros(N)
    out_unlock  = np.zeros(N)
    out_dtc_cnt = np.zeros(N)
    out_lamp    = np.zeros(N)
    out_aeb_en  = np.ones(N)
    out_ttc_w   = np.full(N, 4.0)
    out_ttc_l1  = np.full(N, 3.0)
    out_kp      = np.full(N, 10.0)
    out_resp_sid= np.zeros(N)  # SID of response (0 = none)

    req_map = {k[0]: k for k in req_seq}

    fault_arr = [
        timeout_err, np.zeros(N), np.zeros(N),  # C1001,C1002,C1003
        crc_err, alive_err, act_err,              # C1004,C1005,C1006
        (ecu_temp > TEMP_MAX).astype(float)       # C1007
    ]
    deb_lim = [DEB_FAULT, DEB_FAULT, DEB_FAULT,
               DEB_FAULT, DEB_ALIVE, DEB_FAULT, DEB_FAULT]

    for k in range(N):
        # DTC update
        for i in range(7):
            if fault_arr[i][k] > 0:
                dtc_ctr[i] += 1
                if dtc_ctr[i] >= deb_lim[i]:
                    dtc[i] = 1
            else:
                dtc_ctr[i] = 0

        # Session S3 timer
        if sess > 1:
            s3_timer += DT
            if s3_timer >= S3:
                sess = 1; unlocked = False; s3_timer = 0.0

        # Process request if any
        resp_sid = 0
        if k in req_map:
            _, SID, payload = req_map[k]
            if SID == 0x10:
                sub = payload[0]
                sess = sub; unlocked = False; s3_timer = 0.0
                resp_sid = 0x50
            elif SID == 0x3E:
                s3_timer = 0.0
                resp_sid = 0x7E
            elif SID == 0x19:
                resp_sid = 0x59
            elif SID == 0x14:
                dtc = [0]*7; dtc_ctr = [0]*7
                resp_sid = 0x54
            elif SID == 0x27:
                sub = payload[0]
                if sub == 0x01:
                    seed_v = lfsr32(seed_v)
                    resp_sid = 0x67
                elif sub == 0x02:
                    recv = struct.unpack('<I', bytes(payload[1:5]))[0]
                    exp  = compute_key(seed_v)
                    if recv == exp:
                        unlocked = True; resp_sid = 0x67
                    else:
                        resp_sid = 0x7F  # NRC
            elif SID == 0x2E:
                if unlocked:
                    did = (payload[0] << 8) | payload[1]
                    val = struct.unpack('<f', bytes(payload[2:6]))[0]
                    if did == 0xF200: ttc_w = val
                    elif did == 0xF201: ttc_l1 = val
                    elif did == 0xF202: kp = val
                    resp_sid = 0x6E
                else:
                    resp_sid = 0x7F
            elif SID == 0x31:
                rid = (payload[1] << 8) | payload[2] if len(payload) > 2 else 0
                if rid == 0x0301:
                    aeb_en = payload[3] if len(payload) > 3 else 1
                resp_sid = 0x71

        out_sess[k]    = sess
        out_unlock[k]  = float(unlocked)
        out_dtc_cnt[k] = sum(dtc)
        out_lamp[k]    = float(any(dtc))
        out_aeb_en[k]  = float(aeb_en)
        out_ttc_w[k]   = ttc_w
        out_ttc_l1[k]  = ttc_l1
        out_kp[k]      = kp
        out_resp_sid[k]= resp_sid

    return dict(t=t, sess=out_sess, unlock=out_unlock, dtc_cnt=out_dtc_cnt,
                lamp=out_lamp, aeb_en=out_aeb_en, ttc_w=out_ttc_w,
                ttc_l1=out_ttc_l1, kp=out_kp, resp=out_resp_sid,
                fault_arr=fault_arr, dtc=dtc)


# =============================================================================
# fig 1 -- UDS Architecture
# =============================================================================
print('Gerando fig_uds_architecture.pdf ...')
fig, ax = plt.subplots(figsize=(10, 7))
ax.set_xlim(0, 10); ax.set_ylim(0, 9); ax.axis('off')

layers = [
    (5, 8.2, 8, 0.7, DKBLUE, 'white',
     'Tester / Ferramenta de Diagnóstico (PC, VCI, InfoCenter)'),
    (5, 7.0, 7, 0.7, BLUE,   'white',
     'ISO 15765-2: Transport (TP) — CAN 500 kbps, SF/FF/CF'),
    (5, 5.8, 7, 0.7, CYAN,   'white',
     'ISO 14229-1: UDS Session Layer — Default / Extended / Programming'),
    (5, 4.6, 7, 0.7, PURPLE, 'white',
     'Security Access (SID 0x27) — LFSR-32 Seed + Seed-Key Algorithm'),
    (5, 3.4, 7, 0.7, ORANGE, 'white',
     'Service Router: 0x10 0x11 0x14 0x19 0x22 0x2E 0x31 0x3E'),
    (5, 2.2, 7, 0.7, GREEN,  'white',
     'DTC Manager — Debounce, NVM simulada, Fault Lamp'),
    (5, 1.0, 7, 0.7, '#333333', 'white',
     'AEB Application: fsm_state, ttc, brake_bar, sensor_fault (leitura / calibração)'),
]

for (cx, cy, w, h, fc, tc, label) in layers:
    box = FancyBboxPatch((cx - w/2, cy - h/2), w, h,
                         boxstyle='round,pad=0.06', lw=1.4,
                         edgecolor='#ffffff', facecolor=fc)
    ax.add_patch(box)
    ax.text(cx, cy, label, ha='center', va='center',
            fontsize=8.5, color=tc, fontweight='bold')

# Arrows between layers
for i in range(len(layers)-1):
    y_top = layers[i][1] - layers[i][3]/2
    y_bot = layers[i+1][1] + layers[i+1][3]/2
    ax.annotate('', xy=(5, y_bot + 0.02), xytext=(5, y_top - 0.02),
                arrowprops=dict(arrowstyle='<->', color='#555555', lw=1.5))

# SIDs legend on the right
sids = [
    (0x10, 'DiagnosticSessionControl'),
    (0x11, 'ECUReset'),
    (0x14, 'ClearDiagnosticInformation'),
    (0x19, 'ReadDTCInformation'),
    (0x22, 'ReadDataByIdentifier'),
    (0x27, 'SecurityAccess *'),
    (0x2E, 'WriteDataByIdentifier'),
    (0x31, 'RoutineControl'),
    (0x3E, 'TesterPresent'),
]
ax.text(9.8, 8.5, 'Serviços UDS implementados', ha='right', fontsize=8,
        fontweight='bold', color=DKBLUE)
for i, (sid, name) in enumerate(sids):
    ax.text(9.8, 8.1 - i*0.7, f'0x{sid:02X}  {name}', ha='right',
            fontsize=7.5, color='#333333',
            fontweight='bold' if sid == 0x27 else 'normal')

ax.text(5, 0.3, '* Criptografia: LFSR-32 seed + ROTL32(seed ^ mask, 7) + const',
        ha='center', fontsize=7.5, color=PURPLE, style='italic')

ax.set_title('Arquitetura da Camada de Diagnóstico UDS --- ISO 14229 / ISO 15765',
             fontsize=11, pad=10)
fig.tight_layout()
fig.savefig(os.path.join(fig_dir, 'fig_uds_architecture.pdf'))
plt.close(fig)

# =============================================================================
# fig 2 -- Session State Machine
# =============================================================================
print('Gerando fig_session_states.pdf ...')
fig, ax = plt.subplots(figsize=(10, 6))
ax.set_xlim(0, 10); ax.set_ylim(0, 6); ax.axis('off')

states = {
    'DEFAULT':     (2.0, 3.0, BLUE),
    'EXTENDED':    (5.5, 3.0, ORANGE),
    'PROGRAMMING': (9.0, 3.0, RED),
}
for name, (x, y, fc) in states.items():
    box = FancyBboxPatch((x-1.2, y-0.55), 2.4, 1.1,
                         boxstyle='round,pad=0.08', lw=1.5,
                         edgecolor='#333333', facecolor=fc)
    ax.add_patch(box)
    ax.text(x, y, name, ha='center', va='center', fontsize=9,
            color='white', fontweight='bold')

transitions = [
    ('DEFAULT', 'EXTENDED',    '0x10 0x02', 'top'),
    ('EXTENDED', 'DEFAULT',    'S3 timeout\n5 s / ECUReset\n0x11', 'bottom'),
    ('EXTENDED', 'PROGRAMMING','0x10 0x03', 'top'),
    ('PROGRAMMING', 'DEFAULT', 'S3 timeout\n5 s / ECUReset', 'bottom'),
    ('DEFAULT', 'DEFAULT',     '0x10 0x01\n(confirmação)', 'top_self'),
]

# Simple arrows
arcs = [
    ((2.0, 3.55), (5.5, 3.55),  '0x10 0x02', 0.0),
    ((5.5, 2.45), (2.0, 2.45),  'S3 timeout / 0x11', 0.0),
    ((5.5, 3.55), (9.0, 3.55),  '0x10 0x03', 0.0),
    ((9.0, 2.45), (5.5, 2.45),  'S3 timeout / 0x11', 0.0),
]
for (x0,y0),(x1,y1),lbl,_ in arcs:
    ax.annotate('', xy=(x1,y1), xytext=(x0,y0),
                arrowprops=dict(arrowstyle='->', color='#444444', lw=1.3,
                                connectionstyle='arc3,rad=0.0'))
    mx, my = (x0+x1)/2, (y0+y1)/2
    offset = 0.25 if y0 > 3 else -0.3
    ax.text(mx, my + offset, lbl, ha='center', fontsize=7.5,
            color='#333333',
            bbox=dict(boxstyle='round,pad=0.1', fc='white', alpha=0.8, lw=0))

# TesterPresent note
ax.text(5.0, 1.5, 'SID 0x3E TesterPresent: suprime o timer S3 em qualquer sessão',
        ha='center', fontsize=8.5, color=GREEN,
        bbox=dict(boxstyle='round,pad=0.2', fc='#e8ffe8', ec=GREEN, lw=1))

ax.text(5.0, 0.8,
        'Security Access (0x27) disponível apenas em EXTENDED / PROGRAMMING',
        ha='center', fontsize=8.5, color=RED, style='italic')

ax.set_title('Máquina de Estados de Sessão UDS --- ISO 14229-1', fontsize=11, pad=10)
fig.tight_layout()
fig.savefig(os.path.join(fig_dir, 'fig_session_states.pdf'))
plt.close(fig)

# =============================================================================
# fig 3 -- Security Access flow (SID 0x27)
# =============================================================================
print('Gerando fig_security_access.pdf ...')

seed_init = 0xABCD1234
seed = lfsr32(seed_init)
key  = compute_key(seed)

fig, ax = plt.subplots(figsize=(10, 8))
ax.set_xlim(0, 10); ax.set_ylim(0, 10); ax.axis('off')

# Timeline columns
TESTER_X = 2.0
ECU_X    = 8.0

ax.text(TESTER_X, 9.5, 'TESTER', ha='center', fontsize=11,
        fontweight='bold',
        bbox=dict(boxstyle='round', fc=BLUE, ec=DKBLUE, lw=1.5, pad=0.4),
        color='white')
ax.text(ECU_X, 9.5, 'ECU AEB', ha='center', fontsize=11,
        fontweight='bold',
        bbox=dict(boxstyle='round', fc=RED, ec='#800000', lw=1.5, pad=0.4),
        color='white')

# Vertical timeline bars
ax.plot([TESTER_X, TESTER_X], [0.5, 9.2], color=BLUE, lw=1.5, ls='--', alpha=0.4)
ax.plot([ECU_X,    ECU_X],    [0.5, 9.2], color=RED,  lw=1.5, ls='--', alpha=0.4)

msgs = [
    # (y, direction, label, sublabel, color)
    (8.5, '→', '0x27 0x01  RequestSeed', 'SID=0x27, subFunc=0x01', BLUE),
    (7.3, '←', '0x67 0x01  [SEED 4 bytes]',
     f'LFSR-32: seed=0x{seed:08X}', GREEN),
    (6.5, None, 'LFSR-32 seed', f'seed = 0x{seed:08X}', GRAY),
    (5.8, None, 'compute_key:', None, GRAY),
    (5.2, None, f'  masked  = seed ^ 0xA55A3CC3 = 0x{seed ^ 0xA55A3CC3:08X}', None, PURPLE),
    (4.6, None, f'  rotated = ROTL32(masked, 7) = 0x{rotl32(seed ^ 0xA55A3CC3, 7):08X}', None, PURPLE),
    (4.0, None, f'  key     = rotated + 0x12345678 = 0x{key:08X}', None, RED),
    (3.2, '→', '0x27 0x02  SendKey', f'key=0x{key:08X}', BLUE),
    (2.2, '←', '0x67 0x02  SecurityAccess Granted', 'Security Level unlocked', GREEN),
    (1.2, '→', '0x2E 0xF200  WriteDID TTC_WARN=3.5s', 'Allowed after unlock', ORANGE),
    (0.5, '←', '0x6E 0xF200  Positive Response', '', GREEN),
]

for item in msgs:
    y_pos = item[0]
    direction = item[1]
    label = item[2]
    sublabel = item[3]
    color = item[4]

    if direction == '→':
        ax.annotate('', xy=(ECU_X-0.3, y_pos), xytext=(TESTER_X+0.3, y_pos),
                    arrowprops=dict(arrowstyle='->', color=color, lw=1.5))
        ax.text((TESTER_X+ECU_X)/2, y_pos+0.18, label,
                ha='center', fontsize=8, fontweight='bold', color=color)
        if sublabel:
            ax.text((TESTER_X+ECU_X)/2, y_pos-0.18, sublabel,
                    ha='center', fontsize=7, color='#555555', style='italic')
    elif direction == '←':
        ax.annotate('', xy=(TESTER_X+0.3, y_pos), xytext=(ECU_X-0.3, y_pos),
                    arrowprops=dict(arrowstyle='->', color=color, lw=1.5))
        ax.text((TESTER_X+ECU_X)/2, y_pos+0.18, label,
                ha='center', fontsize=8, fontweight='bold', color=color)
        if sublabel:
            ax.text((TESTER_X+ECU_X)/2, y_pos-0.18, sublabel,
                    ha='center', fontsize=7, color='#555555', style='italic')
    else:
        ax.text(ECU_X - 1.0, y_pos, label, ha='right', fontsize=7.5, color=color)

ax.set_title('Security Access (SID 0x27) --- Fluxo Seed-Key com LFSR-32',
             fontsize=11, pad=10)
fig.tight_layout()
fig.savefig(os.path.join(fig_dir, 'fig_security_access.pdf'))
plt.close(fig)

# =============================================================================
# fig 4 -- DTC lifecycle
# =============================================================================
print('Gerando fig_dtc_lifecycle.pdf ...')
sim = simulate_uds('dtc_lifecycle')
t = sim['t']

fig, axes = plt.subplots(4, 1, figsize=(10, 9), sharex=True)

# Simula debounce counter para C1001 (para visualização)
timeout_err = np.zeros(len(t))
timeout_err[int(5/DT):int(25/DT)] = 1.0
dtc_ctr = np.zeros(len(t))
dtc_c1001 = np.zeros(len(t))
ctr = 0
for k in range(len(t)):
    if timeout_err[k] > 0:
        ctr = min(ctr + 1, 10)
    else:
        ctr = 0
    dtc_ctr[k] = ctr
    dtc_c1001[k] = float(ctr >= DEB_FAULT)

# DTC clear at t=25s
dtc_c1001_cleared = dtc_c1001.copy()
dtc_c1001_cleared[int(25/DT):] = 0.0

ax = axes[0]
ax.fill_between(t, timeout_err, alpha=0.3, color=RED, label='Falha radar timeout')
ax.step(t, timeout_err, where='post', color=RED, lw=1.5)
ax.set_ylim(-0.1, 1.5); ax.set_ylabel('Falha flag')
ax.legend(loc='upper right'); ax.grid(True)
ax.set_title('Condição de falha injetada (C1001 Radar Timeout)')

ax = axes[1]
ax.plot(t, dtc_ctr, color=ORANGE, lw=1.5, label='Debounce counter')
ax.axhline(DEB_FAULT, ls='--', color=RED, lw=1, label=f'Limiar={DEB_FAULT} ciclos')
ax.set_ylabel('Contador'); ax.legend(loc='upper right'); ax.grid(True)
ax.set_title('Contador de debounce --- DTC confirmado após 3 ciclos consecutivos')

ax = axes[2]
ax.step(t, dtc_c1001_cleared, where='post', color=PURPLE, lw=2, label='DTC C1001')
ax.fill_between(t, dtc_c1001_cleared, alpha=0.25, color=PURPLE, step='post')
ax.axvline(25.0, ls='--', color=GREEN, lw=1.5, label='ClearDTC (0x14) t=25s')
ax.set_ylim(-0.1, 1.3); ax.set_ylabel('DTC C1001')
ax.legend(loc='upper right'); ax.grid(True)
ax.set_title('DTC C1001 --- inactive → pending → confirmed → cleared')

ax = axes[3]
ax.step(t, sim['lamp'], where='post', color=RED, lw=2, label='MIL (fault lamp)')
ax.fill_between(t, sim['lamp'], alpha=0.2, color=RED, step='post')
ax.set_ylim(-0.1, 1.3); ax.set_ylabel('MIL (0/1)')
ax.set_xlabel('Tempo (s)'); ax.legend(loc='upper right'); ax.grid(True)
ax.set_title('MIL --- Malfunction Indicator Lamp')

fig.suptitle('Ciclo de Vida DTC --- C1001 Radar Timeout', fontsize=11)
fig.tight_layout(rect=[0, 0, 1, 0.97])
fig.savefig(os.path.join(fig_dir, 'fig_dtc_lifecycle.pdf'))
plt.close(fig)

# =============================================================================
# fig 5 -- ReadDataByIdentifier timeline
# =============================================================================
print('Gerando fig_did_read.pdf ...')

did_timeline = [
    # (t, DID, name, value_str)
    (2.0,  'F100', 'TTC',        '9.87 s'),
    (3.0,  'F101', 'FSM state',  '0x01 (STANDBY)'),
    (4.0,  'F102', 'brake_bar',  '0.00 bar'),
    (5.0,  'F103', 'fault_cnt',  '0x00'),
    (8.0,  'F100', 'TTC',        '7.42 s'),
    (12.0, 'F103', 'fault_cnt',  '0x01 (C1001)'),
]

fig, ax = plt.subplots(figsize=(11, 5))
ax.set_xlim(-0.5, 14); ax.set_ylim(-0.5, len(did_timeline) + 0.5)
ax.axis('off')

TESTER_Y = len(did_timeline) + 0.2
ECU_Y    = len(did_timeline) + 0.2
ax.text(1.0, TESTER_Y, 'TESTER', ha='center', fontsize=10, fontweight='bold',
        color=DKBLUE)
ax.text(13.0, TESTER_Y, 'ECU AEB', ha='center', fontsize=10, fontweight='bold',
        color=RED)

ax.plot([1.0, 1.0], [-0.3, len(did_timeline)], color=BLUE, lw=1.5, ls='--', alpha=0.4)
ax.plot([13.0, 13.0], [-0.3, len(did_timeline)], color=RED, lw=1.5, ls='--', alpha=0.4)

for i, (t_val, did, name, val) in enumerate(did_timeline):
    y = len(did_timeline) - 1 - i
    # Request arrow
    ax.annotate('', xy=(12.8, y+0.2), xytext=(1.2, y+0.2),
                arrowprops=dict(arrowstyle='->', color=BLUE, lw=1.2))
    ax.text(7.0, y+0.32, f't={t_val:.0f}s  0x22 0x{did}  Read {name}',
            ha='center', fontsize=8, color=DKBLUE)
    # Response arrow
    ax.annotate('', xy=(1.2, y-0.2), xytext=(12.8, y-0.2),
                arrowprops=dict(arrowstyle='->', color=GREEN, lw=1.2))
    ax.text(7.0, y-0.38, f'0x62 0x{did} → {val}',
            ha='center', fontsize=8, color=GREEN, style='italic')

ax.set_title('SID 0x22 ReadDataByIdentifier --- DIDs disponíveis no sistema AEB',
             fontsize=11, pad=10)
fig.tight_layout()
fig.savefig(os.path.join(fig_dir, 'fig_did_read.pdf'))
plt.close(fig)

# =============================================================================
# fig 6 -- Calibration WriteDataByIdentifier
# =============================================================================
print('Gerando fig_calibration_write.pdf ...')
sim_cal = simulate_uds('calibration')
t = sim_cal['t']

fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)

ax = axes[0]
ax.step(t, sim_cal['sess'], where='post', color=PURPLE, lw=2)
ax.fill_between(t, 0, 1, where=(sim_cal['unlock'] > 0),
                alpha=0.2, color=GREEN, step='post', label='Desbloqueado')
ax.set_yticks([1,2,3]); ax.set_yticklabels(['DEFAULT','EXTENDED','PROGRAMMING'])
ax.set_ylim(0.5, 3.5); ax.set_ylabel('Sessão')
ax.legend(loc='upper right'); ax.grid(True)
ax.set_title('Sessão UDS e janela de Security Access unlocked')

ax = axes[1]
ax.plot(t, sim_cal['ttc_w'], color=BLUE, lw=2, label='TTC Warning (F200)')
ax.plot(t, sim_cal['ttc_l1'], color=ORANGE, lw=2, label='TTC L1 (F201)')
ax.axhline(4.0, ls='--', color=BLUE, lw=0.8, alpha=0.5)
ax.axhline(3.0, ls='--', color=ORANGE, lw=0.8, alpha=0.5)
ax.set_ylim(1.5, 5.5); ax.set_ylabel('TTC (s)')
ax.legend(loc='upper right'); ax.grid(True)
ax.set_title('Limiares TTC calibrados via 0x2E WriteDataByIdentifier')

ax = axes[2]
ax.plot(t, sim_cal['kp'], color=RED, lw=2)
ax.axhline(10.0, ls='--', color=GRAY, lw=0.8)
ax.set_ylim(0, 15); ax.set_ylabel('Kp PID')
ax.set_xlabel('Tempo (s)'); ax.grid(True)
ax.set_title('Ganho Kp calibrado via DID 0xF202')

fig.suptitle('Calibração via WriteDataByIdentifier --- Requer Security Access',
             fontsize=11)
fig.tight_layout(rect=[0, 0, 1, 0.97])
fig.savefig(os.path.join(fig_dir, 'fig_calibration_write.pdf'))
plt.close(fig)

# =============================================================================
# fig 7 -- Key algorithm visualization
# =============================================================================
print('Gerando fig_key_algorithm.pdf ...')

# Show LFSR output distribution and key bit patterns
seeds_seq = [0xABCD1234]
for _ in range(15):
    seeds_seq.append(lfsr32(seeds_seq[-1]))
keys_seq = [compute_key(s) for s in seeds_seq]

fig, axes = plt.subplots(2, 2, figsize=(11, 8))

ax = axes[0, 0]
ax.bar(range(len(seeds_seq)), [s / 2**32 for s in seeds_seq],
       color=BLUE, alpha=0.7, edgecolor='white')
ax.set_xlabel('Iteração LFSR'); ax.set_ylabel('Seed normalizado')
ax.set_title('Sequência LFSR-32 --- distribuição de sementes')
ax.grid(True, alpha=0.4)

ax = axes[0, 1]
ax.bar(range(len(keys_seq)), [k / 2**32 for k in keys_seq],
       color=GREEN, alpha=0.7, edgecolor='white')
ax.set_xlabel('Iteração'); ax.set_ylabel('Chave normalizada')
ax.set_title('Chaves derivadas correspondentes')
ax.grid(True, alpha=0.4)

ax = axes[1, 0]
# Bit pattern of seed vs key for first 4 seeds
n_show = 4
labels = [f'0x{s:08X}' for s in seeds_seq[:n_show]]
bits_s = np.array([[int(b) for b in f'{s:032b}'] for s in seeds_seq[:n_show]])
im = ax.imshow(bits_s, aspect='auto', cmap='Blues', vmin=0, vmax=1)
ax.set_yticks(range(n_show)); ax.set_yticklabels(labels, fontsize=7)
ax.set_xlabel('Bit (31 → 0)'); ax.set_title('Bits das sementes LFSR')

ax = axes[1, 1]
bits_k = np.array([[int(b) for b in f'{k:032b}'] for k in keys_seq[:n_show]])
key_labels = [f'0x{k:08X}' for k in keys_seq[:n_show]]
ax.imshow(bits_k, aspect='auto', cmap='Reds', vmin=0, vmax=1)
ax.set_yticks(range(n_show)); ax.set_yticklabels(key_labels, fontsize=7)
ax.set_xlabel('Bit (31 → 0)')
ax.set_title('Bits das chaves derivadas')

fig.suptitle(
    'Algoritmo Seed-Key: key = ROTL32(seed ^ 0xA55A3CC3, 7) + 0x12345678',
    fontsize=11)
fig.tight_layout(rect=[0, 0, 1, 0.96])
fig.savefig(os.path.join(fig_dir, 'fig_key_algorithm.pdf'))
plt.close(fig)

print('Todas as figuras geradas em:', fig_dir)
