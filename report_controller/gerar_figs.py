"""
gerar_figs.py
Gera todas as figuras para relatorio_controlador.tex
Figuras produzidas (pasta figs/):
  fig_fsm_estados.pdf       -- Diagrama de estados da FSM (7 estados)
  fig_ttc_calculo.pdf       -- TTC e d_brake vs tempo (CCRs 50 km/h)
  fig_pid_resposta.pdf      -- Resposta do controlador PI (decel + brake_pct)
  fig_cenarios_euronNCAP.pdf -- Comparação CCRs 30/50 e CCRm
  fig_dbake_floor.pdf       -- Efeito do piso de distância d_brake
  fig_alert_sequence.pdf    -- Sequência de alertas por estado
  fig_override.pdf          -- Cenário de override do motorista
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.patches as FancyArrow
from matplotlib.patches import FancyArrowPatch
from matplotlib.patches import Circle, FancyBboxPatch

# ── diretório de saída ────────────────────────────────────────────────────────
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

# =============================================================================
# Parâmetros AEB (aeb_config.h)
# =============================================================================
DT       = 0.001
TTC_W    = 4.0
TTC_L1   = 3.0
TTC_L2   = 2.2
TTC_L3   = 1.8
D_L1     = 20.0
D_L2     = 10.0
D_L3     = 5.0
DECEL_L1 = 2.0
DECEL_L2 = 4.0
DECEL_L3 = 6.0
V_MIN    = 1.39
V_MAX    = 16.67
WARN_MIN = 0.8
HYST     = 0.2
POST     = 2.0
V_REL_MIN = 0.5
KP       = 10.0
KI       = 0.05


def simulate_scenario(v0, v_tgt0, d0, tend=10.0, override_t=None,
                      fault_conf_t=None, aeb_disabled=False):
    """Simula física + FSM + PID para um cenário CCRs/CCRm."""
    t = np.arange(0, tend + DT, DT)
    N = len(t)
    v_ego  = np.zeros(N); v_ego[0] = v0
    v_tgt  = np.full(N, v_tgt0)
    d_tgt  = np.zeros(N); d_tgt[0] = d0
    a_ego  = np.zeros(N)
    fsm    = np.ones(N, dtype=int)
    decel  = np.zeros(N)
    bpct   = np.zeros(N)
    bbar   = np.zeros(N)
    ttc_s  = np.full(N, 10.0)
    dbrk   = np.zeros(N)
    alrt   = np.zeros(N, dtype=int)
    buzz   = np.zeros(N, dtype=int)
    ovr    = np.zeros(N)
    conf   = np.full(N, 15.0)
    warn_t = np.zeros(N)
    st_t   = np.zeros(N)

    st = 1; warn_acc = 0; deb_t_ = 0; pb_t = 0; integ = 0; prev_out = 0
    warn_acc_arr = np.zeros(N)

    DEC = [0, 0, 0, DECEL_L1, DECEL_L2, DECEL_L3, DECEL_L3]
    INTEG_MAX = 50.0; OUT_MAX = 100.0
    max_delta = (100.0 * (100.0 / 10.0)) * DT  # 1.0 %/cycle

    for k in range(1, N):
        tk = t[k]

        # override do motorista
        bp = 1 if (override_t and tk >= override_t[0] and tk < override_t[1]) else 0
        ovr[k] = float(bp != 0)
        if fault_conf_t and tk >= fault_conf_t[0] and tk < fault_conf_t[1]:
            conf[k] = 0
        if aeb_disabled:
            conf[k] = 0  # equivalente a AEB desabilitado

        # TTC
        vr = v_ego[k-1] - v_tgt[k-1]
        if vr > V_REL_MIN and d_tgt[k-1] > 0:
            ttc_s[k] = min(d_tgt[k-1] / vr, 10.0)
        else:
            ttc_s[k] = 10.0
        dbrk[k] = (v_ego[k-1]**2) / (2 * DECEL_L3)

        # ameaça desejada
        in_w = V_MIN <= v_ego[k-1] <= V_MAX
        conf_ok = conf[k] >= 10
        if ovr[k] or not in_w or not conf_ok:
            des = 1
        elif ttc_s[k] <= TTC_L3 or dbrk[k] >= d_tgt[k-1]:
            des = 5
        elif ttc_s[k] <= TTC_L2 or d_tgt[k-1] <= D_L2:
            des = 4
        elif ttc_s[k] <= TTC_L1 or d_tgt[k-1] <= D_L1:
            des = 3
        elif ttc_s[k] <= TTC_W:
            des = 2
        else:
            des = 1

        # FSM
        if st == 1:
            if des >= 2:
                st = 2; warn_acc = 0; deb_t_ = 0
        elif st == 2:
            warn_acc += DT
            if des < 2:
                deb_t_ += DT
                if deb_t_ >= HYST: st = 1; deb_t_ = 0
            else:
                deb_t_ = 0
                if warn_acc >= WARN_MIN and des >= 3:
                    st = des
        elif st in (3, 4, 5):
            if des < st:
                deb_t_ += DT
                if deb_t_ >= HYST:
                    st = 6 if des < 2 else des
                    pb_t = 0; deb_t_ = 0
            else:
                deb_t_ = 0
                if des > st: st = des
            if v_ego[k-1] < V_MIN and st >= 3:
                st = 6; pb_t = 0
        elif st == 6:
            pb_t += DT
            if pb_t >= POST: st = 1

        fsm[k] = st
        warn_acc_arr[k] = warn_acc

        # decel alvo
        dc = DEC[st] if st < len(DEC) else 0
        decel[k] = dc

        # PID
        actual_decel = -a_ego[k-1]
        err = dc - actual_decel
        integ += KI * err * DT
        integ = max(0, min(INTEG_MAX, integ))
        raw = KP * err + integ
        raw = max(0, min(OUT_MAX, raw))
        delta = max(-max_delta, min(max_delta, raw - prev_out))
        bp_out = max(0, min(OUT_MAX, prev_out + delta))
        if st < 3: bp_out = 0; integ = 0  # sem frenagem fora de brake
        prev_out = bp_out
        bpct[k] = bp_out
        bbar[k] = bp_out * 0.1

        # dinâmica
        a_ego[k] = -dc
        v_ego[k] = max(0, v_ego[k-1] + a_ego[k] * DT)
        vr_new = v_ego[k] - v_tgt[k]
        d_tgt[k] = max(0.1, d_tgt[k-1] - vr_new * DT)

        # alertas
        if st == 2:   alrt[k] = 1; buzz[k] = 1
        elif st == 3: alrt[k] = 2; buzz[k] = 2
        elif st == 4: alrt[k] = 3; buzz[k] = 4
        elif st == 5: alrt[k] = 3; buzz[k] = 3
        elif st == 6: alrt[k] = 1; buzz[k] = 0
        else:         alrt[k] = 0; buzz[k] = 0

    return dict(t=t, v_ego=v_ego, v_tgt=v_tgt, d_tgt=d_tgt, a_ego=a_ego,
                fsm=fsm, decel=decel, bpct=bpct, bbar=bbar,
                ttc=ttc_s, dbrk=dbrk, alrt=alrt, buzz=buzz, ovr=ovr,
                warn=warn_acc_arr)


# =============================================================================
# fig 1 — Diagrama de estados FSM
# =============================================================================
print('Gerando fig_fsm_estados.pdf ...')
fig, ax = plt.subplots(figsize=(10, 7))
ax.set_xlim(0, 10); ax.set_ylim(0, 8); ax.axis('off')
ax.set_facecolor('#f8f8f8')

states = {
    'OFF':       (1.0, 7.0, '#cccccc', '#333333'),
    'STANDBY':   (1.0, 5.0, BLUE,     'white'),
    'WARNING':   (5.0, 5.0, ORANGE,   'white'),
    'BRAKE_L1':  (5.0, 3.2, '#f0a030', 'white'),
    'BRAKE_L2':  (5.0, 1.8, RED,      'white'),
    'BRAKE_L3':  (8.5, 3.2, '#990000', 'white'),
    'POST_BRAKE':(2.5, 1.8, PURPLE,   'white'),
}

node_patches = {}
for name, (x, y, fc, tc) in states.items():
    box = FancyBboxPatch((x - 0.85, y - 0.4), 1.7, 0.8,
                         boxstyle='round,pad=0.08', linewidth=1.5,
                         edgecolor='#333333', facecolor=fc)
    ax.add_patch(box)
    ax.text(x, y, name.replace('_', '\n'), ha='center', va='center',
            fontsize=8, color=tc, fontweight='bold', linespacing=1.2)
    node_patches[name] = (x, y)

arrows = [
    ('OFF',       'STANDBY',   'power_on\n(aeb_enable)',     'left', 0),
    ('STANDBY',   'WARNING',   'ttc <= 4s',                   'top', 0),
    ('WARNING',   'STANDBY',   'ttc > 4s\n(0.2s hyst.)',      'bottom', 0),
    ('WARNING',   'BRAKE_L1',  'warn_acc >= 0.8s\nttc <= 3s', 'right', 0.15),
    ('BRAKE_L1',  'BRAKE_L2',  'ttc <= 2.2s\nd <= 10m',       'right', 0.1),
    ('BRAKE_L2',  'BRAKE_L3',  'ttc <= 1.8s\nd_brake >= d',   'right', 0),
    ('BRAKE_L3',  'POST_BRAKE','v < 1.39 m/s\n(veículo parado)', 'left', 0),
    ('POST_BRAKE','STANDBY',   't_hold >= 2s',                 'left', 0),
    ('BRAKE_L1',  'STANDBY',   'override\nou desativado',      'top', -0.1),
    ('STANDBY',   'OFF',       'power_off',                    'left', 0),
]

for src, dst, label, _, _ in arrows:
    x0, y0 = node_patches[src]
    x1, y1 = node_patches[dst]
    ax.annotate('', xy=(x1, y1), xytext=(x0, y0),
                arrowprops=dict(arrowstyle='->', color='#444444',
                                lw=1.2, connectionstyle='arc3,rad=0.1'))
    mx, my = (x0 + x1) / 2, (y0 + y1) / 2
    ax.text(mx, my + 0.15, label, ha='center', va='bottom',
            fontsize=6.5, color='#333333',
            bbox=dict(boxstyle='round,pad=0.1', fc='white', alpha=0.8, lw=0))

ax.set_title('Diagrama de Estados FSM --- AEB Controller (aeb_fsm.c)', fontsize=11, pad=12)
fig.tight_layout()
fig.savefig(os.path.join(fig_dir, 'fig_fsm_estados.pdf'))
plt.close(fig)

# =============================================================================
# fig 2 — TTC e d_brake vs tempo (CCRs 50 km/h)
# =============================================================================
print('Gerando fig_ttc_calculo.pdf ...')
sim = simulate_scenario(13.89, 0, 50)
t = sim['t']

fig, axes = plt.subplots(4, 1, figsize=(9, 8), sharex=True)

ax = axes[0]
ax.plot(t, sim['d_tgt'], color=BLUE, lw=1.5, label='Distância (m)')
ax.axhline(D_L1, ls='--', color=GRAY, lw=1, label=f'D_L1={D_L1}m')
ax.axhline(D_L2, ls='--', color=ORANGE, lw=1, label=f'D_L2={D_L2}m')
ax.axhline(D_L3, ls='--', color=RED, lw=1, label=f'D_L3={D_L3}m')
ax.set_ylabel('d (m)'); ax.legend(loc='upper right', ncol=2); ax.grid(True)
ax.set_title('Distância ao alvo e pisos de distância')

ax = axes[1]
ax.plot(t, sim['ttc'], color=GREEN, lw=1.5, label='TTC (s)')
ax.plot(t, sim['dbrk'], ':', color=ORANGE, lw=1.2, label='d_brake (m)')
ax.axhline(TTC_W,  ls='--', color='black', lw=0.8)
ax.axhline(TTC_L1, ls='--', color=ORANGE, lw=0.8)
ax.axhline(TTC_L2, ls='--', color=RED, lw=0.8)
ax.axhline(TTC_L3, ls='--', color='darkred', lw=0.8)
ax.text(t[-1]*0.97, TTC_W +0.1, '4.0s', ha='right', fontsize=7)
ax.text(t[-1]*0.97, TTC_L1+0.1, '3.0s', ha='right', fontsize=7)
ax.text(t[-1]*0.97, TTC_L2+0.1, '2.2s', ha='right', fontsize=7)
ax.text(t[-1]*0.97, TTC_L3+0.1, '1.8s', ha='right', fontsize=7)
ax.set_ylim(0, 12); ax.set_ylabel('TTC (s) / d_brake (m)')
ax.legend(loc='upper right'); ax.grid(True)
ax.set_title('TTC = d / v_{rel} e d_brake = v_{ego}^2 / (2 x DECEL_L3)')

ax = axes[2]
state_names = {0:'OFF',1:'STANDBY',2:'WARNING',3:'L1',4:'L2',5:'L3',6:'POST'}
c_map = {1:'#3a7abf',2:'#e06000',3:'#f0a030',4:'#cc3322',5:'#990000',6:'#7a30c0'}
prev = sim['fsm'][0]
x0 = t[0]
for k in range(1, len(t)):
    if sim['fsm'][k] != prev or k == len(t)-1:
        ax.axvspan(x0, t[k], alpha=0.35, color=c_map.get(prev,'#cccccc'))
        x0 = t[k]; prev = sim['fsm'][k]
ax.step(t, sim['fsm'], where='post', color=PURPLE, lw=2)
ax.set_yticks(list(state_names.keys()))
ax.set_yticklabels(list(state_names.values()))
ax.set_ylim(-0.5, 6.5); ax.set_ylabel('Estado FSM'); ax.grid(True)
ax.set_title('Estado FSM resultante')

ax = axes[3]
ax.plot(t, sim['v_ego'] * 3.6, color=CYAN, lw=1.5, label='v_ego (km/h)')
ax.axhline(5,  ls='--', color=GRAY, lw=0.8, label='V_MIN=5 km/h')
ax.axhline(60, ls='--', color=RED,  lw=0.8, label='V_MAX=60 km/h')
ax.set_ylabel('v_{ego} (km/h)'); ax.set_xlabel('Tempo (s)')
ax.legend(loc='upper right'); ax.grid(True)
ax.set_title('Velocidade do veículo ego')

fig.suptitle('TTC e d_brake --- Cenário CCRs 50 km/h', fontsize=11)
fig.tight_layout(rect=[0, 0, 1, 0.97])
fig.savefig(os.path.join(fig_dir, 'fig_ttc_calculo.pdf'))
plt.close(fig)

# =============================================================================
# fig 3 — Resposta do controlador PI
# =============================================================================
print('Gerando fig_pid_resposta.pdf ...')

fig, axes = plt.subplots(3, 1, figsize=(9, 7), sharex=True)

ax = axes[0]
ax.plot(t, sim['decel'], color=RED, lw=1.5, label='Decel. alvo (m/s²)')
ax.axhline(DECEL_L1, ls='--', color=GRAY,   lw=1)
ax.axhline(DECEL_L2, ls='--', color=ORANGE, lw=1)
ax.axhline(DECEL_L3, ls='--', color='darkred', lw=1)
ax.text(t[-1]*0.97, DECEL_L1+0.1, 'L1=2', ha='right', fontsize=7)
ax.text(t[-1]*0.97, DECEL_L2+0.1, 'L2=4', ha='right', fontsize=7)
ax.text(t[-1]*0.97, DECEL_L3+0.1, 'L3=6', ha='right', fontsize=7)
ax.set_ylim(-0.5, 8); ax.set_ylabel('Decel. (m/s²)'); ax.grid(True)
ax.set_title('Desaceleração alvo (saída FSM)')

ax = axes[1]
ax.plot(t, sim['bpct'], color=BLUE, lw=1.5, label='brake_pct (%)')
ax.axhline(20, ls='--', color=GRAY,   lw=0.8)
ax.axhline(40, ls='--', color=ORANGE, lw=0.8)
ax.axhline(60, ls='--', color=RED,    lw=0.8)
ax.set_ylim(-2, 105); ax.set_ylabel('brake_pct (%)'); ax.grid(True)
ax.set_title('Saída PID --- limitação de jerk 1.0 %/ciclo (Kp=10, Ki=0.05)')

ax = axes[2]
ax.plot(t, sim['bbar'], color=GREEN, lw=1.5, label='brake_bar (bar)')
ax.set_ylabel('Pressão (bar)'); ax.set_xlabel('Tempo (s)'); ax.grid(True)
ax.set_title('Pressão de freio = brake_pct x 0.1')

fig.suptitle('Resposta do Controlador PI --- CCRs 50 km/h', fontsize=11)
fig.tight_layout(rect=[0, 0, 1, 0.97])
fig.savefig(os.path.join(fig_dir, 'fig_pid_resposta.pdf'))
plt.close(fig)

# =============================================================================
# fig 4 — Comparação Euro NCAP (CCRs 30, CCRs 50, CCRm)
# =============================================================================
print('Gerando fig_cenarios_euroNCAP.pdf ...')
sim30  = simulate_scenario(8.33,  0,    30)
sim50  = simulate_scenario(13.89, 0,    50)
simm   = simulate_scenario(13.89, 5.56, 50)

fig, axes = plt.subplots(3, 1, figsize=(9, 7), sharex=True)

ax = axes[0]
ax.plot(sim30['t'], sim30['d_tgt'],  color=GREEN,  lw=1.5, label='CCRs 30 km/h')
ax.plot(sim50['t'], sim50['d_tgt'],  color=BLUE,   lw=1.5, label='CCRs 50 km/h')
ax.plot(simm['t'],  simm['d_tgt'],   color=ORANGE, lw=1.5, label='CCRm 50/20 km/h')
ax.axhline(D_L3, ls='--', color=RED, lw=0.8, label=f'D_L3={D_L3}m')
ax.set_ylabel('d (m)'); ax.legend(loc='upper right'); ax.grid(True)
ax.set_title('Distância ao alvo --- comparação de cenários Euro NCAP')

ax = axes[1]
ax.step(sim30['t'], sim30['fsm'], where='post', color=GREEN,  lw=1.5, label='CCRs 30')
ax.step(sim50['t'], sim50['fsm'], where='post', color=BLUE,   lw=1.5, label='CCRs 50')
ax.step(simm['t'],  simm['fsm'],  where='post', color=ORANGE, lw=1.5, label='CCRm')
ax.set_yticks([1,2,3,4,5,6])
ax.set_yticklabels(['STBY','WARN','L1','L2','L3','POST'])
ax.set_ylim(0.5, 6.5); ax.set_ylabel('Estado FSM')
ax.legend(loc='lower left'); ax.grid(True)
ax.set_title('Estado FSM por cenário')

ax = axes[2]
ax.plot(sim30['t'], sim30['bbar'],  color=GREEN,  lw=1.5, label='CCRs 30')
ax.plot(sim50['t'], sim50['bbar'],  color=BLUE,   lw=1.5, label='CCRs 50')
ax.plot(simm['t'],  simm['bbar'],   color=ORANGE, lw=1.5, label='CCRm')
ax.set_ylabel('Pressão (bar)'); ax.set_xlabel('Tempo (s)')
ax.legend(loc='upper right'); ax.grid(True)
ax.set_title('Pressão de freio por cenário')

fig.suptitle('Comparação de Cenários Euro NCAP --- CCRs e CCRm', fontsize=11)
fig.tight_layout(rect=[0, 0, 1, 0.97])
fig.savefig(os.path.join(fig_dir, 'fig_cenarios_euroNCAP.pdf'))
plt.close(fig)

# =============================================================================
# fig 5 — Efeito do piso d_brake
# =============================================================================
print('Gerando fig_dbake_floor.pdf ...')
# Simula com velocidade alta para que d_brake >= d dispare L3 antes do TTC
sim_hv = simulate_scenario(16.67, 0, 25)

fig, axes = plt.subplots(3, 1, figsize=(9, 6), sharex=True)

ax = axes[0]
ax.plot(sim_hv['t'], sim_hv['d_tgt'],  color=BLUE,   lw=1.5, label='d_tgt (m)')
ax.plot(sim_hv['t'], sim_hv['dbrk'],   color=RED,    lw=1.5, ls='--', label='d_brake (m)')
ax.fill_between(sim_hv['t'],
                sim_hv['d_tgt'], sim_hv['dbrk'],
                where=(sim_hv['dbrk'] >= sim_hv['d_tgt']),
                alpha=0.25, color=RED, label='d_brake >= d_tgt (L3 ativo)')
ax.axhline(D_L3, ls=':', color=GRAY, lw=0.8)
ax.set_ylabel('Distância (m)'); ax.legend(loc='upper right'); ax.grid(True)
ax.set_title('Piso d_brake: d_brake = v_{ego}^2 / (2 x DECEL_L3) >= d_tgt aciona L3')

ax = axes[1]
ax.plot(sim_hv['t'], sim_hv['ttc'], color=GREEN, lw=1.5)
ax.axhline(TTC_L3, ls='--', color=RED, lw=1, label=f'TTC_L3={TTC_L3}s')
ax.set_ylim(0, 12); ax.set_ylabel('TTC (s)'); ax.legend(); ax.grid(True)
ax.set_title('TTC calculado (piso pode acionar L3 antes do limiar TTC)')

ax = axes[2]
ax.step(sim_hv['t'], sim_hv['fsm'], where='post', color=PURPLE, lw=2)
ax.set_yticks([1,2,3,4,5,6])
ax.set_yticklabels(['STBY','WARN','L1','L2','L3','POST'])
ax.set_ylim(0.5, 6.5); ax.set_ylabel('Estado FSM')
ax.set_xlabel('Tempo (s)'); ax.grid(True)
ax.set_title('Estado FSM (v_ego=60 km/h, d_0=25m)')

fig.suptitle('Efeito do Piso de Distância d_brake --- v_{ego}=60 km/h', fontsize=11)
fig.tight_layout(rect=[0, 0, 1, 0.97])
fig.savefig(os.path.join(fig_dir, 'fig_dbake_floor.pdf'))
plt.close(fig)

# =============================================================================
# fig 6 — Sequência de alertas por estado
# =============================================================================
print('Gerando fig_alert_sequence.pdf ...')

fig, axes = plt.subplots(3, 1, figsize=(9, 6), sharex=True)
t_ref = sim50['t']

ax = axes[0]
ax.step(t_ref, sim50['alrt'], where='post', color=ORANGE, lw=2)
ax.set_yticks([0,1,2,3])
ax.set_yticklabels(['OFF','AVISO','PARCIAL','TOTAL'])
ax.set_ylim(-0.5, 3.5); ax.set_ylabel('Nível'); ax.grid(True)
ax.set_title('Nível de alerta por estado FSM')

buzz_labels = {0:'Off',1:'SingleBeep',2:'DoubleBeep',3:'Continuous',4:'FastPulse'}
ax = axes[1]
ax.step(t_ref, sim50['buzz'], where='post', color=RED, lw=2)
ax.set_yticks([0,1,2,3,4])
ax.set_yticklabels(list(buzz_labels.values()))
ax.set_ylim(-0.5, 4.5); ax.set_ylabel('BuzzerCmd'); ax.grid(True)
ax.set_title('Comando buzzer (DBC 0x300) --- mapeamento aeb_alert.c')

ax = axes[2]
ax.step(t_ref, sim50['fsm'], where='post', color=PURPLE, lw=2)
ax.set_yticks([1,2,3,4,5,6])
ax.set_yticklabels(['STBY','WARN','L1','L2','L3','POST'])
ax.set_ylim(0.5, 6.5); ax.set_ylabel('Estado FSM')
ax.set_xlabel('Tempo (s)'); ax.grid(True)
ax.set_title('Estado FSM (referência)')

fig.suptitle('Sequência de Alertas --- AEB_AlertCmd (0x300)', fontsize=11)
fig.tight_layout(rect=[0, 0, 1, 0.97])
fig.savefig(os.path.join(fig_dir, 'fig_alert_sequence.pdf'))
plt.close(fig)

# =============================================================================
# fig 7 — Cenário de override do motorista
# =============================================================================
print('Gerando fig_override.pdf ...')
sim_ovr = simulate_scenario(13.89, 0, 50, override_t=(3.5, 4.5))

fig, axes = plt.subplots(3, 1, figsize=(9, 6), sharex=True)

ax = axes[0]
ax.step(sim_ovr['t'], sim_ovr['fsm'], where='post', color=PURPLE, lw=2, label='FSM')
ax.fill_between(sim_ovr['t'], 0, 6.5,
                where=(sim_ovr['ovr'] > 0),
                alpha=0.15, color=RED, label='Override ativo')
ax.set_yticks([1,2,3,4,5,6])
ax.set_yticklabels(['STBY','WARN','L1','L2','L3','POST'])
ax.set_ylim(0.5, 6.5); ax.set_ylabel('Estado FSM')
ax.legend(loc='upper right'); ax.grid(True)
ax.set_title('Override do motorista: FSM retorna a STANDBY imediatamente')

ax = axes[1]
ax.plot(sim_ovr['t'], sim_ovr['bbar'], color=BLUE, lw=1.5)
ax.fill_between(sim_ovr['t'], 0, sim_ovr['bbar'].max(),
                where=(sim_ovr['ovr'] > 0),
                alpha=0.15, color=RED, label='Override ativo')
ax.set_ylabel('Pressão (bar)'); ax.legend(); ax.grid(True)
ax.set_title('Pressão de freio: anulada durante override')

ax = axes[2]
ax.step(sim_ovr['t'], sim_ovr['ovr'], where='post', color=RED, lw=2)
ax.set_ylabel('Override (0/1)'); ax.set_xlabel('Tempo (s)')
ax.set_ylim(-0.1, 1.3); ax.grid(True)
ax.set_title('Flag override ativo (pedal de freio ou volante > 5 graus)')

fig.suptitle('Override do Motorista --- driver_override() em aeb_fsm.c', fontsize=11)
fig.tight_layout(rect=[0, 0, 1, 0.97])
fig.savefig(os.path.join(fig_dir, 'fig_override.pdf'))
plt.close(fig)

print('Todas as figuras geradas em:', fig_dir)
