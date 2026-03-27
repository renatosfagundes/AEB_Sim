"""
gerar_figs.py — Figuras para o relatório do Barramento CAN do sistema AEB
Gera 7 figuras PNG para inclusão no relatorio_can.tex.

Requisitos: matplotlib, numpy
  pip install matplotlib numpy
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.gridspec as gridspec
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch
import os

OUT_DIR = os.path.dirname(os.path.abspath(__file__))

def savefig(name):
    path = os.path.join(OUT_DIR, name)
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'  Salvo: {name}')


# ─────────────────────────────────────────────────────────────────────────────
# Fig 1: Arquitetura do barramento CAN
# ─────────────────────────────────────────────────────────────────────────────
def fig_arquitetura():
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.set_xlim(0, 12); ax.set_ylim(0, 5); ax.axis('off')
    ax.set_facecolor('#f8f9fa')

    nodes = {
        'Radar_ECU':      (1.2, 3.8, '#4A90D9'),
        'Vehicle_ECU':    (6.0, 3.8, '#5AAF5A'),
        'AEB_Controller': (10.8,3.8, '#E06040'),
    }

    # Barramento CAN (linha horizontal)
    ax.plot([0.3, 11.7], [2.0, 2.0], 'k-', linewidth=6, solid_capstyle='round')
    ax.plot([0.3, 11.7], [2.0, 2.0], color='#FFD700', linewidth=3, linestyle='--',
            solid_capstyle='round')
    ax.text(6.0, 1.6, 'CAN Bus  500 kbps', ha='center', fontsize=12, fontweight='bold',
            color='#333')

    for name, (x, y, color) in nodes.items():
        rect = FancyBboxPatch((x-1.0, y-0.45), 2.0, 0.9,
                               boxstyle='round,pad=0.1', facecolor=color,
                               edgecolor='white', linewidth=2)
        ax.add_patch(rect)
        ax.text(x, y, name.replace('_','\n'), ha='center', va='center',
                fontsize=9, fontweight='bold', color='white')
        # Linha para o barramento
        ax.plot([x, x], [2.0, y-0.45], 'k-', linewidth=2.5)

    # Mensagens
    msgs = [
        (2.0, 2.5, '0x120 AEB_RadarTarget\n20 ms →', '#4A90D9', 'right'),
        (3.8, 2.5, '0x100 AEB_EgoVehicle\n10 ms →',  '#5AAF5A', 'right'),
        (8.2, 2.5, '← 0x080 AEB_BrakeCmd\n10 ms  ASIL-B', '#E06040', 'left'),
        (8.2, 2.0, '← 0x200 AEB_FSMState\n50 ms', '#B06040', 'left'),
        (9.5, 2.5, '← 0x300 AEB_Alert\nevento', '#904090', 'left'),
    ]
    for (x, y, txt, col, ha) in msgs:
        ax.text(x, y, txt, ha=ha, va='bottom', fontsize=7.5, color=col,
                bbox=dict(boxstyle='round,pad=0.2', facecolor='white', edgecolor=col,
                          alpha=0.85))

    ax.set_title('Arquitetura do Barramento CAN — Sistema AEB\n3 nós, 5 mensagens, 500 kbps',
                 fontsize=13, fontweight='bold')
    savefig('fig_arquitetura_can.png')


# ─────────────────────────────────────────────────────────────────────────────
# Fig 2: Temporização das mensagens
# ─────────────────────────────────────────────────────────────────────────────
def fig_timing():
    fig, ax = plt.subplots(figsize=(11, 5))
    t_max = 120  # ms

    messages = [
        ('AEB_BrakeCmd  0x080',  10, '#E06040'),
        ('AEB_EgoVehicle 0x100', 10, '#5AAF5A'),
        ('AEB_RadarTarget 0x120',20, '#4A90D9'),
        ('AEB_FSMState   0x200', 50, '#B06040'),
        ('AEB_Alert 0x300',       0, '#904090'),  # event-driven
    ]

    for i, (name, period, color) in enumerate(messages):
        y = 4 - i
        if period == 0:  # event-driven
            for te in [15, 35, 75]:
                ax.barh(y, 8, left=te, height=0.55, color=color, alpha=0.85,
                        edgecolor='white', linewidth=1.5)
            ax.annotate('evento', xy=(75+4, y), va='center', fontsize=8,
                        color=color)
        else:
            for t0 in range(0, t_max, period):
                ax.barh(y, period*0.75, left=t0, height=0.55, color=color,
                        alpha=0.85, edgecolor='white', linewidth=1.5)
        ax.text(-2, y, name, ha='right', va='center', fontsize=9)

    ax.set_xlabel('Tempo (ms)', fontsize=10)
    ax.set_xlim(-25, t_max+5)
    ax.set_ylim(0.2, 4.8)
    ax.set_yticks([])
    ax.set_title('Temporização das Mensagens CAN — Diagrama de Gantt\n'
                 'BrakeCmd e EgoVehicle: 10ms | RadarTarget: 20ms | FSMState: 50ms | Alert: evento',
                 fontsize=11, fontweight='bold')
    ax.grid(axis='x', linestyle='--', alpha=0.4)
    savefig('fig_timing_can.png')


# ─────────────────────────────────────────────────────────────────────────────
# Fig 3: Codificação DBC — AEB_BrakeCmd
# ─────────────────────────────────────────────────────────────────────────────
def fig_codificacao_dbc():
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    # Esquerda: mapa de bits do frame 0x080 (4 bytes = 32 bits)
    ax = axes[0]
    ax.set_xlim(-0.5, 31.5); ax.set_ylim(-0.5, 1.5); ax.axis('off')
    ax.set_title('Layout de bits — AEB_BrakeCmd (0x080, 4 bytes)', fontsize=10)

    signal_map = {
        'BrakeReq\n(1b)':  ([ 0],       '#E06040'),
        'BrakePressure\n(15b, ×0.1)': (list(range(1,16)), '#4A90D9'),
        'BrakeMode\n(3b)': (list(range(16,19)), '#5AAF5A'),
        'Pad\n(5b)':       (list(range(19,24)), '#CCCCCC'),
        'Alive\n(4b)':     (list(range(24,28)), '#FFB347'),
        'CRC\n(4b)':       (list(range(28,32)), '#904090'),
    }

    for sig_name, (bits, color) in signal_map.items():
        for b in bits:
            ax.add_patch(plt.Rectangle((b-0.45, 0.2), 0.9, 0.7,
                         facecolor=color, edgecolor='white', linewidth=1.5))
            ax.text(b, 0.55, str(b), ha='center', va='center',
                    fontsize=5.5, color='white', fontweight='bold')
        # Rótulo centralizado
        cx = np.mean(bits)
        label_color = '#555555' if color == '#CCCCCC' else color
        ax.text(cx, 1.1, sig_name, ha='center', va='bottom', fontsize=7.5,
                color=label_color)
        ax.annotate('', xy=(min(bits)-0.45, 1.05), xytext=(max(bits)+0.45, 1.05),
                    arrowprops=dict(arrowstyle='<->', color=color, lw=1.5))

    # Rótulos de bytes
    for byte_n, label in enumerate(['Byte 0','Byte 1','Byte 2','Byte 3']):
        ax.text(byte_n*8 + 3.5, -0.3, label, ha='center', fontsize=8, color='#555')
        if byte_n < 3:
            ax.axvline(byte_n*8 + 7.5, color='gray', lw=1.5, linestyle='--')

    # Direita: exemplo numérico de codificação
    ax2 = axes[1]
    ax2.axis('off')
    ax2.set_title('Exemplo de codificação (BRAKE_L2, alive=7)', fontsize=10)

    example = [
        ('Sinal físico',    'Valor físico', 'Fórmula DBC',       'Raw (int)'),
        ('BrakeRequest',    '1 (ativo)',     'raw = physical',     '1'),
        ('BrakePressure',   '4.0 bar',       'raw = 4.0 / 0.1',   '40'),
        ('BrakeMode',       '3 (Moderate_L2)','raw = physical',   '3'),
        ('AliveCounter',    '7',             'rolante mod 16',     '7'),
        ('CRC-4',           '-',             '(7 XOR 1 XOR 3)&0xF','5'),
    ]

    col_colors = ['#4A4A4A', '#5AAF5A', '#4A90D9', '#E06040']
    header = example[0]
    y0 = 0.9
    for j, h in enumerate(header):
        ax2.text(0.05 + j*0.23, y0, h, fontsize=8.5, fontweight='bold',
                 color=col_colors[j % len(col_colors)])
    ax2.axhline(y0 - 0.04, xmin=0.02, xmax=0.98, color='#888', lw=1)

    for i, row in enumerate(example[1:]):
        yrow = y0 - 0.12*(i+1)
        bg = '#F0F4FF' if i % 2 == 0 else 'white'
        ax2.add_patch(plt.Rectangle((0.02, yrow-0.04), 0.96, 0.1,
                      facecolor=bg, edgecolor='none'))
        for j, val in enumerate(row):
            ax2.text(0.05 + j*0.23, yrow, val, fontsize=8,
                     color=col_colors[j % len(col_colors)] if j > 0 else '#333')

    ax2.text(0.5, 0.12,
             'CRC-4 = (alive XOR brake_req XOR brake_mode) AND 0x0F\n'
             '       = (7 XOR 1 XOR 3) & 0xF = 5 & 0xF = 5',
             ha='center', fontsize=8.5, style='italic', color='#444',
             bbox=dict(boxstyle='round,pad=0.4', facecolor='#FFF8E7', edgecolor='#FFB347'))

    plt.suptitle('Codificação DBC — AEB_BrakeCmd 0x080 (ASIL-B)',
                 fontsize=12, fontweight='bold')
    plt.tight_layout()
    savefig('fig_codificacao_dbc.png')


# ─────────────────────────────────────────────────────────────────────────────
# Fig 4: AliveCounter e CRC ao longo do tempo
# ─────────────────────────────────────────────────────────────────────────────
def fig_alive_crc():
    # Simula 20 frames de AEB_BrakeCmd com falha de alive em frames 12-15
    np.random.seed(42)
    n_frames = 20
    alive = np.arange(n_frames) % 16

    # Simula freeze entre frames 12-15
    alive_rx = alive.copy()
    alive_rx[12:16] = alive[11]  # congelado

    # CRC corrompido no frame 17
    def crc4(alive_v, req, mode):
        return int(alive_v ^ req ^ mode) & 0xF

    breq_v = np.zeros(n_frames, dtype=int)
    breq_v[6:] = 1  # frenagem a partir do frame 6
    bmode_v = np.zeros(n_frames, dtype=int)
    bmode_v[6:8] = 1; bmode_v[8:12] = 2; bmode_v[12:] = 3

    crc_tx = np.array([crc4(alive[i], breq_v[i], bmode_v[i]) for i in range(n_frames)])
    crc_rx = crc_tx.copy()
    crc_rx[17] = crc_tx[17] ^ 0xF  # corrompe frame 17

    alive_err = np.zeros(n_frames, dtype=int)
    for i in range(1, n_frames):
        alive_err[i] = int(alive_rx[i] != (alive_rx[i-1] + 1) % 16)

    crc_err = (crc_rx != crc_tx).astype(int)

    frames = np.arange(n_frames)

    fig, axes = plt.subplots(4, 1, figsize=(11, 8), sharex=True)
    fig.suptitle('Mecanismo de AliveCounter e CRC-4 — AEB_BrakeCmd (0x080)',
                 fontsize=12, fontweight='bold')

    axes[0].step(frames, alive,    where='post', color='#4A90D9', lw=2, label='AliveCounter TX')
    axes[0].step(frames, alive_rx, where='post', color='#E06040', lw=2,
                 linestyle='--', label='AliveCounter RX')
    axes[0].fill_between(frames[12:16], 0, 15, alpha=0.15, color='orange', step='post',
                         label='Freeze (frames 12–15)')
    axes[0].set_ylabel('Valor (0–15)'); axes[0].legend(fontsize=8); axes[0].grid(True)
    axes[0].set_title('AliveCounter TX vs RX')

    axes[1].step(frames, alive_err, where='post', color='orange', lw=2)
    axes[1].fill_between(frames, 0, alive_err, alpha=0.4, color='orange', step='post')
    axes[1].set_ylabel('Erro (0/1)'); axes[1].set_ylim(-0.1, 1.4); axes[1].grid(True)
    axes[1].set_title('Erro de AliveCounter detectado no RX (DTC C1005)')

    axes[2].step(frames, crc_tx, where='post', color='#5AAF5A', lw=2, label='CRC TX')
    axes[2].step(frames, crc_rx, where='post', color='#E06040', lw=2,
                 linestyle='--', label='CRC RX')
    axes[2].annotate('Corrupção\n(frame 17)', xy=(17, crc_rx[17]),
                     xytext=(15, 12),
                     arrowprops=dict(arrowstyle='->', color='red'),
                     fontsize=8, color='red')
    axes[2].set_ylabel('Valor CRC (0–15)'); axes[2].legend(fontsize=8); axes[2].grid(True)
    axes[2].set_title('CRC-4 TX vs RX')

    axes[3].step(frames, crc_err, where='post', color='red', lw=2)
    axes[3].fill_between(frames, 0, crc_err, alpha=0.4, color='red', step='post')
    axes[3].set_ylabel('Erro (0/1)'); axes[3].set_ylim(-0.1, 1.4); axes[3].grid(True)
    axes[3].set_title('Erro de CRC detectado no RX (DTC C1004)')
    axes[3].set_xlabel('Número do frame')

    plt.tight_layout()
    savefig('fig_alive_crc.png')


# ─────────────────────────────────────────────────────────────────────────────
# Fig 5: Injeção de falhas e DTCs ao longo do tempo
# ─────────────────────────────────────────────────────────────────────────────
def fig_injecao_falhas():
    t = np.linspace(0, 5, 5000)

    # Dinâmica CCRs 50 km/h
    v0 = 13.89
    v_ego = np.maximum(0, v0 - 4.0 * np.maximum(t - 2.0, 0))
    # Recalcular corretamente
    d_tgt = np.zeros(len(t))
    d_tgt[0] = 50
    dt = t[1] - t[0]
    for i in range(1, len(t)):
        d_tgt[i] = max(0.5, d_tgt[i-1] - v_ego[i-1] * dt)
    ttc_v = np.minimum(20, d_tgt / np.maximum(v_ego, 0.01))

    # FSM
    fsm = np.ones(len(t))
    fsm[ttc_v <= 4.0] = 2; fsm[ttc_v <= 3.0] = 3
    fsm[ttc_v <= 2.2] = 4; fsm[ttc_v <= 1.8] = 5; fsm[v_ego < 0.1] = 1

    # Falhas
    fdrop  = ((t >= 3.5) & (t < 3.6)).astype(float)
    fcrc   = ((t >= 4.0) & (t < 4.05)).astype(float)
    falive = ((t >= 4.5) & (t < 4.7)).astype(float)

    # DTCs (debounce simplificado: dtc ativo enquanto falha ativa + margem)
    kernel = np.ones(200)
    dtc1004 = np.clip(np.convolve(fcrc,   kernel, mode='same'), 0, 1)
    dtc1005 = np.clip(np.convolve(falive, kernel, mode='same'), 0, 1)
    dtc1010 = np.clip(np.convolve(fdrop,  kernel, mode='same'), 0, 1)

    fig, axes = plt.subplots(5, 1, figsize=(11, 9), sharex=True)
    fig.suptitle('Injeção de Falhas e Diagnóstico CAN — Cenário CCRs 50 km/h',
                 fontsize=12, fontweight='bold')

    axes[0].plot(t, ttc_v, color='#4A90D9', lw=2, label='TTC')
    axes[0].axhline(4.0, color='orange', ls='--', lw=1, label='TTC_warn=4.0s')
    axes[0].axhline(1.8, color='red', ls='--', lw=1, label='TTC_L3=1.8s')
    axes[0].set_ylabel('TTC (s)'); axes[0].set_ylim(0, 12); axes[0].legend(fontsize=7)
    axes[0].grid(True)

    axes[1].step(t, fsm, where='post', color='#5AAF5A', lw=2)
    axes[1].set_yticks([1,2,3,4,5]); axes[1].set_yticklabels(['STBY','WARN','L1','L2','L3'])
    axes[1].set_ylabel('FSM'); axes[1].grid(True)

    axes[2].fill_between(t, fdrop,  alpha=0.7, color='#E06040', label='Dropout (100ms)')
    axes[2].fill_between(t, fcrc,   alpha=0.7, color='#FFB347', label='CRC corrupt (50ms)')
    axes[2].fill_between(t, falive, alpha=0.7, color='#904090', label='Alive freeze (200ms)')
    axes[2].set_ylabel('Fault inj.'); axes[2].set_ylim(-0.1, 1.3)
    axes[2].legend(fontsize=7, loc='upper left'); axes[2].grid(True)

    axes[3].fill_between(t, dtc1004, alpha=0.6, color='#FFB347', label='C1004 CRC')
    axes[3].fill_between(t, dtc1005, alpha=0.6, color='#904090', label='C1005 Alive')
    axes[3].fill_between(t, dtc1010, alpha=0.6, color='#E06040', label='C1010 Timeout')
    axes[3].set_ylabel('DTC'); axes[3].set_ylim(-0.1, 1.3)
    axes[3].legend(fontsize=7, loc='upper left'); axes[3].grid(True)

    bpress = np.zeros(len(t))
    bpress[fsm == 3] = 2.0; bpress[fsm == 4] = 4.0; bpress[fsm == 5] = 6.0
    axes[4].plot(t, bpress, color='#E06040', lw=2)
    axes[4].set_ylabel('Pressão (bar)'); axes[4].set_xlabel('Tempo (s)')
    axes[4].grid(True)
    axes[4].set_title('BrakePressure decodificado (AEB_BrakeCmd)')

    plt.tight_layout()
    savefig('fig_injecao_falhas.png')


# ─────────────────────────────────────────────────────────────────────────────
# Fig 6: Pipeline TX → Bus → RX (diagrama de bloco)
# ─────────────────────────────────────────────────────────────────────────────
def fig_pipeline():
    fig, ax = plt.subplots(figsize=(13, 5))
    ax.axis('off'); ax.set_xlim(0, 13); ax.set_ylim(0, 5)

    def box(ax, x, y, w, h, label, color, fontsize=8.5):
        rect = FancyBboxPatch((x, y), w, h, boxstyle='round,pad=0.15',
                               facecolor=color, edgecolor='white', linewidth=2)
        ax.add_patch(rect)
        ax.text(x + w/2, y + h/2, label, ha='center', va='center',
                fontsize=fontsize, fontweight='bold', color='white',
                wrap=True)

    def arrow(ax, x1, y, x2, label=''):
        ax.annotate('', xy=(x2, y), xytext=(x1, y),
                    arrowprops=dict(arrowstyle='->', color='#555', lw=2))
        if label:
            ax.text((x1+x2)/2, y+0.15, label, ha='center', fontsize=7.5, color='#555')

    # Linha AEB_BrakeCmd (ASIL-B)
    box(ax, 0.2, 2.8, 1.8, 0.8, 'Sinais\nfísicos', '#888888')
    arrow(ax, 2.0, 3.2, 2.8, 'physical')
    box(ax, 2.8, 2.8, 2.0, 0.8, 'TX_BrakeCmd\nEncode + Alive\n+ CRC-4', '#E06040')
    arrow(ax, 4.8, 3.2, 5.6, 'raw int')
    box(ax, 5.6, 2.8, 1.8, 0.8, 'Injeção\nde Falhas', '#FFB347')
    arrow(ax, 7.4, 3.2, 8.2, '±corrompido')
    box(ax, 8.2, 2.8, 2.0, 0.8, 'RX_BrakeCmd\nDecode + CRC\ncheck + Alive', '#4A90D9')
    arrow(ax, 10.2, 3.2, 11.0, 'physical')
    box(ax, 11.0, 2.8, 1.8, 0.8, 'Diagnósticos\nDTC C1004\nC1005 C1010', '#904090')

    ax.text(6.5, 3.75, '◄ AEB_BrakeCmd 0x080 — 10ms — ASIL-B ►',
            ha='center', fontsize=9, color='#E06040', fontweight='bold',
            bbox=dict(boxstyle='round,pad=0.3', facecolor='#FFF0F0', edgecolor='#E06040'))

    # Linhas simplificadas para outras mensagens
    other_msgs = [
        (1.5, 'AEB_EgoVehicle 0x100 — 10ms', '#5AAF5A'),
        (0.8, 'AEB_RadarTarget 0x120 — 20ms (+ Alive)', '#4A90D9'),
        (0.2, 'AEB_FSMState 0x200 — 50ms', '#B06040'),
    ]
    for i, (dy, label, color) in enumerate(other_msgs):
        y = 0.3 + i*0.6
        ax.plot([0.2, 12.8], [y, y], '-', color=color, lw=2.5, alpha=0.7)
        ax.text(6.5, y+0.1, label, ha='center', fontsize=8, color=color)

    ax.set_title('Pipeline TX → CAN Bus → RX — AEB_CAN.slx\n'
                 'BrakeCmd (ASIL-B) com AliveCounter + CRC-4 + injeção de falhas',
                 fontsize=11, fontweight='bold')
    savefig('fig_pipeline.png')


# ─────────────────────────────────────────────────────────────────────────────
# Fig 7: Parâmetros DBC
# ─────────────────────────────────────────────────────────────────────────────
def fig_parametros_dbc():
    fig, ax = plt.subplots(figsize=(12, 6))
    ax.axis('off')
    ax.set_title('Parâmetros DBC — aeb_system.dbc (500 kbps)',
                 fontsize=12, fontweight='bold', pad=20)

    columns = ['Mensagem', 'CAN ID', 'Período', 'Bytes', 'Nó TX', 'Sinal chave', 'Factor / Offset']
    rows = [
        ['AEB_BrakeCmd',    '0x080', '10 ms', '4', 'AEB_Controller',
         'BrakePressure [bar]', '0,1 / 0  (15b)'],
        ['AEB_EgoVehicle',  '0x100', '10 ms', '8', 'Vehicle_ECU',
         'VehicleSpeed [m/s]',  '0,01 / 0  (16b)'],
        ['AEB_RadarTarget', '0x120', '20 ms', '8', 'Radar_ECU',
         'TargetDistance [m]',  '0,01 / 0  (16b)'],
        ['AEB_FSMState',    '0x200', '50 ms', '4', 'AEB_Controller',
         'TTCThreshold [s]',    '0,1 / 0  (8b)'],
        ['AEB_Alert',       '0x300', 'evento','2', 'AEB_Controller',
         'BuzzerCmd',           '1 / 0  (3b)'],
    ]

    colors_row = ['#FFE8E0','#E8F5E8','#E0EEFF','#FFF0D8','#F0E0FF']

    table = ax.table(
        cellText=rows, colLabels=columns,
        cellLoc='center', loc='center',
        bbox=[0, 0, 1, 0.95]
    )
    table.auto_set_font_size(False)
    table.set_fontsize(9)

    for j in range(len(columns)):
        table[0, j].set_facecolor('#333333')
        table[0, j].set_text_props(color='white', fontweight='bold')

    for i in range(len(rows)):
        for j in range(len(columns)):
            table[i+1, j].set_facecolor(colors_row[i])

    table.auto_set_column_width(col=list(range(len(columns))))
    savefig('fig_parametros_dbc.png')


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    print('Gerando figuras para relatorio_can.tex ...')

    try:
        from scipy.ndimage import uniform_filter1d
    except ImportError:
        pass  # fig_injecao_falhas tem fallback

    fig_arquitetura()
    fig_timing()
    fig_codificacao_dbc()
    fig_alive_crc()
    fig_injecao_falhas()
    fig_pipeline()
    fig_parametros_dbc()

    print('\nTodas as 7 figuras geradas com sucesso!')
    print(f'Diretório: {OUT_DIR}')
