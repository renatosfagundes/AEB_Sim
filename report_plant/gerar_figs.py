# -*- coding: utf-8 -*-
"""Gera todas as figuras do relatório da planta AEB."""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.patches as FancyPatch
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
import os

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "figs")
os.makedirs(OUT, exist_ok=True)

DPI = 180
AZUL   = "#1E3C64"
AZUL2  = "#3464A0"
CINZA  = "#607D8B"
VERDE  = "#2E7D32"
LARANJA = "#E65100"
FUNDO  = "#F5F8FF"

# ─── helpers ──────────────────────────────────────────────────────────────────
def save(name):
    path = os.path.join(OUT, name)
    plt.savefig(path, dpi=DPI, bbox_inches='tight', facecolor='white')
    plt.close()
    print(f"  saved: {name}")

# ══════════════════════════════════════════════════════════════════════════════
# FIG 1 — Diagrama de blocos do modelo da planta
# ══════════════════════════════════════════════════════════════════════════════
def fig_blocos():
    fig, ax = plt.subplots(figsize=(14, 5))
    ax.set_xlim(0, 14)
    ax.set_ylim(0, 5)
    ax.axis('off')
    ax.set_facecolor(FUNDO)
    fig.patch.set_facecolor(FUNDO)

    def bloco(cx, cy, w, h, titulo, subtitulo="", cor=AZUL2, cor_txt="white"):
        rect = FancyBboxPatch((cx-w/2, cy-h/2), w, h,
                              boxstyle="round,pad=0.08", linewidth=1.5,
                              edgecolor=AZUL, facecolor=cor, zorder=3)
        ax.add_patch(rect)
        ax.text(cx, cy + (0.15 if subtitulo else 0), titulo,
                ha='center', va='center', fontsize=9, fontweight='bold',
                color=cor_txt, zorder=4)
        if subtitulo:
            ax.text(cx, cy - 0.28, subtitulo, ha='center', va='center',
                    fontsize=7, color=cor_txt, alpha=0.85, zorder=4)

    def seta(x0, y0, x1, y1, label="", cor=AZUL):
        ax.annotate("", xy=(x1, y1), xytext=(x0, y0),
                    arrowprops=dict(arrowstyle="-|>", color=cor, lw=1.6), zorder=5)
        if label:
            mx, my = (x0+x1)/2, (y0+y1)/2 + 0.2
            ax.text(mx, my, label, ha='center', va='bottom', fontsize=7.5,
                    color=cor, style='italic')

    # Entrada
    bloco(1.1, 2.5, 1.4, 0.8, "Entrada", "brake_cmd\n[bar]", cor="#263238")
    # Atuador
    bloco(3.5, 2.5, 2.0, 1.2, "Atuador de Freio",
          "dead time 30 ms\n1ª ordem τ=50 ms", cor=AZUL2)
    # Dinâmica ego
    bloco(6.5, 2.5, 2.0, 1.2, "Dinâmica do Ego",
          "a = -brake-drag\n    -roll-grade", cor=AZUL2)
    # Alvo
    bloco(6.5, 0.9, 2.0, 0.8, "Veículo-Alvo",
          "perfil CCRs/CCRm/CCRb", cor=CINZA)
    # Cinemática
    bloco(10.0, 2.5, 2.0, 1.2, "Cinemática\nRelativa",
          "distance, v_rel", cor="#0277BD")
    # Saídas
    bloco(12.8, 3.4, 1.6, 0.7, "v_ego, a_ego", "", cor=VERDE, cor_txt="white")
    bloco(12.8, 2.5, 1.6, 0.7, "distance", "", cor=VERDE, cor_txt="white")
    bloco(12.8, 1.6, 1.6, 0.7, "v_rel", "", cor=VERDE, cor_txt="white")

    # Setas principais
    seta(1.85, 2.5, 2.5, 2.5, "0-10 bar")
    seta(4.5, 2.5, 5.5, 2.5, "decel\n[m/s²]")
    seta(7.5, 2.5, 9.0, 2.5, "v_ego, x_ego")
    seta(7.5, 0.9, 9.0, 0.9)
    # Alvo para cinemaática
    ax.annotate("", xy=(9.0, 1.8), xytext=(7.5, 0.9),
                arrowprops=dict(arrowstyle="-|>", color=CINZA, lw=1.4), zorder=5)
    ax.text(8.4, 1.5, "x_target,\nv_target", ha='center', va='center',
            fontsize=7, color=CINZA, style='italic')

    seta(11.0, 3.1, 12.0, 3.4)
    seta(11.0, 2.5, 12.0, 2.5)
    seta(11.0, 1.9, 12.0, 1.6)

    # Teste interno
    ax.text(1.1, 4.2, "Sinal de teste\n(Step 0→5 bar\nem t=3s)",
            ha='center', va='center', fontsize=8, color=LARANJA,
            bbox=dict(boxstyle='round,pad=0.3', facecolor='#FFF3E0',
                      edgecolor=LARANJA, lw=1.2))
    ax.annotate("", xy=(1.1, 2.9), xytext=(1.1, 3.8),
                arrowprops=dict(arrowstyle="-|>", color=LARANJA, lw=1.4,
                                linestyle='dashed'), zorder=5)
    ax.text(0.55, 3.35, "standalone\n(Manual Switch)", ha='center',
            fontsize=7, color=LARANJA, style='italic')

    ax.set_title("Diagrama de Blocos — Modelo da Planta AEB",
                 fontsize=13, fontweight='bold', color=AZUL, pad=10)
    save("fig_blocos.png")


# ══════════════════════════════════════════════════════════════════════════════
# FIG 2 — Resposta do atuador de freio
# ══════════════════════════════════════════════════════════════════════════════
def fig_atuador():
    t = np.linspace(0, 0.5, 2000)
    dead = 0.030
    tau  = 0.050
    cmd  = 6.0  # BRAKE_L3

    # Resposta: 0 antes do dead time, depois 1ª ordem
    decel = np.where(t < dead, 0.0,
                     cmd * (1 - np.exp(-(t - dead) / tau)))

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    fig.patch.set_facecolor(FUNDO)

    # -- Esquerda: curva de resposta --
    ax = axes[0]
    ax.set_facecolor(FUNDO)
    ax.plot(t * 1000, decel, color=AZUL2, lw=2.5, label='Desaceleração efetiva')
    ax.axhline(cmd, color=CINZA, lw=1.2, ls='--', label=f'Comando: {cmd} m/s²')
    ax.axhline(0.95*cmd, color=VERDE, lw=1, ls=':', alpha=0.7)
    ax.axhline(0.63*cmd, color=LARANJA, lw=1, ls=':', alpha=0.7)

    # Marcadores
    t_63 = (dead + tau) * 1000
    t_95 = (dead + 3*tau) * 1000
    ax.axvline(dead*1000, color='gray', lw=1, ls='--', alpha=0.6)
    ax.axvline(t_63, color=LARANJA, lw=1, ls='--', alpha=0.6)
    ax.axvline(t_95, color=VERDE, lw=1, ls='--', alpha=0.6)

    ax.annotate(f'Dead time\n{dead*1000:.0f} ms', xy=(dead*1000, 0.2),
                xytext=(50, 1.5), fontsize=8, color='gray',
                arrowprops=dict(arrowstyle='->', color='gray', lw=1))
    ax.annotate(f'63% em {t_63:.0f} ms', xy=(t_63, 0.63*cmd),
                xytext=(t_63+30, 0.63*cmd-1.2), fontsize=8, color=LARANJA,
                arrowprops=dict(arrowstyle='->', color=LARANJA, lw=1))
    ax.annotate(f'95% em {t_95:.0f} ms', xy=(t_95, 0.95*cmd),
                xytext=(t_95+20, 0.95*cmd-1.2), fontsize=8, color=VERDE,
                arrowprops=dict(arrowstyle='->', color=VERDE, lw=1))

    ax.set_xlabel('Tempo [ms]', fontsize=10)
    ax.set_ylabel('Desaceleração [m/s²]', fontsize=10)
    ax.set_title('Resposta do Atuador de Freio\n(Comando BRAKE_L3 = 6 m/s²)',
                 fontsize=10, color=AZUL, fontweight='bold')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_xlim(0, 500)
    ax.set_ylim(-0.3, 7.5)

    # -- Direita: diagrama de blocos do atuador --
    ax2 = axes[1]
    ax2.set_xlim(0, 10)
    ax2.set_ylim(0, 4)
    ax2.axis('off')
    ax2.set_facecolor(FUNDO)

    def bk(cx, cy, w, h, txt, cor=AZUL2):
        r = FancyBboxPatch((cx-w/2, cy-h/2), w, h,
                           boxstyle="round,pad=0.1", lw=1.5,
                           edgecolor=AZUL, facecolor=cor)
        ax2.add_patch(r)
        ax2.text(cx, cy, txt, ha='center', va='center',
                 fontsize=9, color='white', fontweight='bold')

    def arr2(x0, y0, x1, y1, lbl=""):
        ax2.annotate("", xy=(x1,y1), xytext=(x0,y0),
                     arrowprops=dict(arrowstyle="-|>", color=AZUL2, lw=1.5))
        if lbl:
            ax2.text((x0+x1)/2, (y0+y1)/2+0.2, lbl, ha='center',
                     fontsize=8, color=AZUL2, style='italic')

    bk(1.2, 2, 1.8, 0.8, "bar → norm\n÷ 10")
    bk(3.5, 2, 2.0, 0.8, "Dead Time\n30 ms")
    bk(6.0, 2, 2.2, 0.8, "TF  1/(τs+1)\nτ = 50 ms")
    bk(8.8, 2, 1.6, 0.8, "× 10\nnorm → m/s²")

    arr2(0.2, 2, 0.3, 2, "")
    ax2.annotate("", xy=(0.3, 2), xytext=(0.1, 2),
                 arrowprops=dict(arrowstyle="-|>", color=AZUL, lw=1.5))
    ax2.text(0.1, 2.5, "brake\ncmd [bar]", fontsize=8, color=AZUL, ha='center')
    arr2(2.1, 2, 2.5, 2, "")
    arr2(4.5, 2, 4.9, 2, "")
    arr2(7.1, 2, 7.9, 2, "")
    ax2.annotate("", xy=(9.9, 2), xytext=(9.6, 2),
                 arrowprops=dict(arrowstyle="-|>", color=AZUL, lw=1.5))
    ax2.text(9.9, 2.5, "decel\n[m/s²]", fontsize=8, color=AZUL, ha='center')

    # Saturation
    bk(8.8, 0.7, 1.6, 0.5, "Sat [0, 10]", cor=CINZA)
    ax2.annotate("", xy=(8.8, 1.0), xytext=(8.8, 1.6),
                 arrowprops=dict(arrowstyle="-|>", color=CINZA, lw=1.2, linestyle='dashed'))

    ax2.set_title('Cadeia do Atuador de Freio (Simulink)',
                  fontsize=10, color=AZUL, fontweight='bold')

    plt.tight_layout()
    save("fig_atuador.png")


# ══════════════════════════════════════════════════════════════════════════════
# FIG 3 — Dinâmica longitudinal (forças)
# ══════════════════════════════════════════════════════════════════════════════
def fig_dinamica():
    v = np.linspace(0, 60/3.6, 300)  # 0-60 km/h em m/s
    m = 1500; Cd = 0.30; A = 2.25; rho = 1.225; Crr = 0.012; g = 9.81

    F_drag  = 0.5 * Cd * A * rho * v**2
    F_roll  = np.full_like(v, Crr * m * g)
    a_drag  = F_drag / m
    a_roll  = F_roll / m
    a_total_passivo = a_drag + a_roll

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    fig.patch.set_facecolor(FUNDO)

    ax = axes[0]
    ax.set_facecolor(FUNDO)
    vkm = v * 3.6
    ax.fill_between(vkm, 0, a_drag, alpha=0.25, color=AZUL2, label='_nolegend_')
    ax.fill_between(vkm, a_drag, a_drag+a_roll, alpha=0.2, color=LARANJA, label='_nolegend_')
    ax.plot(vkm, a_drag, color=AZUL2, lw=2, label=f'Arrasto (Cd={Cd}, A={A} m²)')
    ax.plot(vkm, a_roll * np.ones_like(v), color=LARANJA, lw=2,
            label=f'Rolamento (Crr={Crr})')
    ax.plot(vkm, a_total_passivo, color=AZUL, lw=2.5, ls='--',
            label='Total passivo (sem freio)')

    # Marca 50 km/h
    v50 = 50/3.6
    a50 = 0.5*Cd*A*rho*v50**2/m + Crr*g
    ax.axvline(50, color='gray', lw=1, ls=':', alpha=0.6)
    ax.annotate(f'50 km/h\na_passivo ≈ {a50:.3f} m/s²',
                xy=(50, a50), xytext=(38, a50+0.05),
                fontsize=8, color='gray',
                arrowprops=dict(arrowstyle='->', color='gray', lw=1))

    ax.set_xlabel('Velocidade do Ego [km/h]', fontsize=10)
    ax.set_ylabel('Desaceleração passiva [m/s²]', fontsize=10)
    ax.set_title('Forças Resistivas vs. Velocidade\n(sem frenagem AEB)',
                 fontsize=10, color=AZUL, fontweight='bold')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # -- Direita: diagrama de integração --
    ax2 = axes[1]
    ax2.set_xlim(0, 10)
    ax2.set_ylim(0, 5)
    ax2.axis('off')
    ax2.set_facecolor(FUNDO)

    def bk2(cx, cy, w, h, txt, cor=AZUL2):
        r = FancyBboxPatch((cx-w/2, cy-h/2), w, h,
                           boxstyle="round,pad=0.1", lw=1.5,
                           edgecolor=AZUL, facecolor=cor)
        ax2.add_patch(r)
        for i, line in enumerate(txt.split('\n')):
            n = len(txt.split('\n'))
            ax2.text(cx, cy + (n-1)*0.13 - i*0.26, line,
                     ha='center', va='center', fontsize=8.5,
                     color='white', fontweight='bold')

    def arr3(x0, y0, x1, y1, lbl="", cor=AZUL2):
        ax2.annotate("", xy=(x1,y1), xytext=(x0,y0),
                     arrowprops=dict(arrowstyle="-|>", color=cor, lw=1.4))
        if lbl:
            ax2.text((x0+x1)/2+0.1, (y0+y1)/2+0.2, lbl,
                     fontsize=7.5, color=cor, style='italic', ha='center')

    bk2(2.0, 3.8, 2.2, 0.9, "Σ Forças\n÷ m", cor=AZUL)
    bk2(5.0, 3.8, 2.0, 0.75, "Integrador\na → v", cor=AZUL2)
    bk2(8.0, 3.8, 2.0, 0.75, "Integrador\nv → x", cor=AZUL2)
    bk2(2.0, 1.8, 2.0, 0.75, "v²  (drag)", cor=CINZA)
    bk2(5.0, 1.8, 2.0, 0.75, "× k_drag", cor=CINZA)
    bk2(5.0, 0.6, 2.0, 0.6,  "- Crr·g", cor=CINZA)

    arr3(3.1, 3.8, 4.0, 3.8, "a_ego")
    arr3(6.0, 3.8, 7.0, 3.8, "v_ego")
    ax2.text(8.0, 4.55, "x_ego", ha='center', fontsize=8, color=AZUL2, style='italic')

    # Feedback v -> v²
    arr3(5.0, 3.42, 5.0, 2.18, "", cor=CINZA)
    arr3(4.0, 1.8, 3.0, 1.8, "", cor=CINZA)
    arr3(2.0, 2.17, 2.0, 3.36, "", cor=CINZA)
    arr3(6.0, 1.8, 6.8, 3.5, "", cor=CINZA)
    arr3(5.0, 0.9, 3.5, 3.36, "", cor=CINZA)

    # Entrada do freio
    ax2.text(0.3, 3.8, "-brake\n[m/s²]", ha='center', fontsize=8, color=LARANJA)
    ax2.annotate("", xy=(0.9, 3.8), xytext=(0.6, 3.8),
                 arrowprops=dict(arrowstyle="-|>", color=LARANJA, lw=1.4))

    # Saturação
    bk2(5.0, 3.8+1.2, 1.8, 0.55, "Sat v ≥ 0", cor="#2E7D32")
    ax2.annotate("", xy=(5.0, 4.67), xytext=(5.0, 4.18),
                 arrowprops=dict(arrowstyle="-|>", color=VERDE, lw=1.2, linestyle='dashed'))

    ax2.set_title('Cadeia de Integração (Dinâmica Longitudinal)',
                  fontsize=10, color=AZUL, fontweight='bold')

    plt.tight_layout()
    save("fig_dinamica.png")


# ══════════════════════════════════════════════════════════════════════════════
# FIG 4 — Cenários Euro NCAP (simulação numérica da planta)
# ══════════════════════════════════════════════════════════════════════════════
def simular_planta(v0_kmh, vt_kmh, gap, brake_cmd_bar, decel_tgt=0, t_brake=99,
                   t_max=12, dt=0.001):
    """Simula numericamente a planta AEB."""
    m=1500; Cd=0.30; A=2.25; rho=1.225; Crr=0.012; g=9.81
    dead=0.030; tau=0.050; max_decel=10.0; bar_max=10.0

    v0 = v0_kmh / 3.6
    vt = vt_kmh / 3.6
    t = np.arange(0, t_max, dt)
    v_ego = np.zeros(len(t)); x_ego = np.zeros(len(t))
    v_tgt = np.zeros(len(t)); x_tgt = np.zeros(len(t))
    decel_actual = np.zeros(len(t))
    dist = np.zeros(len(t))

    v_ego[0] = v0; x_ego[0] = 0
    v_tgt[0] = vt; x_tgt[0] = gap
    press_delayed = 0.0; press_tf = 0.0

    for i in range(1, len(t)):
        ti = t[i]
        # Brake actuator
        cmd_norm = brake_cmd_bar / bar_max
        if ti < dead:
            press_delayed = 0.0
        else:
            press_delayed = cmd_norm
        press_tf += dt * (press_delayed - press_tf) / tau
        decel_i = min(max_decel, max(0, press_tf * max_decel))
        decel_actual[i] = decel_i

        # Ego dynamics
        a_drag = 0.5*Cd*A*rho*(v_ego[i-1]**2) / m
        a_roll = Crr * g
        a_ego = -decel_i - a_drag - a_roll
        v_ego[i] = max(0, v_ego[i-1] + a_ego * dt)
        x_ego[i] = x_ego[i-1] + v_ego[i-1] * dt

        # Target
        if ti >= t_brake:
            v_tgt_new = max(0, v_tgt[i-1] + decel_tgt * dt)
        else:
            v_tgt_new = vt
        v_tgt[i] = v_tgt_new
        x_tgt[i] = x_tgt[i-1] + v_tgt[i-1] * dt

        dist[i] = max(0, x_tgt[i] - x_ego[i])

    return t, v_ego*3.6, v_tgt*3.6, dist, decel_actual

def fig_cenarios():
    cenarios = [
        ("CCRs 50 km/h", 50, 0,  100, 6.0, 0,   99),
        ("CCRm 50/20 km/h", 50, 20, 100, 4.0, 0, 99),
        ("CCRb 50 km/h",  50, 50, 40,  6.0, -2,  3),
        ("Sem AEB (CCRs)", 50, 0,  100, 0.0, 0,  99),
    ]
    cores = [AZUL2, VERDE, LARANJA, "#C62828"]

    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    fig.patch.set_facecolor(FUNDO)
    fig.suptitle("Cenários Euro NCAP — Simulação da Planta AEB",
                 fontsize=13, fontweight='bold', color=AZUL, y=0.98)

    for idx, (nome, v0, vt, gap, brake, dtgt, tbk) in enumerate(cenarios):
        t, vego, vtgt, dist, _ = simular_planta(v0, vt, gap, brake, dtgt, tbk)
        ax = axes[idx//2][idx%2]
        ax.set_facecolor(FUNDO)

        ax2 = ax.twinx()
        ax2.plot(t, dist, color=CINZA, lw=1.5, ls='--', alpha=0.7, label='Distância')
        ax2.set_ylabel('Distância [m]', color=CINZA, fontsize=8)
        ax2.tick_params(axis='y', colors=CINZA, labelsize=7)
        ax2.set_ylim(-5, max(dist)*1.15 + 1)

        ax.plot(t, vego, color=cores[idx], lw=2.2, label='v_ego')
        ax.plot(t, vtgt, color='gray', lw=1.8, ls=':', label='v_target')

        # Marca colisão
        col_idx = np.where(dist <= 0.5)[0]
        if len(col_idx) > 0:
            tc = t[col_idx[0]]
            vc = vego[col_idx[0]]
            ax.axvline(tc, color='red', lw=1.2, ls=':', alpha=0.8)
            ax.text(tc+0.1, vc/2, f'Colisão\n{vc:.1f} km/h', fontsize=7.5,
                    color='red', va='center')
        else:
            stop = np.where(vego < 0.5)[0]
            if len(stop) > 0:
                ts = t[stop[0]]
                ax.axvline(ts, color=VERDE, lw=1.2, ls=':', alpha=0.8)
                ax.text(ts+0.1, 5, 'Parado', fontsize=7.5, color=VERDE)

        ax.set_xlabel('Tempo [s]', fontsize=9)
        ax.set_ylabel('Velocidade [km/h]', fontsize=9, color=AZUL)
        ax.set_title(nome, fontsize=10, fontweight='bold', color=AZUL)
        ax.set_xlim(0, max(t))
        ax.set_ylim(-2, max(v0, vt)*1.1 + 2)
        ax.tick_params(axis='y', colors=AZUL, labelsize=7)
        ax.grid(True, alpha=0.25)

        lines1, labels1 = ax.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax.legend(lines1+lines2, labels1+labels2, fontsize=7.5, loc='upper right')

    plt.tight_layout(rect=[0, 0, 1, 0.97])
    save("fig_cenarios.png")


# ══════════════════════════════════════════════════════════════════════════════
# FIG 5 — TTC ao longo do tempo (CCRs 50 km/h)
# ══════════════════════════════════════════════════════════════════════════════
def fig_ttc():
    t, vego, vtgt, dist, decel = simular_planta(50, 0, 100, 6.0, 0, 99)
    v_rel = np.maximum(0.01, (vego - vtgt) / 3.6)  # m/s
    ttc = np.where(v_rel > 0.1, dist / v_rel, 99.0)
    ttc = np.clip(ttc, 0, 10)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    fig.patch.set_facecolor(FUNDO)

    # TTC e limiares
    ax1.set_facecolor(FUNDO)
    ax1.plot(t, ttc, color=AZUL2, lw=2.2, label='TTC')
    ax1.axhline(4.0, color='#FFA000', lw=1.5, ls='--', alpha=0.8, label='WARNING (4,0 s)')
    ax1.axhline(3.0, color=LARANJA, lw=1.5, ls='--', alpha=0.8, label='BRAKE_L1 (3,0 s)')
    ax1.axhline(2.2, color='#D84315', lw=1.5, ls='--', alpha=0.8, label='BRAKE_L2 (2,2 s)')
    ax1.axhline(1.8, color='#B71C1C', lw=1.5, ls='--', alpha=0.8, label='BRAKE_L3 (1,8 s)')

    # Regiões coloridas
    ax1.axhspan(3.0, 4.0, alpha=0.06, color='#FFA000')
    ax1.axhspan(2.2, 3.0, alpha=0.08, color=LARANJA)
    ax1.axhspan(1.8, 2.2, alpha=0.10, color='#D84315')
    ax1.axhspan(0,   1.8, alpha=0.12, color='#B71C1C')

    ax1.set_ylabel('TTC [s]', fontsize=10, color=AZUL)
    ax1.set_title('TTC e Limiares da FSM — Cenário CCRs 50 km/h com BRAKE_L3',
                  fontsize=10, fontweight='bold', color=AZUL)
    ax1.legend(fontsize=8, loc='upper right')
    ax1.grid(True, alpha=0.25)
    ax1.set_ylim(0, 10.5)

    # Velocidade e distância
    ax2.set_facecolor(FUNDO)
    ax2.plot(t, vego, color=AZUL2, lw=2, label='v_ego [km/h]')
    ax2_r = ax2.twinx()
    ax2_r.plot(t, dist, color=CINZA, lw=1.8, ls='--', label='Distância [m]')
    ax2_r.set_ylabel('Distância [m]', color=CINZA, fontsize=9)
    ax2_r.tick_params(axis='y', colors=CINZA)

    ax2.set_xlabel('Tempo [s]', fontsize=10)
    ax2.set_ylabel('Velocidade [km/h]', fontsize=10, color=AZUL)
    ax2.set_title('Resposta do Veículo Ego', fontsize=10, fontweight='bold', color=AZUL)
    ax2.grid(True, alpha=0.25)
    lines1, l1 = ax2.get_legend_handles_labels()
    lines2, l2 = ax2_r.get_legend_handles_labels()
    ax2.legend(lines1+lines2, l1+l2, fontsize=8)

    plt.tight_layout()
    save("fig_ttc.png")


# ══════════════════════════════════════════════════════════════════════════════
# FIG 6 — Parâmetros do veículo (tabela visual)
# ══════════════════════════════════════════════════════════════════════════════
def fig_params():
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    fig.patch.set_facecolor(FUNDO)

    # -- Esquerda: tabela de parâmetros --
    ax = axes[0]
    ax.axis('off')
    ax.set_facecolor(FUNDO)

    dados = [
        ["Parâmetro", "Símbolo", "Valor", "Unidade"],
        ["Massa", "m", "1500", "kg"],
        ["Coef. arrasto", "Cd", "0,30", "–"],
        ["Área frontal", "A", "2,25", "m²"],
        ["Dens. do ar", "ρ", "1,225", "kg/m³"],
        ["Coef. rolamento", "Crr", "0,012", "–"],
        ["Grav.", "g", "9,81", "m/s²"],
        ["Inclinação", "grade", "0", "%"],
        ["Dead time freio", "t_dead", "30", "ms"],
        ["Const. de tempo", "τ", "50", "ms"],
        ["Decel. máx.", "a_max", "10", "m/s²"],
    ]

    tabela = ax.table(cellText=dados[1:], colLabels=dados[0],
                      loc='center', cellLoc='center')
    tabela.auto_set_font_size(False)
    tabela.set_fontsize(9)
    tabela.scale(1, 1.5)

    for (r, c), cell in tabela.get_celld().items():
        if r == 0:
            cell.set_facecolor(AZUL)
            cell.set_text_props(color='white', fontweight='bold')
        elif r % 2 == 0:
            cell.set_facecolor('#E8F0FE')
        else:
            cell.set_facecolor('white')
        cell.set_edgecolor('#BBCFE0')

    ax.set_title('Parâmetros do Veículo e Atuador', fontsize=11,
                 fontweight='bold', color=AZUL, pad=15)

    # -- Direita: força de arrasto vs velocidade --
    ax2 = axes[1]
    ax2.set_facecolor(FUNDO)
    v = np.linspace(0, 60/3.6, 300)
    vkm = v * 3.6
    m=1500; Cd=0.30; A=2.25; rho=1.225; Crr=0.012; g=9.81

    F_drag = 0.5*Cd*A*rho*v**2
    F_roll = Crr*m*g*np.ones_like(v)

    ax2.plot(vkm, F_drag, color=AZUL2, lw=2.2, label='Arrasto (N)')
    ax2.plot(vkm, F_roll, color=LARANJA, lw=2.2, ls='--', label='Rolamento (N)')
    ax2.fill_between(vkm, F_drag, alpha=0.15, color=AZUL2)
    ax2.fill_between(vkm, F_roll, alpha=0.1, color=LARANJA)

    ax2.axvline(50, color='gray', lw=1, ls=':', alpha=0.6)
    F50_drag = 0.5*Cd*A*rho*(50/3.6)**2
    ax2.annotate(f'50 km/h\nF_drag={F50_drag:.0f} N', xy=(50, F50_drag),
                 xytext=(38, F50_drag+30), fontsize=8, color=AZUL2,
                 arrowprops=dict(arrowstyle='->', color=AZUL2, lw=1))

    ax2.set_xlabel('Velocidade [km/h]', fontsize=10)
    ax2.set_ylabel('Força [N]', fontsize=10)
    ax2.set_title('Forças Resistivas (m = 1500 kg)', fontsize=10,
                  fontweight='bold', color=AZUL)
    ax2.legend(fontsize=9)
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    save("fig_params.png")


# ══════════════════════════════════════════════════════════════════════════════
# FIG 7 — Diagrama de fluxo da execução do modelo
# ══════════════════════════════════════════════════════════════════════════════
def fig_fluxo():
    fig, ax = plt.subplots(figsize=(8, 11))
    ax.set_xlim(0, 8)
    ax.set_ylim(0, 11)
    ax.axis('off')
    ax.set_facecolor(FUNDO)
    fig.patch.set_facecolor(FUNDO)

    def caixa(cx, cy, w, h, txt, cor=AZUL2, forma='round'):
        bs = f"{forma},pad=0.1" if forma == 'round' else "square,pad=0.1"
        r = FancyBboxPatch((cx-w/2, cy-h/2), w, h,
                           boxstyle=bs, lw=1.8, edgecolor=AZUL, facecolor=cor)
        ax.add_patch(r)
        for i, ln in enumerate(txt.split('\n')):
            n = len(txt.split('\n'))
            ax.text(cx, cy + (n-1)*0.18 - i*0.36, ln,
                    ha='center', va='center', fontsize=9,
                    color='white', fontweight='bold')

    def losango(cx, cy, w, h, txt):
        pts = np.array([[cx, cy+h/2], [cx+w/2, cy],
                        [cx, cy-h/2], [cx-w/2, cy]])
        patch = plt.Polygon(pts, closed=True, facecolor='#0277BD',
                            edgecolor=AZUL, lw=1.5)
        ax.add_patch(patch)
        ax.text(cx, cy, txt, ha='center', va='center',
                fontsize=8.5, color='white', fontweight='bold')

    def seta_v(x, y0, y1, lbl=""):
        ax.annotate("", xy=(x, y1), xytext=(x, y0),
                    arrowprops=dict(arrowstyle="-|>", color=AZUL2, lw=1.5))
        if lbl:
            ax.text(x+0.12, (y0+y1)/2, lbl, fontsize=8, color=AZUL2, va='center')

    # Passos
    caixa(4, 10.2, 5.5, 0.7, "PASSO 1 — cd modeling/", cor="#263238")
    seta_v(4, 9.85, 9.45)
    caixa(4, 9.1,  5.5, 0.65, "PASSO 2 — run('AEB_Plant_build.m')", cor=AZUL)
    seta_v(4, 8.77, 8.37)
    ax.text(4.15, 8.55, "Cria AEB_Plant.slx + define parâmetros\nno workspace MATLAB",
            fontsize=7.5, color=CINZA, ha='center')
    seta_v(4, 8.2, 7.85)
    losango(4, 7.5, 5.0, 0.6, "Mudar cenário?")
    # Sim
    ax.annotate("", xy=(6.5, 7.5), xytext=(6.5, 7.5),
                arrowprops=dict(arrowstyle="-|>", color=AZUL2, lw=1.5))
    ax.annotate("", xy=(6.5, 6.8), xytext=(6.5, 7.2),
                arrowprops=dict(arrowstyle="-|>", color=AZUL2, lw=1.5))
    ax.plot([4, 6.5, 6.5], [7.2, 7.2, 6.8+0.01], color=AZUL2, lw=1.5)
    ax.text(5.4, 7.3, "Sim", fontsize=8, color=AZUL2)
    caixa(6.5, 6.5, 2.8, 0.55,
          "AEB_Plant_scenarios\n('CCRs_50')", cor=AZUL2)
    ax.plot([6.5, 6.5, 4], [6.22, 5.9, 5.9], color=AZUL2, lw=1.5)
    ax.annotate("", xy=(4, 5.9), xytext=(4.15, 5.9),
                arrowprops=dict(arrowstyle="-|>", color=AZUL2, lw=1.5))
    ax.text(1.5, 7.3, "Não", fontsize=8, color=AZUL2)
    ax.plot([4-2.5, 4-2.5, 4], [7.2, 5.9, 5.9], color=AZUL2, lw=1.5)
    ax.annotate("", xy=(4, 5.9), xytext=(4.15, 5.9),
                arrowprops=dict(arrowstyle="-|>", color=AZUL2, lw=1.5))

    caixa(4, 5.5, 5.5, 0.65, "PASSO 4 — out = sim('AEB_Plant')", cor=AZUL)
    seta_v(4, 5.17, 4.77)
    ax.text(4.15, 4.95, "Roda simulação (t=0 até 15 s)", fontsize=7.5,
            color=CINZA, ha='center')
    seta_v(4, 4.6, 4.2)
    caixa(4, 3.85, 5.5, 0.65, "PASSO 5 — AEB_Plant_plot(out)", cor=AZUL)
    seta_v(4, 3.52, 3.12)
    ax.text(4.15, 3.3, "4 gráficos + resultado no Command Window",
            fontsize=7.5, color=CINZA, ha='center')
    seta_v(4, 2.95, 2.6)
    losango(4, 2.25, 4.5, 0.6, "Outra rodada?")
    ax.plot([4+2.25, 7, 7, 4+2.75], [2.25, 2.25, 7.5, 7.5], color=AZUL2, lw=1.5)
    ax.annotate("", xy=(6.5+0.25, 7.5), xytext=(6.5+0.5, 7.5),
                arrowprops=dict(arrowstyle="-|>", color=AZUL2, lw=1.5))
    ax.text(5.8, 2.4, "Sim", fontsize=8, color=AZUL2)

    seta_v(4, 1.95, 1.55)
    ax.text(1.5, 2.1, "Não", fontsize=8, color=AZUL2)
    caixa(4, 1.2, 4.0, 0.65, "FIM", cor=VERDE)

    ax.set_title("Fluxo de Execução — AEB Plant Model",
                 fontsize=13, fontweight='bold', color=AZUL, pad=10)
    save("fig_fluxo.png")


# ══════════════════════════════════════════════════════════════════════════════
print("Gerando figuras...")
fig_blocos()
fig_atuador()
fig_dinamica()
fig_cenarios()
fig_ttc()
fig_params()
fig_fluxo()
print("Todas as figuras geradas com sucesso.")
