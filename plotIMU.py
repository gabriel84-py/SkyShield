import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import csv
import sys
import os
import random

# ── lecture fichier (argument ou défaut) ──────────────────────
fname = sys.argv[1] if len(sys.argv) > 1 else "imu_data.csv"
if not os.path.exists(fname):
    print(f"Fichier '{fname}' introuvable")
    sys.exit(1)

cols = {
    "time": [], "roll": [], "pitch": [],
    "throttle": [],
    "m1": [], "m2": [], "m3": [], "m4": [],
    "corr_roll": [], "corr_pitch": [],
    "p_roll": [], "i_roll": [], "d_roll": [],
    "p_pitch": [], "i_pitch": [], "d_pitch": [],
}

with open(fname, "r") as f:
    reader = csv.DictReader(f)
    headers = reader.fieldnames
    for row in reader:
        for key in cols:
            if key in headers:
                cols[key].append(float(row[key]))

t = cols["time"]
has_motors  = len(cols["m1"]) > 0
has_pid     = len(cols["p_roll"]) > 0

# ── layout dynamique selon colonnes disponibles ───────────────
n_plots = 2 + (1 if has_motors else 0) + (1 if has_pid else 0)
fig = plt.figure(figsize=(14, 4 * n_plots))
fig.suptitle(f"Analyse vol — {os.path.basename(fname)}", fontsize=13, fontweight='bold')
gs = gridspec.GridSpec(n_plots, 1, hspace=0.45)

plot_idx = 0

# ── 1. Roll / Pitch ───────────────────────────────────────────
ax1 = fig.add_subplot(gs[plot_idx]); plot_idx += 1
ax1.plot(t, cols["roll"],  color='#2196F3', lw=1.5, label='Roll')
ax1.plot(t, cols["pitch"], color='#F44336', lw=1.5, label='Pitch')
ax1.axhline(0, color='k', lw=0.5, ls='--', alpha=0.4)
ax1.set_ylabel('Angle (°)')
ax1.set_title('Roll & Pitch')
ax1.legend(loc='upper right')
ax1.grid(True, alpha=0.25)

# ── 2. Throttle radio ─────────────────────────────────────────
ax2 = fig.add_subplot(gs[plot_idx], sharex=ax1); plot_idx += 1
ax2.plot(t, cols["throttle"], color='#4CAF50', lw=1.5, label='Throttle radio')
ax2.set_ylabel('Throttle (%)')
ax2.set_title('Commande throttle radio')
ax2.set_ylim(-2, 55)
ax2.legend(loc='upper right')
ax2.grid(True, alpha=0.25)

# ── 3. Throttle moteurs individuels ───────────────────────────
if has_motors:
    ax3 = fig.add_subplot(gs[plot_idx], sharex=ax1); plot_idx += 1
    colors_m = {'m1': '#E91E63', 'm2': '#9C27B0', 'm3': '#FF9800', 'm4': '#00BCD4'}
    labels_m = {
        'm1': 'M1 front-right (CW)',
        'm2': 'M2 rear-right (CCW)',
        'm3': 'M3 rear-left (CW)',
        'm4': 'M4 front-left (CCW)',
    }
    for m in ['m1', 'm2', 'm3', 'm4']:
        ax3.plot(t, cols[m], color=colors_m[m], lw=1.2, label=labels_m[m])
    # throttle radio en fond pour comparaison
    ax3.plot(t, cols["throttle"], color='#4CAF50', lw=0.8, ls='--', alpha=0.4, label='Throttle radio (réf)')
    ax3.set_ylabel('Throttle (%)')
    ax3.set_title('Throttle par moteur (après mixage PID)')
    ax3.set_ylim(-2, 105)
    ax3.legend(loc='upper right', fontsize=8, ncol=2)
    ax3.grid(True, alpha=0.25)

# ── 4. Termes PID ─────────────────────────────────────────────
if has_pid:
    ax4 = fig.add_subplot(gs[plot_idx], sharex=ax1); plot_idx += 1
    # roll
    ax4.plot(t, cols["p_roll"],    color='#2196F3', lw=1.0, ls='-',  label='P roll')
    ax4.plot(t, cols["i_roll"],    color='#2196F3', lw=1.0, ls='--', label='I roll', alpha=0.6)
    ax4.plot(t, cols["d_roll"],    color='#2196F3', lw=1.0, ls=':',  label='D roll', alpha=0.6)
    # pitch
    ax4.plot(t, cols["p_pitch"],   color='#F44336', lw=1.0, ls='-',  label='P pitch')
    ax4.plot(t, cols["i_pitch"],   color='#F44336', lw=1.0, ls='--', label='I pitch', alpha=0.6)
    ax4.plot(t, cols["d_pitch"],   color='#F44336', lw=1.0, ls=':',  label='D pitch', alpha=0.6)
    ax4.axhline(0, color='k', lw=0.5, ls='--', alpha=0.4)
    ax4.set_ylabel('Correction')
    ax4.set_title('Termes PID (P/I/D séparés)')
    ax4.legend(loc='upper right', fontsize=8, ncol=2)
    ax4.grid(True, alpha=0.25)

# ── axe X partagé ─────────────────────────────────────────────
fig.add_subplot(gs[-1]).set_xlabel('Temps (s)')
for ax in fig.axes[:-1]:
    plt.setp(ax.get_xticklabels(), visible=False)
plt.setp(fig.axes[-2].get_xticklabels(), visible=True)
fig.axes[-1].set_visible(False)

# ── sauvegarde ────────────────────────────────────────────────
basename = os.path.splitext(os.path.basename(fname))[0]
out = os.path.join(os.path.dirname(fname), basename + f"_plot{random.randint(1,100)}.png")
# si le dossier source est en lecture seule → sauvegarde à côté du script
if not os.access(os.path.dirname(os.path.abspath(out)), os.W_OK):
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), basename + f"_plot{random.randint(1,100)}.png")
plt.savefig(out, dpi=150, bbox_inches='tight')
print(f"Graphique sauvegardé : {out}")
plt.show()