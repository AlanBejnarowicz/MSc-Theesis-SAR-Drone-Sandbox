"""
plot_ekf_validation.py  —  EKF vs GPS validation plots
Usage: python plot_ekf_validation.py ekf_validation.csv
"""

import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.patches import Ellipse
import matplotlib.patheffects as pe

CSV_FILE = sys.argv[1] if len(sys.argv) > 1 else "ekf_validation.csv"
MPL      = 111111.0
MPL_LON  = 64528.0

# ── Load & clean ──────────────────────────────────────────────────
df = pd.read_csv(CSV_FILE)
df = df[df["gps_lat"] != 0].copy().reset_index(drop=True)
df = df.drop_duplicates(subset="timestamp_s").reset_index(drop=True)

t = df["timestamp_s"].values

# ── Convert to metres relative to true track mean ────────────────
ref_lat = df["true_lat"].mean()
ref_lon = df["true_lon"].mean()

def to_m(lat, lon):
    return (lon - ref_lon) * MPL_LON, (lat - ref_lat) * MPL

true_x, true_y = to_m(df["true_lat"], df["true_lon"])
gps_x,  gps_y  = to_m(df["gps_lat"],  df["gps_lon"])
ekf_x,  ekf_y  = to_m(df["ekf_lat"],  df["ekf_lon"])

# ── Errors ────────────────────────────────────────────────────────
gps_ex, gps_ey = gps_x - true_x, gps_y - true_y
ekf_ex, ekf_ey = ekf_x - true_x, ekf_y - true_y

gps_dist = np.sqrt(gps_ex**2 + gps_ey**2)
ekf_dist = np.sqrt(ekf_ex**2 + ekf_ey**2)

def stats(ex, ey):
    d = np.sqrt(ex**2 + ey**2)
    return dict(mx=ex.mean(), my=ey.mean(), sx=ex.std(), sy=ey.std(),
                rmse=np.sqrt((ex**2+ey**2).mean()),
                p50=np.percentile(d,50), p95=np.percentile(d,95))

gs_ = stats(gps_ex, gps_ey)
es_ = stats(ekf_ex, ekf_ey)

# ── Style ─────────────────────────────────────────────────────────
BG      = "#ffffff"
PANEL   = "#f8f9fa"
BORDER  = "#cccccc"
TEXT    = "#1a1a1a"
SUBTEXT = "#555555"

C_TRUE  = "#2d6a4f"   # dark green
C_GPS   = "#c0392b"   # red
C_EKF   = "#1a5276"   # navy blue
C_TIME  = "viridis"

plt.rcParams.update({
    "figure.facecolor":  BG,
    "axes.facecolor":    PANEL,
    "axes.edgecolor":    BORDER,
    "axes.labelcolor":   TEXT,
    "axes.titlecolor":   TEXT,
    "xtick.color":       SUBTEXT,
    "ytick.color":       SUBTEXT,
    "grid.color":        BORDER,
    "grid.linewidth":    0.6,
    "text.color":        TEXT,
    "font.family":       "serif",
    "font.size":         9,
})

fig = plt.figure(figsize=(16, 10), facecolor=BG)
fig.text(0.5, 0.97, "EKF Validation — Half of swarm GPS Spoofed, Tested UAV GPS OK",
         ha="center", va="top", fontsize=14, fontweight="bold",
         color=TEXT, fontfamily="serif")

# ── Layout: 2 cols top, stats bar bottom ──────────────────────────
outer = gridspec.GridSpec(2, 1, figure=fig, height_ratios=[10, 1.6],
                          hspace=0.08, left=0.05, right=0.97,
                          top=0.93, bottom=0.04)
top   = gridspec.GridSpecFromSubplotSpec(1, 2, subplot_spec=outer[0],
                                         wspace=0.32)

ax_traj   = fig.add_subplot(top[0])   # trajectory
ax_scatter = fig.add_subplot(top[1])  # combined error scatter
ax_stats  = fig.add_subplot(outer[1]) # stats bar

# ─── 1. Trajectory ───────────────────────────────────────────────
ax_traj.plot(true_x, true_y, color=C_TRUE, lw=1.8, zorder=3,
             label="True", solid_capstyle="round")
ax_traj.scatter(gps_x, gps_y, c=t, cmap=C_TIME, s=12, alpha=0.55,
                zorder=2, linewidths=0, label="GPS")
ax_traj.scatter(ekf_x, ekf_y, color=C_EKF, s=8, alpha=0.6,
                zorder=4, linewidths=0, label="EKF")

# Start / end markers
for x, y, label in [(true_x.iloc[0], true_y.iloc[0], "START"),
                     (true_x.iloc[-1], true_y.iloc[-1], "END")]:
    ax_traj.scatter(x, y, color=C_TRUE, s=60, zorder=6, marker="D",
                    edgecolors="white", linewidths=0.8)
    ax_traj.text(x+0.3, y+0.3, label, color=C_TRUE, fontsize=7, zorder=7)

ax_traj.set_title("Trajectory", fontsize=11, pad=8)
ax_traj.set_xlabel("East (m)")
ax_traj.set_ylabel("North (m)")
ax_traj.set_aspect("equal", adjustable="datalim")
ax_traj.grid(True, alpha=0.5)
ax_traj.legend(fontsize=8, framealpha=0.15, loc="upper left",
               facecolor=PANEL, edgecolor=BORDER)

# ─── 2. Combined error scatter ───────────────────────────────────
def draw_sigma_ellipses(ax, mx, my, sx, sy, color, sigmas=(1,2,3)):
    alphas = (0.22, 0.12, 0.06)
    for n, a in zip(sigmas, alphas):
        theta = np.linspace(0, 2*np.pi, 300)
        ex_ = mx + n*sx*np.cos(theta)
        ey_ = my + n*sy*np.sin(theta)
        ax.fill(ex_, ey_, color=color, alpha=a, zorder=1)
        ax.plot(ex_, ey_, color=color, lw=0.9, alpha=0.7,
                linestyle="--", zorder=2)
        ax.text(mx + n*sx + 0.05, my, f"{n}σ",
                color=color, fontsize=7, alpha=0.85, va="center")

ax_scatter.scatter(gps_ex, gps_ey, color=C_GPS, s=14, alpha=0.45,
                   linewidths=0, zorder=3, label="GPS error")
ax_scatter.scatter(ekf_ex, ekf_ey, color=C_EKF, s=14, alpha=0.45,
                   linewidths=0, zorder=3, label="EKF error")

draw_sigma_ellipses(ax_scatter, gs_["mx"], gs_["my"],
                    gs_["sx"], gs_["sy"], C_GPS)
draw_sigma_ellipses(ax_scatter, es_["mx"], es_["my"],
                    es_["sx"], es_["sy"], C_EKF)

# Mean crosses
for mx, my, c in [(gs_["mx"], gs_["my"], C_GPS),
                   (es_["mx"], es_["my"], C_EKF)]:
    ax_scatter.scatter(mx, my, color=c, s=80, marker="+",
                       linewidths=2, zorder=6)

ax_scatter.axhline(0, color=BORDER, lw=1.0, zorder=0)
ax_scatter.axvline(0, color=BORDER, lw=1.0, zorder=0)
ax_scatter.set_title("Position Error Scatter", fontsize=11, pad=8)
ax_scatter.set_xlabel("ΔEast (m)")
ax_scatter.set_ylabel("ΔNorth (m)")
ax_scatter.set_aspect("equal", adjustable="datalim")
ax_scatter.grid(True, alpha=0.5)
ax_scatter.legend(fontsize=8, framealpha=0.15,
                  facecolor=PANEL, edgecolor=BORDER)

# ─── 3. Stats bar ────────────────────────────────────────────────
ax_stats.set_facecolor("#f0f0f0")
ax_stats.axis("off")

cols   = ["RMSE (m)", "Median (m)", "95th pct (m)", "Bias E (m)", "Bias N (m)"]
g_vals = [gs_["rmse"], gs_["p50"], gs_["p95"], gs_["mx"], gs_["my"]]
e_vals = [es_["rmse"], es_["p50"], es_["p95"], es_["mx"], es_["my"]]

n_cols = len(cols)
xs = np.linspace(0.05, 1.0, n_cols)

# ax_stats.text(0.01, 0.82, "STATISTICS", color=SUBTEXT,
#               fontsize=8, fontweight="bold", transform=ax_stats.transAxes)

for i, (col, gv, ev) in enumerate(zip(cols, g_vals, e_vals)):
    x = xs[i]
    ax_stats.text(x, 0.82, col, ha="center", color=SUBTEXT,
                  fontsize=8, transform=ax_stats.transAxes)
    ax_stats.text(x, 0.48, f"{gv:+.2f}", ha="center", color=C_GPS,
                  fontsize=10, fontweight="bold",
                  transform=ax_stats.transAxes)
    ax_stats.text(x, 0.12, f"{ev:+.2f}", ha="center", color=C_EKF,
                  fontsize=10, fontweight="bold",
                  transform=ax_stats.transAxes)

# Row labels
ax_stats.text(0.01, 0.48, "GPS", color=C_GPS, fontsize=9,
              fontweight="bold", transform=ax_stats.transAxes, va="center")
ax_stats.text(0.01, 0.12, "EKF", color=C_EKF, fontsize=9,
              fontweight="bold", transform=ax_stats.transAxes, va="center")

# Divider lines
for y in [0.65, 0.3]:
    ax_stats.axhline(y, color=BORDER, lw=0.8, xmin=0.0, xmax=1.0)

# ── Save ──────────────────────────────────────────────────────────
out = CSV_FILE.replace(".csv", "_plot.png")
plt.savefig(out, dpi=150, bbox_inches="tight", facecolor=BG)
print(f"Saved → {out}")