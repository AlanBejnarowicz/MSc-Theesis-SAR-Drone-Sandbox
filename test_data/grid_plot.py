#!/usr/bin/env python3
"""
grid_plot.py  —  Swarm surveillance grid validation plots

Usage:
    python3 grid_plot.py [grid_coverage.csv] [grid_cells.csv]

Produces a 2x2 figure — NO data is trimmed or cropped:
  (a) Coverage % over time          — full mission duration
  (b) Staleness metrics over time   — full mission duration
  (c) Spatial coverage map at mission END
  (d) Spatial coverage map at mission START (first snapshot)
"""

import sys
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.colors import LinearSegmentedColormap
import warnings
warnings.filterwarnings('ignore')

# ── Load — no filtering, no trimming ─────────────────────────────
cov_path   = sys.argv[1] if len(sys.argv) > 1 else "grid_coverage.csv"
cells_path = sys.argv[2] if len(sys.argv) > 2 else "grid_cells.csv"

cov   = pd.read_csv(cov_path)
cells = pd.read_csv(cells_path)

cov["t_min"]   = cov["t_s"] / 60.0
cells["t_min"] = cells["t_s"] / 60.0

# Spatial snapshots: first and last available
unique_snaps = sorted(cells["t_s"].unique())
t_first = unique_snaps[0]
t_last  = unique_snaps[-1]
snap_first = cells[cells["t_s"] == t_first].copy()
snap_last  = cells[cells["t_s"] == t_last].copy()

print(f"Coverage rows  : {len(cov)}")
print(f"Cell rows      : {len(cells)}")
print(f"Snapshots      : {len(unique_snaps)}  "
      f"(t={t_first/60:.1f} – {t_last/60:.1f} min)")
print(f"Mission duration: {cov['t_min'].max():.1f} min")
print(f"Final coverage  : {cov['coverage_pct'].iloc[-1]:.1f}%")
print(f"Final max stale : {cov['max_staleness_s'].iloc[-1]/60:.1f} min")

# ── Style ─────────────────────────────────────────────────────────
plt.rcParams.update({
    "font.family":    "serif",
    "font.size":      10,
    "axes.titlesize": 11,
    "axes.labelsize": 10,
    "legend.fontsize": 9,
    "figure.dpi":     150,
})

cmap_stale = LinearSegmentedColormap.from_list(
    "stale", ["#1B5E20", "#FFEB3B", "#B71C1C"])  # green→yellow→red

MPL = 111320.0; MPLO = 63990.0
lat0 = cells["centre_lat"].mean()
lon0 = cells["centre_lon"].mean()

def to_xy(df):
    d = df.copy()
    d["x"] = (d["centre_lon"] - lon0) * MPLO
    d["y"] = (d["centre_lat"] - lat0) * MPL
    return d

snap_first = to_xy(snap_first)
snap_last  = to_xy(snap_last)

# Shared staleness colour scale across both maps
stale_max = max(
    snap_last["staleness_s"].replace(-1, 0).max(),
    snap_first["staleness_s"].replace(-1, 0).max(),
    1.0
) / 60.0  # in minutes

# ── Figure ────────────────────────────────────────────────────────
fig = plt.figure(figsize=(14, 10))
fig.suptitle("Swarm Surveillance Grid — Coverage Validation",
             fontsize=13, fontweight="bold", y=0.98)
gs = gridspec.GridSpec(2, 2, figure=fig, hspace=0.42, wspace=0.32)

# ══ (a) Coverage % over time — ALL data ═══════════════════════════
ax_a = fig.add_subplot(gs[0, 0])

ax_a.fill_between(cov["t_min"], cov["coverage_pct"],
                  alpha=0.18, color="#2E7D32")
ax_a.plot(cov["t_min"], cov["coverage_pct"],
          color="#2E7D32", lw=2.0, label="Coverage (%)")

ax_a2 = ax_a.twinx()
ax_a2.fill_between(cov["t_min"], cov["drones_active"],
                   alpha=0.10, color="#1565C0")
ax_a2.plot(cov["t_min"], cov["drones_active"],
           color="#1565C0", lw=1.0, ls="--", alpha=0.7,
           label="Active drones")
ax_a2.set_ylabel("Active drones", color="#1565C0", fontsize=9)
ax_a2.tick_params(axis="y", labelcolor="#1565C0", labelsize=8)
ax_a2.set_ylim(0, cov["drones_active"].max() * 1.5)

# Milestone markers — only if milestone actually reached
for target in [25, 50, 75, 100]:
    row = cov[cov["coverage_pct"] >= target]
    if not row.empty:
        t_hit = row["t_min"].iloc[0]
        ax_a.axvline(t_hit, color="grey", lw=0.7, ls=":", alpha=0.5)
        ax_a.text(t_hit + cov["t_min"].max()*0.005, target - 4,
                  f"{target}%\nt={t_hit:.0f} min",
                  fontsize=6.5, color="grey", va="top")

ax_a.set_xlabel("Time (min)")
ax_a.set_ylabel("Coverage (%)")
ax_a.set_title("(a) Zone Coverage Over Time")
ax_a.set_ylim(0, max(cov["coverage_pct"].max() * 1.05, 5))
ax_a.set_xlim(0, cov["t_min"].max())
ax_a.grid(True, alpha=0.3)
lines1, lab1 = ax_a.get_legend_handles_labels()
lines2, lab2 = ax_a2.get_legend_handles_labels()
ax_a.legend(lines1+lines2, lab1+lab2, loc="lower right", fontsize=8)

# ══ (b) Staleness over time — ALL data ════════════════════════════
ax_b = fig.add_subplot(gs[0, 1])

ax_b.fill_between(cov["t_min"],
                  cov["min_staleness_s"] / 60,
                  cov["max_staleness_s"] / 60,
                  alpha=0.13, color="#E65100", label="Min–max range")
ax_b.plot(cov["t_min"], cov["max_staleness_s"] / 60,
          color="#B71C1C", lw=1.8, label="Max staleness")
ax_b.plot(cov["t_min"], cov["mean_staleness_s"] / 60,
          color="#E65100", lw=1.5, ls="--", label="Mean staleness")
ax_b.plot(cov["t_min"], cov["min_staleness_s"] / 60,
          color="#FFA726", lw=1.2, label="Min staleness")

ax_b.set_xlabel("Time (min)")
ax_b.set_ylabel("Time since last visit (min)")
ax_b.set_title("(b) Cell Staleness Over Time")
ax_b.legend(loc="upper left", fontsize=8)
ax_b.set_ylim(bottom=0)
ax_b.set_xlim(0, cov["t_min"].max())
ax_b.grid(True, alpha=0.3)

# ══ (c) Spatial map — FIRST snapshot ══════════════════════════════
ax_c = fig.add_subplot(gs[1, 0])

def draw_map(ax, snap, title):
    visited   = snap[snap["staleness_s"] >= 0]
    unvisited = snap[snap["staleness_s"] <  0]
    cell_sz   = 14

    if not unvisited.empty:
        ax.scatter(unvisited["x"], unvisited["y"], s=cell_sz,
                   color="#F0F0F0", edgecolors="#CCCCCC",
                   linewidths=0.3, zorder=2,
                   label=f"Unvisited ({len(unvisited)})")

    sc = None
    if not visited.empty:
        sc = ax.scatter(visited["x"], visited["y"], s=cell_sz,
                        c=visited["staleness_s"] / 60,
                        cmap=cmap_stale, vmin=0, vmax=stale_max,
                        edgecolors="none", zorder=3,
                        label=f"Visited ({len(visited)})")

    cov_pct = 100 * len(visited) / len(snap) if len(snap) > 0 else 0
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_title(title + f"\n({len(visited)}/{len(snap)} cells, {cov_pct:.1f}%)")
    ax.set_aspect("equal", "datalim")
    ax.legend(loc="best", fontsize=7, markerscale=1.5)
    ax.grid(True, alpha=0.2)
    return sc

sc_c = draw_map(ax_c, snap_first,
                f"(c) Coverage at t={t_first/60:.1f} min (start)")

sc_d_ax = fig.add_subplot(gs[1, 1])
sc_d = draw_map(sc_d_ax, snap_last,
                f"(d) Coverage at t={t_last/60:.1f} min (end)")

# Shared colourbar for (c) and (d)
if sc_d is not None:
    cb = fig.colorbar(sc_d, ax=[ax_c, sc_d_ax],
                      fraction=0.025, pad=0.03, aspect=30)
    cb.set_label("Staleness (min since last visit)", fontsize=9)
    cb.ax.tick_params(labelsize=8)
elif sc_c is not None:
    cb = fig.colorbar(sc_c, ax=[ax_c, sc_d_ax],
                      fraction=0.025, pad=0.03, aspect=30)
    cb.set_label("Staleness (min since last visit)", fontsize=9)

# ── Stats footer — computed from full data ────────────────────────
final = cov.iloc[-1]
stats = (
    f"Drones: {int(final['drones_active'])}   "
    f"Cells: {int(final['cells_total'])}   "
    f"Cell size: 300×300 m   "
    f"Duration: {cov['t_min'].max():.1f} min   "
    f"Final coverage: {final['coverage_pct']:.1f}%   "
    f"Max staleness: {final['max_staleness_s']/60:.1f} min   "
    f"Mean staleness: {final['mean_staleness_s']/60:.1f} min"
)
fig.text(0.5, 0.005, stats, ha="center", va="bottom", fontsize=8.5,
         family="monospace",
         bbox=dict(boxstyle="round,pad=0.4", facecolor="whitesmoke",
                   edgecolor="grey", alpha=0.8))

# ── Save ──────────────────────────────────────────────────────────
out_pdf = cov_path.replace(".csv", "_plot.pdf")
out_png = cov_path.replace(".csv", "_plot.png")
plt.savefig(out_pdf, bbox_inches="tight")
plt.savefig(out_png, bbox_inches="tight", dpi=200)
print(f"Saved → {out_pdf}")
print(f"Saved → {out_png}")
