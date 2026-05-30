#!/usr/bin/env python3
"""
coverage_from_positions.py  —  Recompute grid coverage statistics
                                and inter-drone distances from
                                drone_positions.csv + zone JSON.

Usage:
    python3 coverage_from_positions.py drone_positions.csv zone.json
    python3 coverage_from_positions.py drone_positions.csv zone.json --cell 300

What it computes (fully independent from the C++ logger):
    - Rebuilds the 300×300m grid from the JSON zone boundary
    - For each cell, finds the time of every visit from position data
    - Derives: coverage %, min/mean/max staleness over time
    - Minimum inter-drone distance at every timestamp
    - 4-subplot validation figure + summary statistics

Output files:
    coverage_from_pos.csv    — same format as grid_coverage.csv
    mindist_from_pos.csv     — t_s, min_dist_m, mean_dist_m, closest_pair
    coverage_from_pos_plot.pdf / .png
"""

import sys
import json
import math
import argparse
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.patches import Polygon as MplPolygon
from matplotlib.collections import PatchCollection
import warnings
warnings.filterwarnings('ignore')

# ── Constants ─────────────────────────────────────────────────────
MPL     = 111320.0   # metres per degree latitude
ARRIVE  = 80.0       # metres — cell considered visited

# ── Args ──────────────────────────────────────────────────────────
parser = argparse.ArgumentParser()
parser.add_argument('positions', help='drone_positions.csv')
parser.add_argument('zone',      help='zone JSON file')
parser.add_argument('--cell',    type=float, default=300.0,
                    help='Cell size in metres (default 300)')
parser.add_argument('--zone-name', default=None,
                    help='Name of search zone to use (default: first)')
args = parser.parse_args()

CELL_M = args.cell

# ── Load zone JSON ────────────────────────────────────────────────
with open(args.zone) as f:
    zdata = json.load(f)

zones = zdata.get('search_zones', [])
if not zones:
    # Try waypoints as polygon
    wps = zdata.get('waypoints', [])
    boundary = [(w['lat'], w['lon']) for w in wps]
    zone_name = 'waypoints'
else:
    if args.zone_name:
        zone = next((z for z in zones if z['name'] == args.zone_name), zones[0])
    else:
        zone = zones[0]
    boundary  = [(p['lat'], p['lon']) for p in zone['boundary']]
    zone_name = zone.get('name', 'zone')

print(f"Zone      : {zone_name}  ({len(boundary)} vertices)")

# ── Build grid ────────────────────────────────────────────────────
def in_polygon(lat, lon, poly):
    n = len(poly); inside = False; j = n - 1
    for i in range(n):
        yi, xi = poly[i]; yj, xj = poly[j]
        if ((yi > lat) != (yj > lat)) and \
           (lon < (xj - xi) * (lat - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside

lat_arr = [p[0] for p in boundary]; lon_arr = [p[1] for p in boundary]
min_lat, max_lat = min(lat_arr), max(lat_arr)
min_lon, max_lon = min(lon_arr), max(lon_arr)

# metres per degree longitude at zone centre
MPLO = math.cos(math.radians(sum(lat_arr)/len(lat_arr))) * 111320.0

d_lat = CELL_M / MPL
d_lon = CELL_M / MPLO

cells = []   # list of (id, centre_lat, centre_lon)
cid   = 0
lat   = min_lat + d_lat * 0.5
while lat < max_lat:
    lon = min_lon + d_lon * 0.5
    while lon < max_lon:
        if in_polygon(lat, lon, boundary):
            cells.append((cid, lat, lon))
            cid += 1
        lon += d_lon
    lat += d_lat

n_cells = len(cells)
print(f"Grid      : {n_cells} cells ({int(CELL_M)}×{int(CELL_M)} m)")

cell_lat = np.array([c[1] for c in cells])
cell_lon = np.array([c[2] for c in cells])

# ── Load positions ────────────────────────────────────────────────
pos = pd.read_csv(args.positions)
pos = pos.sort_values('t_s').reset_index(drop=True)
drone_ids = sorted(pos['drone_id'].unique())
n_drones  = len(drone_ids)
t_min_arr = pos['t_s'].min()
t_max_arr = pos['t_s'].max()

print(f"Drones    : {n_drones}")
print(f"Duration  : {(t_max_arr - t_min_arr)/60:.1f} min")
print(f"Rows      : {len(pos)}")

# ── Per-cell: find every visit timestamp ──────────────────────────
# For each drone position, check which cell it's in (within ARRIVE metres)
# last_visit[cell_id] = last visit timestamp in seconds

print("Computing cell visits...", flush=True)

# Vectorised: for each row, find nearest cell and check if within ARRIVE
lat_pos = pos['lat'].values
lon_pos = pos['lon'].values
t_pos   = pos['t_s'].values

# last_visit_ms[cell] = most recent visit time (seconds), 0=never
last_visit = np.zeros(n_cells, dtype=float)

# Process in chunks for memory efficiency
CHUNK = 5000
for start in range(0, len(pos), CHUNK):
    end = min(start + CHUNK, len(pos))
    lats  = lat_pos[start:end]
    lons  = lon_pos[start:end]
    times = t_pos[start:end]

    # Distance from each position to each cell (vectorised)
    dy = (lats[:, None] - cell_lat[None, :]) * MPL    # (chunk, n_cells)
    dx = (lons[:, None] - cell_lon[None, :]) * MPLO
    dist2 = dx*dx + dy*dy                              # squared metres

    # For each position, find cells within ARRIVE radius
    mask = dist2 < ARRIVE * ARRIVE                     # bool (chunk, n_cells)
    for i in range(end - start):
        hit_cells = np.where(mask[i])[0]
        for ci in hit_cells:
            if times[i] > last_visit[ci]:
                last_visit[ci] = times[i]

n_visited_final = np.sum(last_visit > 0)
print(f"Final coverage: {n_visited_final}/{n_cells} "
      f"({100*n_visited_final/n_cells:.1f}%)")

# ── Time series: coverage and staleness at each unique timestamp ──
print("Building time series...", flush=True)

unique_t = np.sort(pos['t_s'].unique())
# Subsample to max 500 points for speed
if len(unique_t) > 500:
    idx = np.linspace(0, len(unique_t)-1, 500, dtype=int)
    unique_t = unique_t[idx]

cov_rows = []
# Track visits up to each time t using cumulative approach
# Sort all visit events
visit_events = []  # (time, cell_id)
for ci in range(n_cells):
    if last_visit[ci] > 0:
        visit_events.append((last_visit[ci], ci))
# We need ALL visit times, not just last — re-extract
print("  Re-extracting all visit events...", flush=True)
all_visits = []   # (t_s, cell_id)
for start in range(0, len(pos), CHUNK):
    end   = min(start + CHUNK, len(pos))
    lats  = lat_pos[start:end]
    lons  = lon_pos[start:end]
    times = t_pos[start:end]
    dy    = (lats[:, None] - cell_lat[None, :]) * MPL
    dx    = (lons[:, None] - cell_lon[None, :]) * MPLO
    dist2 = dx*dx + dy*dy
    mask  = dist2 < ARRIVE * ARRIVE
    rows_hit, cols_hit = np.where(mask)
    for r, c in zip(rows_hit, cols_hit):
        all_visits.append((times[r], int(c)))

all_visits.sort()
visit_arr  = np.array(all_visits, dtype=[('t','f8'),('cell','i4')]) if all_visits else np.array([],dtype=[('t','f8'),('cell','i4')])

# For each time snapshot, compute coverage
cell_last = {}   # cell_id → most recent visit time up to current t
vi = 0
for t_snap in unique_t:
    # Advance visit pointer
    while vi < len(visit_arr) and visit_arr[vi]['t'] <= t_snap:
        ci = int(visit_arr[vi]['cell'])
        tv = float(visit_arr[vi]['t'])
        if ci not in cell_last or tv > cell_last[ci]:
            cell_last[ci] = tv
        vi += 1

    visited_now = len(cell_last)
    cov_pct     = 100.0 * visited_now / n_cells

    staleness   = [t_snap - cell_last[c] for c in cell_last]
    min_s  = min(staleness) if staleness else 0.0
    max_s  = max(staleness) if staleness else 0.0
    mean_s = sum(staleness)/len(staleness) if staleness else 0.0

    cov_rows.append({
        't_s'            : t_snap,
        'coverage_pct'   : round(cov_pct, 2),
        'cells_visited'  : visited_now,
        'cells_total'    : n_cells,
        'min_staleness_s': round(min_s, 1),
        'max_staleness_s': round(max_s, 1),
        'mean_staleness_s': round(mean_s, 1),
    })

cov_df = pd.DataFrame(cov_rows)
cov_df.to_csv('coverage_from_pos.csv', index=False)
print(f"Saved coverage_from_pos.csv  ({len(cov_df)} rows)")

# ── Inter-drone distances ─────────────────────────────────────────
print("Computing inter-drone distances...", flush=True)

dist_rows = []
for t_snap in unique_t:
    # Get all drone positions at this timestamp (nearest available)
    snap = pos[np.abs(pos['t_s'] - t_snap) < 1.5]
    # One row per drone — take latest within window
    snap = snap.sort_values('t_s').groupby('drone_id').last().reset_index()

    if len(snap) < 2:
        continue

    lats = snap['lat'].values
    lons = snap['lon'].values
    ids  = snap['drone_id'].values

    min_d = np.inf
    closest = (-1, -1)
    sum_d = 0.0; count = 0

    for i in range(len(snap)):
        for j in range(i+1, len(snap)):
            dy = (lats[i] - lats[j]) * MPL
            dx = (lons[i] - lons[j]) * MPLO
            d  = math.sqrt(dx*dx + dy*dy)
            sum_d += d; count += 1
            if d < min_d:
                min_d = d
                closest = (int(ids[i]), int(ids[j]))

    dist_rows.append({
        't_s'         : t_snap,
        'min_dist_m'  : round(min_d, 1),
        'mean_dist_m' : round(sum_d/count, 1) if count else 0,
        'closest_a'   : closest[0],
        'closest_b'   : closest[1],
    })

dist_df = pd.DataFrame(dist_rows)
dist_df.to_csv('mindist_from_pos.csv', index=False)
print(f"Saved mindist_from_pos.csv  ({len(dist_df)} rows)")
print(f"Min inter-drone distance ever: {dist_df['min_dist_m'].min():.1f} m")
print(f"Mean inter-drone distance    : {dist_df['mean_dist_m'].mean():.1f} m")

# ── Plot ──────────────────────────────────────────────────────────
print("Plotting...", flush=True)

plt.rcParams.update({'font.family':'serif','font.size':10,
    'axes.titlesize':11,'axes.labelsize':10,'legend.fontsize':9,'figure.dpi':150})

cmap_stale = LinearSegmentedColormap.from_list(
    'stale', ['#1B5E20','#FFEB3B','#B71C1C'])

fig = plt.figure(figsize=(14, 10))
fig.suptitle(f'Swarm Coverage Analysis — Recomputed from Positions\n'
             f'Zone: {zone_name}   {n_drones} drones   '
             f'{int(CELL_M)}×{int(CELL_M)} m grid   '
             f'{(t_max_arr-t_min_arr)/60:.0f} min mission',
             fontsize=12, fontweight='bold', y=0.98)
gs = gridspec.GridSpec(2, 2, figure=fig, hspace=0.42, wspace=0.32)

cov_df['t_min'] = cov_df['t_s'] / 60.0
dist_df['t_min'] = dist_df['t_s'] / 60.0

# ══ (a) Coverage over time ════════════════════════════════════════
ax_a = fig.add_subplot(gs[0, 0])
ax_a.fill_between(cov_df['t_min'], cov_df['coverage_pct'],
                  alpha=0.18, color='#2E7D32')
ax_a.plot(cov_df['t_min'], cov_df['coverage_pct'],
          color='#2E7D32', lw=2.0, label='Coverage (%)')

for target in [25, 50, 75, 100]:
    row = cov_df[cov_df['coverage_pct'] >= target]
    if not row.empty:
        t_hit = row['t_min'].iloc[0]
        ax_a.axvline(t_hit, color='grey', lw=0.7, ls=':', alpha=0.5)
        ax_a.text(t_hit + cov_df['t_min'].max()*0.005, target - 5,
                  f'{target}%\n{t_hit:.0f} min',
                  fontsize=6.5, color='grey', va='top')

ax_a.set_xlabel('Time (min)'); ax_a.set_ylabel('Coverage (%)')
ax_a.set_title('(a) Zone Coverage Over Time')
ax_a.set_ylim(0, max(cov_df['coverage_pct'].max()*1.05, 5))
ax_a.set_xlim(0, cov_df['t_min'].max())
ax_a.grid(True, alpha=0.3); ax_a.legend(loc='lower right')

# ══ (b) Staleness over time ═══════════════════════════════════════
ax_b = fig.add_subplot(gs[0, 1])
ax_b.fill_between(cov_df['t_min'],
                  cov_df['min_staleness_s']/60,
                  cov_df['max_staleness_s']/60,
                  alpha=0.13, color='#E65100', label='Min–max range')
ax_b.plot(cov_df['t_min'], cov_df['max_staleness_s']/60,
          color='#B71C1C', lw=1.8, label='Max staleness')
ax_b.plot(cov_df['t_min'], cov_df['mean_staleness_s']/60,
          color='#E65100', lw=1.5, ls='--', label='Mean staleness')
ax_b.plot(cov_df['t_min'], cov_df['min_staleness_s']/60,
          color='#FFA726', lw=1.2, label='Min staleness')
ax_b.set_xlabel('Time (min)'); ax_b.set_ylabel('Time since last visit (min)')
ax_b.set_title('(b) Cell Staleness Over Time')
ax_b.legend(loc='upper left', fontsize=8)
ax_b.set_ylim(bottom=0); ax_b.set_xlim(0, cov_df['t_min'].max())
ax_b.grid(True, alpha=0.3)

# ══ (c) Inter-drone distance over time ════════════════════════════
ax_c = fig.add_subplot(gs[1, 0])

ax_c.fill_between(dist_df['t_min'], dist_df['min_dist_m'],
                  dist_df['mean_dist_m'],
                  alpha=0.15, color='#1565C0', label='Min–mean range')
ax_c.plot(dist_df['t_min'], dist_df['min_dist_m'],
          color='#B71C1C', lw=1.8, label='Min distance')
ax_c.plot(dist_df['t_min'], dist_df['mean_dist_m'],
          color='#1565C0', lw=1.5, ls='--', label='Mean distance')

# Collision avoidance threshold line
ax_c.axhline(30, color='red', lw=1.0, ls=':', alpha=0.7,
             label='Danger zone (30 m)')
ax_c.axhline(150, color='orange', lw=1.0, ls=':', alpha=0.7,
             label='Alert zone (150 m)')

# Annotate minimum ever
min_ever = dist_df['min_dist_m'].min()
t_min_ever = dist_df.loc[dist_df['min_dist_m'].idxmin(), 't_min']
ax_c.annotate(f'Min = {min_ever:.0f} m',
              xy=(t_min_ever, min_ever),
              xytext=(t_min_ever + dist_df['t_min'].max()*0.03,
                      min_ever + dist_df['min_dist_m'].max()*0.05),
              fontsize=8, color='#B71C1C',
              arrowprops=dict(arrowstyle='->', color='#B71C1C', lw=1.0))

ax_c.set_xlabel('Time (min)'); ax_c.set_ylabel('Inter-drone distance (m)')
ax_c.set_title('(c) Inter-Drone Distances Over Time')
ax_c.legend(loc='upper right', fontsize=8)
ax_c.set_ylim(bottom=0); ax_c.set_xlim(0, dist_df['t_min'].max())
ax_c.grid(True, alpha=0.3)

# ══ (d) Final spatial coverage map ════════════════════════════════
ax_d = fig.add_subplot(gs[1, 1])

lat0 = sum(lat_arr)/len(lat_arr)
lon0 = sum(lon_arr)/len(lon_arr)

def to_xy(lat, lon):
    return (lon - lon0)*MPLO, (lat - lat0)*MPL

# Draw zone boundary
bx = [(p[1]-lon0)*MPLO for p in boundary]
by = [(p[0]-lat0)*MPL  for p in boundary]
bx.append(bx[0]); by.append(by[0])
ax_d.plot(bx, by, 'k-', lw=1.5, alpha=0.6, label='Zone boundary')

# Draw cells coloured by final staleness
t_final = t_max_arr
visited_cells   = [(i, last_visit[i]) for i in range(n_cells) if last_visit[i] > 0]
unvisited_cells = [i for i in range(n_cells) if last_visit[i] == 0]

if unvisited_cells:
    ux = [(cells[i][2]-lon0)*MPLO for i in unvisited_cells]
    uy = [(cells[i][1]-lat0)*MPL  for i in unvisited_cells]
    ax_d.scatter(ux, uy, s=10, color='#F0F0F0',
                 edgecolors='#CCCCCC', linewidths=0.3, zorder=2,
                 label=f'Unvisited ({len(unvisited_cells)})')

if visited_cells:
    vx  = [(cells[i][2]-lon0)*MPLO for i,_ in visited_cells]
    vy  = [(cells[i][1]-lat0)*MPL  for i,_ in visited_cells]
    stale = [(t_final - lv)/60 for _,lv in visited_cells]
    sc = ax_d.scatter(vx, vy, s=10, c=stale,
                      cmap=cmap_stale, vmin=0, vmax=max(stale) if stale else 1,
                      edgecolors='none', zorder=3,
                      label=f'Visited ({len(visited_cells)})')
    cb = fig.colorbar(sc, ax=ax_d, pad=0.02, fraction=0.04)
    cb.set_label('Final staleness (min)', fontsize=8)
    cb.ax.tick_params(labelsize=7)

# Draw final drone positions
last_pos = pos.groupby('drone_id').last().reset_index()
lx = [(r['lon']-lon0)*MPLO for _,r in last_pos.iterrows()]
ly = [(r['lat']-lat0)*MPL  for _,r in last_pos.iterrows()]
ax_d.scatter(lx, ly, s=60, color='#1565C0', marker='^',
             zorder=6, label='Final drone pos', edgecolors='white', linewidths=0.8)

ax_d.set_xlabel('East (m)'); ax_d.set_ylabel('North (m)')
ax_d.set_title(f'(d) Final Coverage Map\n'
               f'({len(visited_cells)}/{n_cells} cells, '
               f'{100*len(visited_cells)/n_cells:.1f}%)')
ax_d.set_aspect('equal', 'datalim')
ax_d.legend(loc='best', fontsize=7, markerscale=1.5)
ax_d.grid(True, alpha=0.2)

# ── Stats footer ──────────────────────────────────────────────────
final_cov = cov_df.iloc[-1]
stats = (
    f"Drones: {n_drones}   Cells: {n_cells}   Cell: {int(CELL_M)}m   "
    f"Duration: {(t_max_arr-t_min_arr)/60:.0f} min   "
    f"Final coverage: {final_cov['coverage_pct']:.1f}%   "
    f"Max staleness: {final_cov['max_staleness_s']/60:.1f} min\n"
    f"Min inter-drone dist: {dist_df['min_dist_m'].min():.0f} m   "
    f"Mean inter-drone dist: {dist_df['mean_dist_m'].mean():.0f} m   "
    f"Closest pair ever: "
    f"#{int(dist_df.loc[dist_df['min_dist_m'].idxmin(),'closest_a'])}"
    f"↔#{int(dist_df.loc[dist_df['min_dist_m'].idxmin(),'closest_b'])}"
)
fig.text(0.5, 0.003, stats, ha='center', va='bottom', fontsize=8,
         family='monospace',
         bbox=dict(boxstyle='round,pad=0.4', facecolor='whitesmoke',
                   edgecolor='grey', alpha=0.8))

out_pdf = 'coverage_from_pos_plot.pdf'
out_png = 'coverage_from_pos_plot.png'

plt.savefig(out_png, bbox_inches='tight', dpi=200)

print(f"Saved → {out_png}")
