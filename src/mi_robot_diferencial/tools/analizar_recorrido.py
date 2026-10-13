#!/usr/bin/env python3
"""Grafica y resume el CSV generado por prueba_recorrido.py."""
import csv
import math
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

archivo, salida = sys.argv[1], sys.argv[2]
with open(archivo) as f:
    d = {k: np.array([float(r[k]) for r in csv.DictReader(open(archivo))]) for k in next(csv.reader(f))}
t = d['t']
uw = {k: np.unwrap(d[k + '_th']) for k in ('gt', 'odom', 'kin')}
e_kin_odom = np.hypot(d['kin_x'] - d['odom_x'], d['kin_y'] - d['odom_y'])
e_odom_gt = np.hypot(d['odom_x'] - d['gt_x'], d['odom_y'] - d['gt_y'])
e_kin_gt = np.hypot(d['kin_x'] - d['gt_x'], d['kin_y'] - d['gt_y'])
giro = {k: uw[k][-1] - uw[k][0] for k in uw}
print(f'Duración: {t[-1]:.1f} s')
print(f'Giro total  GT: {math.degrees(giro["gt"]):.1f}°  /odom: {math.degrees(giro["odom"]):.1f}°  nodo: {math.degrees(giro["kin"]):.1f}°')
print(f'Razón giro odom/GT: {giro["odom"] / giro["gt"]:.3f} -> b_efectiva ≈ {0.180 * giro["odom"] / giro["gt"]:.3f} m')
for n, e in (('nodo vs /odom', e_kin_odom), ('/odom vs GT', e_odom_gt), ('nodo vs GT', e_kin_gt)):
    print(f'Error {n}: medio {e.mean()*1000:.1f} mm, máx {e.max()*1000:.1f} mm, final {e[-1]*1000:.1f} mm')
eth = np.abs(uw['kin'] - uw['odom'])
print(f'Error orientación nodo vs /odom: medio {eth.mean():.4f} rad, máx {eth.max():.4f} rad')
print(f'Distancia mínima LiDAR: {d["scan_min"].min():.3f} m')

fig, ax = plt.subplots(1, 2, figsize=(13, 5.5))
ax[0].plot(d['gt_x'], d['gt_y'], 'k', lw=2, label='Ground truth (pose real)')
ax[0].plot(d['odom_x'], d['odom_y'], 'g', label='/odom (DiffDrive)')
ax[0].plot(d['kin_x'], d['kin_y'], 'b--', label='/kinematics/odom (nodo)')
ax[0].set_aspect('equal'); ax[0].grid(alpha=.3); ax[0].legend(fontsize=8)
ax[0].set_title('Trayectorias'); ax[0].set_xlabel('x [m]'); ax[0].set_ylabel('y [m]')
ax[1].plot(t, e_kin_odom * 1000, 'b', label='nodo vs /odom')
ax[1].plot(t, e_odom_gt * 1000, 'g', label='/odom vs ground truth')
ax[1].set_yscale('log'); ax[1].grid(alpha=.3, which='both'); ax[1].legend()
ax[1].set_title('Error de posición euclidiano'); ax[1].set_xlabel('t [s]'); ax[1].set_ylabel('error [mm]')
fig.tight_layout(); fig.savefig(salida, dpi=110)
