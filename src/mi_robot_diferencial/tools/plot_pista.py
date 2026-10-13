#!/usr/bin/env python3
"""Dibuja la vista superior de la pista a partir de generar_pista.py."""
import math
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import generar_pista as gp

pts = gp.construir_pista()
fig, ax = plt.subplots(figsize=(9, 7))
for lado, c in ((1, '#888'), (-1, '#c0501a')):
    b = gp.desplazar(pts, lado)
    b.append(b[0])
    ax.plot([p[0] for p in b], [p[1] for p in b], color=c, lw=3)
cl = pts + [pts[0]]
ax.plot([p[0] for p in cl], [p[1] for p in cl], '--', color='#2a7', lw=1, label='Línea central')
for _, x, y, _, lx, ly in [('', 1.0, 0.22, 0, 0.25, 0.40), ('', 2.0, -0.22, 0, 0.25, 0.40)]:
    ax.add_patch(plt.Rectangle((x - lx / 2, y - ly / 2), lx, ly, color='#1f66cc'))
ax.add_patch(plt.Circle((4.2, 2.0), 0.12, color='#d8c21a'))
ax.plot(0, 0, 'k^', ms=12, label='Spawn (0, 0, 0°)')
ax.annotate('Estrechamiento 0.40 m', xy=(1.5, 6.5), xytext=(0.6, 7.3), arrowprops=dict(arrowstyle='->'))
ax.annotate('Chicana en S', xy=(4.4, 4.1), xytext=(5.2, 3.0), arrowprops=dict(arrowstyle='->'))
ax.annotate('Slalom', xy=(1.5, 0), xytext=(2.3, -1.2), arrowprops=dict(arrowstyle='->'))
ax.set_ylim(-1.6, 7.8); ax.set_aspect('equal'); ax.grid(alpha=.3); ax.legend(loc='center')
ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]'); ax.set_title('Pista propia: vista superior')
fig.savefig(sys.argv[1] if len(sys.argv) > 1 else 'pista.png', dpi=110, bbox_inches='tight')
