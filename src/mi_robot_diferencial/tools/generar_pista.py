#!/usr/bin/env python3
"""
Generador de la pista (world SDF) para el robot diferencial.

La pista se define por una línea central compuesta de rectas y arcos, y un
ancho de corredor en cada tramo. Los muros izquierdo y derecho se obtienen
desplazando la línea central +-ancho/2 sobre su normal, y cada par de puntos
consecutivos se convierte en una caja con colisión.

Uso:
    python3 generar_pista.py > ../worlds/pista.sdf
"""
import math

# ======================= PARÁMETROS DE DISEÑO =======================
ANCHO = 0.90            # Ancho normal del corredor (m)  ~4.5x ancho del robot
ANCHO_ESTRECHO = 0.40   # Ancho en el estrechamiento (m) ~2x ancho del robot
MURO_ALTO = 0.30        # Alto del muro (m): > altura del LiDAR (~0.11 m)
MURO_ESPESOR = 0.05     # Espesor del muro (m)
PASO_ARCO = math.radians(10)  # Discretización de las curvas
R = 1.0                 # Radio de las curvas de 90 grados (m)
R_CHICANE = 0.8         # Radio de las curvas de la chicana (m)


class LineaCentral:
    """Acumula puntos (x, y, ancho) recorriendo rectas y arcos."""

    def __init__(self):
        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.puntos = [(0.0, 0.0, 0.0, ANCHO)]

    def _agregar(self, ancho):
        self.puntos.append((self.x, self.y, self.th, ancho))

    def recta(self, largo, ancho=ANCHO, ancho_final=None, pasos=1):
        ancho_final = ancho if ancho_final is None else ancho_final
        for i in range(1, pasos + 1):
            self.x += largo / pasos * math.cos(self.th)
            self.y += largo / pasos * math.sin(self.th)
            self._agregar(ancho + (ancho_final - ancho) * i / pasos)

    def arco(self, radio, angulo_deg, ancho=ANCHO):
        """Arco de radio dado; ángulo > 0 gira a la izquierda."""
        ang = math.radians(angulo_deg)
        signo = 1.0 if ang > 0 else -1.0
        cx = self.x - signo * radio * math.sin(self.th)
        cy = self.y + signo * radio * math.cos(self.th)
        n = max(1, int(math.ceil(abs(ang) / PASO_ARCO)))
        th0 = self.th
        for i in range(1, n + 1):
            self.th = th0 + ang * i / n
            self.x = cx + signo * radio * math.sin(self.th)
            self.y = cy - signo * radio * math.cos(self.th)
            self._agregar(ancho)


def construir_pista():
    lc = LineaCentral()
    # 1. Recta de salida con slalom de obstáculos (rumbo +x)
    lc.recta(3.0, pasos=3)
    # 2. Curva izquierda de 90 grados (rumbo +y)
    lc.arco(R, 90)
    # 3. Recta con obstáculo cilíndrico
    lc.recta(2.0, pasos=2)
    # 4. Chicana en S (derecha 45, izquierda 90, derecha 45)
    lc.arco(R_CHICANE, -45)
    lc.arco(R_CHICANE, 90)
    lc.arco(R_CHICANE, -45)
    # 5. Curva izquierda de 90 grados (rumbo -x)
    lc.arco(R, 90)
    # 6. Recta de regreso con ESTRECHAMIENTO. Su largo se calcula para que
    #    la pista cierre exactamente en el punto de salida.
    largo_regreso = lc.x - 0.0
    transicion = 0.6
    plano = largo_regreso - 2 * transicion
    assert plano > 0.5, 'La recta de regreso es muy corta'
    lc.recta(transicion, ANCHO, ANCHO_ESTRECHO, pasos=3)
    lc.recta(plano, ANCHO_ESTRECHO)
    lc.recta(transicion, ANCHO_ESTRECHO, ANCHO, pasos=3)
    # 7. Curva izquierda (rumbo -y), recta de bajada y curva final (rumbo +x)
    lc.arco(R, 90)
    largo_bajada = lc.y - R
    assert largo_bajada > 0.5, 'La recta de bajada es muy corta'
    lc.recta(largo_bajada, pasos=2)
    lc.arco(R, 90)
    # El último punto coincide con el primero: se descarta para cerrar.
    err = math.hypot(lc.x, lc.y)
    assert err < 1e-6, f'La pista no cierra (error {err:.4f} m)'
    return lc.puntos[:-1]


def desplazar(puntos, lado):
    """Desplaza los puntos sobre la normal (lado=+1 izquierda, -1 derecha)."""
    return [(x - lado * a / 2 * math.sin(th), y + lado * a / 2 * math.cos(th))
            for x, y, th, a in puntos]


def caja(nombre, x, y, yaw, largo, ancho, alto, color):
    geom = f'<box><size>{largo:.4f} {ancho:.4f} {alto:.4f}</size></box>'
    return f'''
      <collision name="{nombre}_col">
        <pose>{x:.4f} {y:.4f} {alto / 2:.4f} 0 0 {yaw:.4f}</pose>
        <geometry>{geom}</geometry>
      </collision>
      <visual name="{nombre}_vis">
        <pose>{x:.4f} {y:.4f} {alto / 2:.4f} 0 0 {yaw:.4f}</pose>
        <geometry>{geom}</geometry>
        <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
      </visual>'''


def muros(puntos):
    out = []
    for lado, nombre in ((1, 'izq'), (-1, 'der')):
        borde = desplazar(puntos, lado)
        color = '0.85 0.85 0.85 1' if lado == 1 else '0.8 0.3 0.1 1'
        for i in range(len(borde)):
            (x0, y0), (x1, y1) = borde[i], borde[(i + 1) % len(borde)]
            largo = math.hypot(x1 - x0, y1 - y0)
            if largo < 1e-6:
                continue
            out.append(caja(f'muro_{nombre}_{i}', (x0 + x1) / 2, (y0 + y1) / 2,
                            math.atan2(y1 - y0, x1 - x0),
                            largo + MURO_ESPESOR, MURO_ESPESOR, MURO_ALTO, color))
    return ''.join(out)


def obstaculos():
    """Obstáculos estáticos dentro del corredor (pose x y yaw)."""
    cajas = [
        # Slalom en la recta de salida: alternados a cada lado del corredor
        ('obs_caja_1', 1.0, 0.22, 0.0, 0.25, 0.40),
        ('obs_caja_2', 2.0, -0.22, 0.0, 0.25, 0.40),
    ]
    out = []
    for n, x, y, yaw, lx, ly in cajas:
        out.append(caja(n, x, y, yaw, lx, ly, 0.25, '0.1 0.4 0.8 1'))
    # Cilindro en la recta vertical (x = 3 + R = 4.0)
    cx, cy, r, h = 4.0 + 0.2, 2.0, 0.12, 0.25
    out.append(f'''
      <collision name="obs_cilindro_col">
        <pose>{cx} {cy} {h / 2} 0 0 0</pose>
        <geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>
      </collision>
      <visual name="obs_cilindro_vis">
        <pose>{cx} {cy} {h / 2} 0 0 0</pose>
        <geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>
        <material><ambient>0.9 0.8 0.1 1</ambient><diffuse>0.9 0.8 0.1 1</diffuse></material>
      </visual>''')
    return ''.join(out)


def generar_sdf():
    puntos = construir_pista()
    return f'''<?xml version="1.0" ?>
<!-- ARCHIVO GENERADO por tools/generar_pista.py. No editar a mano. -->
<sdf version="1.9">
  <world name="pista">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>

    <light type="directional" name="sol">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="suelo">
      <static>true</static>
      <link name="link">
        <collision name="col">
          <geometry><plane><normal>0 0 1</normal><size>30 30</size></plane></geometry>
        </collision>
        <visual name="vis">
          <geometry><plane><normal>0 0 1</normal><size>30 30</size></plane></geometry>
          <material><ambient>0.35 0.35 0.35 1</ambient><diffuse>0.35 0.35 0.35 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- Línea de salida/meta (solo visual). Spawn del robot en (0, 0, yaw=0) -->
    <model name="linea_salida">
      <static>true</static>
      <link name="link">
        <visual name="vis">
          <pose>0.3 0 0.001 0 0 0</pose>
          <geometry><box><size>0.05 {ANCHO} 0.001</size></box></geometry>
          <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material>
        </visual>
      </link>
    </model>

    <model name="muros_pista">
      <static>true</static>
      <link name="link">{muros(puntos)}
      </link>
    </model>

    <model name="obstaculos">
      <static>true</static>
      <link name="link">{obstaculos()}
      </link>
    </model>
  </world>
</sdf>
'''


if __name__ == '__main__':
    print(generar_sdf())
