#!/usr/bin/env python3
"""
Prueba de recorrido: conduce el robot por la pista completa siguiendo la línea
central (con desvíos en los obstáculos) mediante Pure Pursuit, y registra:
  - /ground_truth/odom  (pose real del simulador)
  - /odom               (odometría del plugin DiffDrive)
  - /kinematics/odom    (odometría del nodo propio)
  - /kinematics/position_error
  - distancia mínima medida por el LiDAR

No reemplaza la teleoperación (que se demuestra en vivo); sirve para verificar
que la pista es transitable y para obtener datos del análisis de error.

Uso (con la simulación corriendo):
    python3 prueba_recorrido.py salida.csv
"""
import csv
import math
import os
import sys

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import generar_pista as gp  # noqa: E402

V = 0.15           # Velocidad lineal (m/s)
LOOKAHEAD = 0.30   # Distancia de anticipación de Pure Pursuit (m)


def yaw(q):
    return 2.0 * math.atan2(q.z, q.w)


def ruta():
    """Línea central densa (cada 5 cm) con desvíos para esquivar obstáculos."""
    pts = [(p[0], p[1]) for p in gp.construir_pista()]
    pts.append(pts[0])
    densa = []
    for (x0, y0), (x1, y1) in zip(pts[:-1], pts[1:]):
        n = max(1, int(math.hypot(x1 - x0, y1 - y0) / 0.05))
        densa += [(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n) for i in range(n)]
    out = []
    for x, y in densa:
        if y < 0.5 and 0.4 < x < 1.5:     # caja 1 (lado +y): pasar por -y
            y = -0.24
        elif y < 0.5 and 1.5 <= x < 2.6:  # caja 2 (lado -y): pasar por +y
            y = 0.24
        elif x > 3.0 and 1.5 < y < 2.5:   # cilindro (lado +x): pasar por -x
            x = 3.80
        out.append((x, y))
    out.append(out[0])
    return out


class Prueba(Node):
    def __init__(self, archivo):
        super().__init__('prueba_recorrido')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.ruta = ruta()
        self.idx = 0
        self.gt = self.odom = self.kin = None
        self.err = float('nan')
        self.scan_min = float('inf')
        self.scan_min_global = float('inf')
        self.t0 = None
        self.terminado = False
        self.archivo = open(archivo, 'w', newline='')
        self.csv = csv.writer(self.archivo)
        self.csv.writerow(['t', 'gt_x', 'gt_y', 'gt_th', 'odom_x', 'odom_y', 'odom_th',
                           'kin_x', 'kin_y', 'kin_th', 'error_pos', 'scan_min'])
        self.create_subscription(Odometry, '/ground_truth/odom', self.cb_gt, 10)
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'odom', m), 10)
        self.create_subscription(Odometry, '/kinematics/odom', lambda m: setattr(self, 'kin', m), 10)
        self.create_subscription(Float64, '/kinematics/position_error',
                                 lambda m: setattr(self, 'err', m.data), 10)
        self.create_subscription(LaserScan, '/scan', self.cb_scan, qos_profile_sensor_data)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.05, self.control)
        self.create_timer(0.1, self.registrar)

    def cb_scan(self, m):
        validos = [r for r in m.ranges if m.range_min < r < m.range_max]
        if validos:
            self.scan_min = min(validos)
            self.scan_min_global = min(self.scan_min_global, self.scan_min)

    def cb_gt(self, m):
        self.gt = m

    def control(self):
        if self.gt is None or self.terminado:
            return
        p = self.gt.pose.pose
        x, y, th = p.position.x, p.position.y, yaw(p.orientation)
        # Avanzar el índice objetivo hasta el punto a distancia LOOKAHEAD
        while self.idx < len(self.ruta) - 1 and \
                math.hypot(self.ruta[self.idx][0] - x, self.ruta[self.idx][1] - y) < LOOKAHEAD:
            self.idx += 1
        tx, ty = self.ruta[self.idx]
        if self.idx >= len(self.ruta) - 1 and math.hypot(tx - x, ty - y) < 0.1:
            self.terminado = True
            self.pub.publish(Twist())
            return
        alpha = math.atan2(ty - y, tx - x) - th
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))
        cmd = Twist()
        cmd.linear.x = V if abs(alpha) < 1.0 else 0.03
        cmd.angular.z = max(-1.5, min(1.5, 2.0 * V * math.sin(alpha) / LOOKAHEAD * 2.0))
        self.pub.publish(cmd)

    def registrar(self):
        if None in (self.gt, self.odom, self.kin):
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = t
        fila = [t - self.t0]
        for m in (self.gt, self.odom, self.kin):
            p = m.pose.pose
            fila += [p.position.x, p.position.y, yaw(p.orientation)]
        fila += [self.err, self.scan_min]
        self.csv.writerow([f'{v:.5f}' for v in fila])


def main():
    rclpy.init()
    nodo = Prueba(sys.argv[1] if len(sys.argv) > 1 else 'recorrido.csv')
    try:
        while rclpy.ok() and not nodo.terminado:
            rclpy.spin_once(nodo, timeout_sec=0.1)
        nodo.get_logger().info(
            f'Recorrido completo. Distancia mínima LiDAR: {nodo.scan_min_global:.3f} m')
    except KeyboardInterrupt:
        pass
    finally:
        nodo.pub.publish(Twist())
        nodo.archivo.close()
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
