#!/usr/bin/env python3
"""
Publica la transformación map -> odom (convención REP-105).

La odometría por ruedas (/odom, TF odom -> base_footprint) deriva con el
derrape del skid-steer, sobre todo al girar. Este nodo calcula la corrección
con la pose real del simulador (/ground_truth/odom):

    T_map_odom = T_map_base(real) * inv(T_odom_base(ruedas))

Así, con 'map' como Fixed Frame en RViz, la pista y el LiDAR quedan quietos
aunque la odometría derive. El TF odom -> base_footprint del DiffDrive no se
modifica.
"""
import math
from collections import deque

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformBroadcaster


TOLERANCIA = Duration(seconds=0.1)


def pose2d(msg):
    p = msg.pose.pose
    return p.position.x, p.position.y, 2.0 * math.atan2(p.orientation.z, p.orientation.w)


def stamp_s(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class MapTfNode(Node):
    def __init__(self):
        super().__init__('map_tf_node')
        self.odoms = deque(maxlen=30)
        self.tf_pub = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/odom', lambda m: self.odoms.append(m), 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self.cb_gt, 10)

    def cb_gt(self, gt):
        if not self.odoms:
            return
        # Odometría con el sello de tiempo más cercano al de la pose real
        t = stamp_s(gt)
        odom = min(self.odoms, key=lambda m: abs(stamp_s(m) - t))
        xg, yg, thg = pose2d(gt)
        xo, yo, tho = pose2d(odom)
        th = thg - tho
        c, s = math.cos(th), math.sin(th)

        tf = TransformStamped()
        # Sello adelantado (como transform_tolerance de AMCL): map -> odom cambia
        # lento, y así RViz no descarta barridos del LiDAR por falta de TF.
        tf.header.stamp = (Time.from_msg(gt.header.stamp) + TOLERANCIA).to_msg()
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'odom'
        tf.transform.translation.x = xg - (c * xo - s * yo)
        tf.transform.translation.y = yg - (s * xo + c * yo)
        tf.transform.rotation.z = math.sin(th / 2.0)
        tf.transform.rotation.w = math.cos(th / 2.0)
        self.tf_pub.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = MapTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
