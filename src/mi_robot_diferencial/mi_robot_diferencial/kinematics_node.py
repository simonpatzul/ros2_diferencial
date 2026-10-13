#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

class Yahboom4WDKinematicsNode(Node):
    def __init__(self):
        super().__init__('yahboom_4wd_kinematics_node')

        # Parámetros exactos del robot Yahboom MicroROS
        self.r = 0.024        # Radio de rueda (m)
        self.b = 0.180        # Separación entre ruedas Izq-Der (m)

        # Pose Integrada
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # Pose Gazebo
        self.gazebo_x = 0.0
        self.gazebo_y = 0.0
        self.odom_received = False

        # Subscripciones y Publicadores
        self.sub_joints = self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.pub_custom_odom = self.create_publisher(Odometry, '/kinematics/odom', 10)
        self.pub_error = self.create_publisher(Float64, '/kinematics/position_error', 10)

        self.get_logger().info("Nodo de Cinemática Yahboom 4WD (Skid-Steering) iniciado.")

    def odom_callback(self, msg: Odometry):
        self.gazebo_x = msg.pose.pose.position.x
        self.gazebo_y = msg.pose.pose.position.y
        self.odom_received = True

    def joint_states_callback(self, msg: JointState):
        try:
            # Capturamos la velocidad de los 4 motores
            idx_fl = msg.name.index('joint_rueda_del_izq')
            idx_rl = msg.name.index('joint_rueda_tra_izq')
            idx_fr = msg.name.index('joint_rueda_del_der')
            idx_rr = msg.name.index('joint_rueda_tra_der')
            
            # Promediamos las velocidades de cada lado (Principio Skid-Steer)
            w_left = (msg.velocity[idx_fl] + msg.velocity[idx_rl]) / 2.0
            w_right = (msg.velocity[idx_fr] + msg.velocity[idx_rr]) / 2.0
        except (ValueError, IndexError) as e:
            self.get_logger().error(f"Error leyendo articulaciones. Gazebo está enviando estos nombres: {msg.name}")
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        if dt <= 0.0 or dt > 0.5:
            return

        # 1. Matriz Cinemática
        v = (self.r / 2.0) * (w_right + w_left)
        w = (self.r / self.b) * (w_right - w_left)

        # 2. Derivada e Integración de la Pose
        self.x += (v * math.cos(self.theta)) * dt
        self.y += (v * math.sin(self.theta)) * dt
        self.theta += w * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # 3. Publicar resultados
        self.publish_custom_odometry(current_time, v, w)

        if self.odom_received:
            error_pos = math.sqrt((self.x - self.gazebo_x)**2 + (self.y - self.gazebo_y)**2)
            error_msg = Float64()
            error_msg.data = error_pos
            self.pub_error.publish(error_msg)
            self.get_logger().info(
                f"Pose 4WD: ({self.x:.3f}, {self.y:.3f}) | Gazebo: ({self.gazebo_x:.3f}, {self.gazebo_y:.3f}) | Error: {error_pos * 1000:.2f} mm",
                throttle_duration_sec=1.0
            )

    def publish_custom_odometry(self, stamp, v, w):
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.pub_custom_odom.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = Yahboom4WDKinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()