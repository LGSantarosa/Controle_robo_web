#!/usr/bin/env python3
"""
Detector de cones a partir do /scan do LiDAR (trekking).

Estratégia:
  1. /scan → pontos cartesianos no frame base_laser
  2. Clustering por gap (vizinhos com distância > gap_thr abrem novo cluster)
  3. Filtra cluster por extensão geométrica (largura ~ de cone, 5–40 cm) e
     número mínimo de pontos
  4. Transforma centróides: base_laser → base_link (offset fixo `lidar_offset_x`)
                            base_link → odom (usando /trekking/pose mais recente)
  5. Publica geometry_msgs/PoseArray em /trekking/cones (frame: odom).
     Cada pose: position = (x, y, 0). A largura aparente do cluster é codada
     em orientation.x (em metros) — útil pro UI/runner classificar.

Não usa TF do ROS — depende só da pose publicada pelo pose_estimator. Isso
mantém o cone_detector autônomo (não acorda se o pose_estimator não estiver
publicando).
"""
import math
import threading

import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def _quat_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class ConeDetector(Node):

    def __init__(self):
        super().__init__('cone_detector')

        # --- Geometria do robô ---
        self.declare_parameter('lidar_offset_x', 0.10)   # base_laser deslocado em x
        self.declare_parameter('lidar_offset_y', 0.00)

        # --- Clustering ---
        self.declare_parameter('gap_threshold', 0.08)        # m — gap pra fechar cluster
        self.declare_parameter('min_cluster_points', 2)
        self.declare_parameter('min_cluster_width', 0.04)    # m
        self.declare_parameter('max_cluster_width', 0.45)    # m (cone + margem)
        # Janela de range — fora disso descarta
        self.declare_parameter('range_min', 0.15)
        self.declare_parameter('range_max', 5.0)
        # Janela angular (no frame base_laser; útil pra ignorar trás do robô se
        # quiser, mas default usa scan inteiro)
        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max',  math.pi)

        # --- Saída ---
        self.declare_parameter('odom_frame', 'odom')

        self.off_x          = float(self.get_parameter('lidar_offset_x').value)
        self.off_y          = float(self.get_parameter('lidar_offset_y').value)
        self.gap_thr        = float(self.get_parameter('gap_threshold').value)
        self.min_pts        = int(self.get_parameter('min_cluster_points').value)
        self.min_w          = float(self.get_parameter('min_cluster_width').value)
        self.max_w          = float(self.get_parameter('max_cluster_width').value)
        self.r_min          = float(self.get_parameter('range_min').value)
        self.r_max          = float(self.get_parameter('range_max').value)
        self.a_min          = float(self.get_parameter('angle_min').value)
        self.a_max          = float(self.get_parameter('angle_max').value)
        self.odom_frame     = self.get_parameter('odom_frame').value

        # --- Estado ---
        self._lock = threading.Lock()
        self._pose_x = 0.0
        self._pose_y = 0.0
        self._pose_yaw = 0.0
        self._have_pose = False

        # --- Subs/Pubs ---
        self.create_subscription(PoseStamped, 'trekking/pose', self._on_pose, 10)
        self.create_subscription(LaserScan, 'scan', self._on_scan, 10)
        self.pub_cones = self.create_publisher(PoseArray, 'trekking/cones', 10)

        self.get_logger().info(
            f'cone_detector: gap={self.gap_thr*100:.1f} cm | '
            f'largura {self.min_w*100:.0f}–{self.max_w*100:.0f} cm | '
            f'range {self.r_min:.2f}–{self.r_max:.2f} m'
        )

    # ------------------------------------------------------------------
    def _on_pose(self, msg: PoseStamped):
        with self._lock:
            self._pose_x = msg.pose.position.x
            self._pose_y = msg.pose.position.y
            self._pose_yaw = _quat_to_yaw(
                msg.pose.orientation.x, msg.pose.orientation.y,
                msg.pose.orientation.z, msg.pose.orientation.w,
            )
            self._have_pose = True

    def _on_scan(self, scan: LaserScan):
        with self._lock:
            if not self._have_pose:
                return
            px, py, pyaw = self._pose_x, self._pose_y, self._pose_yaw

        # 1) Scan → (xl, yl) no frame base_laser
        points = []  # lista de (xl, yl)
        a = scan.angle_min
        da = scan.angle_increment
        for r in scan.ranges:
            ang = a
            a += da
            if not math.isfinite(r):
                continue
            if r < self.r_min or r > self.r_max:
                continue
            if ang < self.a_min or ang > self.a_max:
                continue
            points.append((r * math.cos(ang), r * math.sin(ang)))

        if not points:
            self._publish_empty(scan)
            return

        # 2) Cluster sequencial por gap. As leituras do LD20 vêm ordenadas em
        # ângulo, então caminhar em sequência cobre o caso (com o wrap, o
        # último cluster pode tocar o primeiro, mas a perda é desprezível
        # pra cones — só pioraria se um cone caísse exatamente em ±π).
        clusters = []
        cur = [points[0]]
        for i in range(1, len(points)):
            xp, yp = points[i - 1]
            xc, yc = points[i]
            if math.hypot(xc - xp, yc - yp) <= self.gap_thr:
                cur.append((xc, yc))
            else:
                clusters.append(cur)
                cur = [(xc, yc)]
        clusters.append(cur)

        # 3) Filtra por número de pontos e por largura (extensão geométrica
        # entre primeiro e último ponto do cluster — barato e suficiente
        # pra cones, que são pequenos e convexos).
        cones_laser = []     # (cx, cy, width)
        for c in clusters:
            if len(c) < self.min_pts:
                continue
            x0, y0 = c[0]
            xN, yN = c[-1]
            width = math.hypot(xN - x0, yN - y0)
            if width < self.min_w or width > self.max_w:
                continue
            cx = sum(p[0] for p in c) / len(c)
            cy = sum(p[1] for p in c) / len(c)
            cones_laser.append((cx, cy, width))

        if not cones_laser:
            self._publish_empty(scan)
            return

        # 4) base_laser → base_link (translação fixa, sem rotação) → odom
        cosy = math.cos(pyaw); siny = math.sin(pyaw)
        msg = PoseArray()
        msg.header.stamp = scan.header.stamp
        msg.header.frame_id = self.odom_frame
        for cx, cy, w in cones_laser:
            # Em base_link:
            bx = cx + self.off_x
            by = cy + self.off_y
            # Em odom:
            wx = px + bx * cosy - by * siny
            wy = py + bx * siny + by * cosy
            p = Pose()
            p.position.x = wx
            p.position.y = wy
            p.position.z = 0.0
            # Largura aparente no orientation.x (uso interno)
            p.orientation.x = w
            p.orientation.w = 1.0
            msg.poses.append(p)
        self.pub_cones.publish(msg)

    def _publish_empty(self, scan: LaserScan):
        msg = PoseArray()
        msg.header.stamp = scan.header.stamp
        msg.header.frame_id = self.odom_frame
        self.pub_cones.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ConeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
