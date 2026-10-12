
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class InverseKinematics(Node):

    def __init__(self):
        super().__init__('inverse_kinematics')

        # PARAMETER ROBOT
        self.max_linear_speed = 0.15
        self.max_angular_speed = 0.60
        self.waypoint_tolerance = 0.035
        self.yaw_tolerance = math.radians(3.0)

        # LINTASAN PERSEGI PANJANG: 2 x 1 meter
        # Diasumsikan robot mulai di (0, 0) dan menghadap +X.
        self.waypoints = [
            (2.0, 0.0),
            (2.0, 1.0),
            (0.0, 1.0),
            (0.0, 0.0)
        ]

        self.waypoint_index = 0
        self.x = None
        self.y = None
        self.yaw = None
        self.initial_yaw = None

        self.returning_to_initial_yaw = False
        self.finished = False

        # SUBSCRIBER ODOMETRI DARI GAZEBO
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # PUBLISHER GERAK UNTUK GAZEBO DIFFDRIVE
        # Hindari subscribe dan publish /cmd_vel pada node yang sama.
        self.motion_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # KONTROL 20 Hz
        self.control_timer = self.create_timer(
            0.05,
            self.control_callback
        )

        # MONITORING STATUS SETIAP 0,5 DETIK
        self.log_timer = self.create_timer(
            0.5,
            self.display_status
        )

        self.get_logger().info('======================================')
        self.get_logger().info(' ROBOT WAYPOINT PERSEGI PANJANG 2 x 1 m')
        self.get_logger().info('======================================')
        self.get_logger().info('Odometry : /odom')
        self.get_logger().info('Perintah : /cmd_vel')
        self.get_logger().info('Menunggu data odometri...')

    # MEMBACA POSISI DAN ORIENTASI ROBOT
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        # Quaternion menjadi yaw
        sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
        cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

        self.yaw = math.atan2(sin_yaw, cos_yaw)

        # Simpan orientasi awal satu kali
        if self.initial_yaw is None:
            self.initial_yaw = self.yaw
            self.get_logger().info(
                'Orientasi awal: '
                f'{math.degrees(self.initial_yaw):.1f} derajat'
            )

    # NORMALISASI SELISIH SUDUT KE RENTANG -PI SAMPAI +PI
    @staticmethod
    def angle_error(target, current):
        return math.atan2(
            math.sin(target - current),
            math.cos(target - current)
        )

    # MENGIRIM PERINTAH KE GAZEBO
    def publish_velocity(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.motion_publisher.publish(msg)

    # KONTROL OTOMATIS WAYPOINT
    def control_callback(self):
        if (
            self.x is None
            or self.y is None
            or self.yaw is None
            or self.initial_yaw is None
        ):
            return

        # Jika semua tahap selesai, pertahankan robot berhenti
        if self.finished:
            self.publish_velocity(0.0, 0.0)
            return

        # TAHAP AKHIR: KEMBALI MENGHADAP ARAH AWAL
        if self.returning_to_initial_yaw:
            yaw_error = self.angle_error(
                self.initial_yaw,
                self.yaw
            )

            if abs(yaw_error) <= self.yaw_tolerance:
                self.publish_velocity(0.0, 0.0)
                self.finished = True

                self.get_logger().info(
                    'LINTASAN SELESAI. Robot kembali menghadap arah awal. '
                    f'X={self.x:.3f} m, Y={self.y:.3f} m, '
                    f'Yaw={math.degrees(self.yaw):.1f} derajat'
                )
                return

            angular = 1.8 * yaw_error
            angular = max(
                -self.max_angular_speed,
                min(self.max_angular_speed, angular)
            )

            self.publish_velocity(0.0, angular)
            return

        # PERIKSA APAKAH SEMUA WAYPOINT SUDAH DICAPAI
        if self.waypoint_index >= len(self.waypoints):
            self.returning_to_initial_yaw = True
            self.publish_velocity(0.0, 0.0)
            return

        target_x, target_y = self.waypoints[self.waypoint_index]

        dx = target_x - self.x
        dy = target_y - self.y
        distance = math.hypot(dx, dy)

        # WAYPOINT TERCAPAI
        if distance <= self.waypoint_tolerance:
            self.get_logger().info(
                f'Waypoint {self.waypoint_index + 1} tercapai: '
                f'X={self.x:.3f} m, Y={self.y:.3f} m'
            )

            self.waypoint_index += 1
            self.publish_velocity(0.0, 0.0)

            # Setelah waypoint terakhir, pulihkan orientasi awal
            if self.waypoint_index >= len(self.waypoints):
                self.returning_to_initial_yaw = True
                self.get_logger().info(
                    'Semua waypoint tercapai. '
                    'Mengembalikan orientasi ke arah awal...'
                )
                return

            # Ambil target berikutnya
            target_x, target_y = self.waypoints[self.waypoint_index]
            dx = target_x - self.x
            dy = target_y - self.y
            distance = math.hypot(dx, dy)

        # ARAH YANG HARUS DITUJU
        target_yaw = math.atan2(dy, dx)
        yaw_error = self.angle_error(target_yaw, self.yaw)

        # PUTAR DULU JIKA ARAH BELUM SESUAI
        if abs(yaw_error) > 0.12:
            linear = 0.0
            angular = 1.8 * yaw_error
        else:
            linear = min(
                self.max_linear_speed,
                0.55 * distance
            )
            angular = 1.5 * yaw_error

        angular = max(
            -self.max_angular_speed,
            min(self.max_angular_speed, angular)
        )

        self.publish_velocity(linear, angular)

    # MONITORING KOORDINAT GLOBAL
    def display_status(self):
        if self.x is None or self.y is None or self.yaw is None:
            self.get_logger().info(
                'Menunggu data odometri dari /odom...'
            )
            return

        if self.finished:
            target_text = 'SELESAI'
            distance = 0.0

        elif self.returning_to_initial_yaw:
            target_text = 'KEMBALI KE ARAH AWAL'
            distance = 0.0

        elif self.waypoint_index < len(self.waypoints):
            tx, ty = self.waypoints[self.waypoint_index]
            distance = math.hypot(tx - self.x, ty - self.y)
            target_text = f'({tx:.2f}, {ty:.2f})'

        else:
            target_text = 'MENYELESAIKAN'
            distance = 0.0

        self.get_logger().info(
            f'X={self.x:+.3f} m | '
            f'Y={self.y:+.3f} m | '
            f'YAW={math.degrees(self.yaw):+.1f} deg | '
            f'TARGET={target_text} | '
            f'DIST={distance:.3f} m'
        )

    # MENGHENTIKAN ROBOT
    def stop_robot(self):
        self.publish_velocity(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematics()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('Program dihentikan.')

    finally:
        node.stop_robot()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()



# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# import time


# class MoverNode(Node):

#     def __init__(self):
#         super().__init__('mover_node')

#         self.publisher_ = self.create_publisher(
#             Twist,
#             '/cmd_vel',
#             10
#         )

#         # =========================
#         # PARAMETER
#         # =========================

#         self.panjang = 2.0       # meter
#         self.lebar = 1.0         # meter

#         self.kecepatan = 0.5     # m/s
#         self.kecepatan_putar = 0.88   # rad/s

#         # =========================
#         # KOREKSI SETIAP PUTARAN
#         # =========================

#         # Putaran 1
#         self.koreksi_putar_1 = 1.31

#         # Putaran 2
#         self.koreksi_putar_2 = 1.31

#         # Putaran 3
#         self.koreksi_putar_3 = 1.31

#         # Putaran 4
#         self.koreksi_putar_4 = 1.31

#         # =========================
#         # WAKTU GERAK
#         # =========================

#         self.waktu_panjang = self.panjang / self.kecepatan
#         self.waktu_lebar = self.lebar / self.kecepatan

#         # Waktu teoritis 90 derajat
#         self.waktu_putar_teori = (
#             1.5708 / self.kecepatan_putar
#         )

#         # Waktu masing-masing putaran
#         self.waktu_putar_1 = (
#             self.waktu_putar_teori *
#             self.koreksi_putar_1
#         )

#         self.waktu_putar_2 = (
#             self.waktu_putar_teori *
#             self.koreksi_putar_2
#         )

#         self.waktu_putar_3 = (
#             self.waktu_putar_teori *
#             self.koreksi_putar_3
#         )

#         self.waktu_putar_4 = (
#             self.waktu_putar_teori *
#             self.koreksi_putar_4
#         )

#         # =========================
#         # STATE
#         # =========================

#         self.state = 0

#         self.start_time = time.time()

#         # =========================
#         # TIMER
#         # =========================

#         self.timer = self.create_timer(
#             0.1,
#             self.timer_callback
#         )

#         self.get_logger().info(
#             'Robot mulai membuat persegi panjang'
#         )

#         self.get_logger().info(
#             f'Putaran 1 = {self.waktu_putar_1:.3f} s'
#         )

#         self.get_logger().info(
#             f'Putaran 2 = {self.waktu_putar_2:.3f} s'
#         )

#         self.get_logger().info(
#             f'Putaran 3 = {self.waktu_putar_3:.3f} s'
#         )

#         self.get_logger().info(
#             f'Putaran 4 = {self.waktu_putar_4:.3f} s'
#         )

#     def timer_callback(self):

#         msg = Twist()

#         waktu = time.time() - self.start_time

#         # =========================
#         # SISI 1
#         # =========================

#         if self.state == 0:

#             msg.linear.x = self.kecepatan

#             if waktu >= self.waktu_panjang:

#                 self.state = 1
#                 self.start_time = time.time()

#                 self.get_logger().info(
#                     'Sisi 1 selesai'
#                 )

#         # =========================
#         # PUTARAN 1
#         # =========================

#         elif self.state == 1:

#             msg.angular.z = self.kecepatan_putar

#             if waktu >= self.waktu_putar_1:

#                 self.state = 2
#                 self.start_time = time.time()

#                 self.get_logger().info(
#                     'Putaran 1 selesai'
#                 )

#         # =========================
#         # SISI 2
#         # =========================

#         elif self.state == 2:

#             msg.linear.x = self.kecepatan

#             if waktu >= self.waktu_lebar:

#                 self.state = 3
#                 self.start_time = time.time()

#                 self.get_logger().info(
#                     'Sisi 2 selesai'
#                 )

#         # =========================
#         # PUTARAN 2
#         # =========================

#         elif self.state == 3:

#             msg.angular.z = self.kecepatan_putar

#             if waktu >= self.waktu_putar_2:

#                 self.state = 4
#                 self.start_time = time.time()

#                 self.get_logger().info(
#                     'Putaran 2 selesai'
#                 )

#         # =========================
#         # SISI 3
#         # =========================

#         elif self.state == 4:

#             msg.linear.x = self.kecepatan

#             if waktu >= self.waktu_panjang:

#                 self.state = 5
#                 self.start_time = time.time()

#                 self.get_logger().info(
#                     'Sisi 3 selesai'
#                 )

#         # =========================
#         # PUTARAN 3
#         # =========================

#         elif self.state == 5:

#             msg.angular.z = self.kecepatan_putar

#             if waktu >= self.waktu_putar_3:

#                 self.state = 6
#                 self.start_time = time.time()

#                 self.get_logger().info(
#                     'Putaran 3 selesai'
#                 )

#         # =========================
#         # SISI 4
#         # =========================

#         elif self.state == 6:

#             msg.linear.x = self.kecepatan

#             if waktu >= self.waktu_lebar:

#                 self.state = 7
#                 self.start_time = time.time()

#                 self.get_logger().info(
#                     'Sisi 4 selesai'
#                 )

#         # =========================
#         # PUTARAN 4
#         # =========================

#         elif self.state == 7:

#             msg.angular.z = self.kecepatan_putar

#             if waktu >= self.waktu_putar_4:

#                 self.state = 8

#                 self.get_logger().info(
#                     'Putaran 4 selesai'
#                 )

#                 self.get_logger().info(
#                     'Persegi panjang selesai'
#                 )

#         # =========================
#         # BERHENTI
#         # =========================

#         elif self.state == 8:

#             msg.linear.x = 0.0
#             msg.angular.z = 0.0

#             self.publisher_.publish(msg)

#             self.timer.cancel()

#             return

#         self.publisher_.publish(msg)


# def main(args=None):

#     rclpy.init(args=args)

#     node = MoverNode()

#     try:

#         rclpy.spin(node)

#     except KeyboardInterrupt:

#         pass

#     finally:

#         # Pastikan robot berhenti
#         msg = Twist()

#         node.publisher_.publish(msg)

#         node.destroy_node()

#         if rclpy.ok():

#             rclpy.shutdown()


# if __name__ == '__main__':
#     main()