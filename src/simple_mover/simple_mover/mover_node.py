import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class InverseKinematics(Node):

    def __init__(self):
        super().__init__('inverse_kinematics')

        self.wheel_radius = 0.03
        self.wheel_separation = 0.17

        # Input dari /cmd_vel
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.velocity_callback,
            10
        )

        # Output ke roda kiri
        self.left_publisher = self.create_publisher(
            Float64,
            '/left_wheel/command',
            10
        )

        # Output ke roda kanan
        self.right_publisher = self.create_publisher(
            Float64,
            '/right_wheel/command',
            10
        )

        self.get_logger().info('==========================================')
        self.get_logger().info('        INVERSE KINEMATICS')
        self.get_logger().info('==========================================')
        self.get_logger().info(
            f'Wheel radius     : {self.wheel_radius:.3f} m'
        )
        self.get_logger().info(
            f'Wheel separation : {self.wheel_separation:.3f} m'
        )
        self.get_logger().info('Input  : /cmd_vel')
        self.get_logger().info('Left   : /left_wheel/command')
        self.get_logger().info('Right  : /right_wheel/command')
        self.get_logger().info('==========================================')

    def velocity_callback(self, msg):

        v = msg.linear.x
        omega = msg.angular.z

        # Differential drive inverse kinematics
        v_left = v - (self.wheel_separation / 2.0) * omega
        v_right = v + (self.wheel_separation / 2.0) * omega

        omega_left = v_left / self.wheel_radius
        omega_right = v_right / self.wheel_radius

        # Publish left wheel velocity
        left_msg = Float64()
        left_msg.data = omega_left
        self.left_publisher.publish(left_msg)

        # Publish right wheel velocity
        right_msg = Float64()
        right_msg.data = omega_right
        self.right_publisher.publish(right_msg)

        self.get_logger().info(
            f'Linear: {v:.3f} m/s | '
            f'Angular: {omega:.3f} rad/s | '
            f'Left: {omega_left:.3f} rad/s | '
            f'Right: {omega_right:.3f} rad/s'
        )


def main(args=None):

    rclpy.init(args=args)

    node = InverseKinematics()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('Program dihentikan.')

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
