import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class CarlaLocalization(Node):
    def __init__(self):
        super().__init__('carla_localization')

        self.declare_parameter('topic_name', '/localization/kinematic_state')
        self.topic_name = self.get_parameter(
            'topic_name').get_parameter_value().string_value

        self.message_count = 0
        self.last_log_time = self.get_clock().now()

        self.localization_publisher = self.create_publisher(
            Odometry,
            '/ub_mr/localization',
            10,
        )
        self.kinematic_state_subscription = self.create_subscription(
            Odometry,
            self.topic_name,
            self.on_kinematic_state_received,
            10,
        )

        self.get_logger().info(
            f'Forwarding {self.topic_name} directly to /ub_mr/localization'
        )

    def on_kinematic_state_received(self, msg: Odometry):
        self.message_count += 1

        localization_msg = Odometry()
        localization_msg.header = msg.header
        localization_msg.header.frame_id = 'odom'
        localization_msg.child_frame_id = msg.child_frame_id or 'base_link'
        localization_msg.pose = msg.pose
        localization_msg.twist = msg.twist

        self.localization_publisher.publish(localization_msg)

        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds < 1_000_000_000:
            return

        self.last_log_time = now
        position = msg.pose.pose.position

        self.get_logger().info(
            'Forwarded kinematic state '
            f'#{self.message_count}: '
            f'position=({position.x:.3f}, {position.y:.3f}, {position.z:.3f})'
        )


def main(args=None):
    print('Starting Carla Localization Node')
    rclpy.init(args=args)
    carla_localization = CarlaLocalization()
    rclpy.spin(carla_localization)
    carla_localization.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
