import rclpy
from nav_msgs.msg import Odometry
from pyproj import Transformer, database
from pyproj.aoi import AreaOfInterest
from rclpy.node import Node

ORIGIN_LATITUDE = 0.0
ORIGIN_LONGITUDE = 0.0
MGRS_GRID_SIZE_METERS = 100000.0


class KinematicStateReceiver(Node):
    def __init__(self):
        super().__init__('kinematic_state_receiver')

        self.declare_parameter('topic_name', '/localization/kinematic_state')
        self.topic_name = self.get_parameter(
            'topic_name').get_parameter_value().string_value

        self.origin_latitude = ORIGIN_LATITUDE
        self.origin_longitude = ORIGIN_LONGITUDE
        self.transformer, self.map_origin_utm_epsg = self.build_utm_transformer(
            self.origin_latitude,
            self.origin_longitude,
        )
        _, self.map_origin_mgrs_x, self.map_origin_mgrs_y = (
            self.lat_lon_to_mgrs_offset(
                self.origin_latitude,
                self.origin_longitude,
            )
        )
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
            f'Listening for kinematic state messages on {self.topic_name} '
            f'with map origin latitude={self.origin_latitude:.6f}, '
            f'longitude={self.origin_longitude:.6f}, '
            f'utm_epsg={self.map_origin_utm_epsg}, '
            f'mgrs_offset=({self.map_origin_mgrs_x:.3f}, '
            f'{self.map_origin_mgrs_y:.3f})'
        )

    def on_kinematic_state_received(self, msg: Odometry):
        self.message_count += 1

        vehicle_mgrs_x = msg.pose.pose.position.x
        vehicle_mgrs_y = msg.pose.pose.position.y
        offset_x_meters = vehicle_mgrs_x - self.map_origin_mgrs_x
        offset_y_meters = vehicle_mgrs_y - self.map_origin_mgrs_y

        localization_msg = Odometry()
        localization_msg.header = msg.header
        localization_msg.header.frame_id = 'odom'
        localization_msg.child_frame_id = msg.child_frame_id or 'base_link'
        localization_msg.pose.pose = msg.pose.pose
        localization_msg.pose.pose.position.x = offset_x_meters
        localization_msg.pose.pose.position.y = offset_y_meters
        localization_msg.twist = msg.twist
        localization_msg.pose.covariance = msg.pose.covariance
        localization_msg.twist.covariance = msg.twist.covariance

        self.localization_publisher.publish(localization_msg)

        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds < 1_000_000_000:
            return

        self.last_log_time = now

        position = msg.pose.pose.position
        linear_velocity = msg.twist.twist.linear

        self.get_logger().info(
            'Received kinematic state '
            f'#{self.message_count}: '
            f'vehicle_mgrs_offset=({vehicle_mgrs_x:.3f}, {vehicle_mgrs_y:.3f}), '
            f'map_origin_mgrs_offset=({self.map_origin_mgrs_x:.3f}, '
            f'{self.map_origin_mgrs_y:.3f}), '
            f'offset=({offset_x_meters:.3f}, {offset_y_meters:.3f}, {position.z:.3f}), '
            f'linear_velocity=({linear_velocity.x:.3f}, '
            f'{linear_velocity.y:.3f}, {linear_velocity.z:.3f})'
        )

    def lat_lon_to_mgrs_offset(self, latitude: float, longitude: float):
        easting_meters, northing_meters = self.transformer.transform(
            longitude,
            latitude,
        )

        mgrs_x = easting_meters % MGRS_GRID_SIZE_METERS
        mgrs_y = northing_meters % MGRS_GRID_SIZE_METERS

        return self.map_origin_utm_epsg, mgrs_x, mgrs_y

    def build_utm_transformer(self, latitude: float, longitude: float):
        utm_crs_info = database.query_utm_crs_info(
            datum_name='WGS 84',
            area_of_interest=AreaOfInterest(
                west_lon_degree=longitude,
                south_lat_degree=latitude,
                east_lon_degree=longitude,
                north_lat_degree=latitude,
            ),
        )
        if not utm_crs_info:
            raise RuntimeError(
                f'Unable to determine a UTM CRS for lat={latitude}, lon={longitude}.'
            )

        utm_crs = utm_crs_info[0]
        transformer = Transformer.from_crs(
            'EPSG:4326',
            f'EPSG:{utm_crs.code}',
            always_xy=True,
        )

        return transformer, utm_crs.code


def main(args=None):
    print('Starting Kinematic State Receiver Node')
    rclpy.init(args=args)
    kinematic_state_receiver = KinematicStateReceiver()
    rclpy.spin(kinematic_state_receiver)
    kinematic_state_receiver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
