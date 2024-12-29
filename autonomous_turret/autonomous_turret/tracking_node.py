#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import cv2
import numpy as np


class ColorWindowNode(Node):
    def __init__(self):
        super().__init__('color_window_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/main/scan/sim',
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(Bool, '/collision_detected', 10)
        self.hit_detected = False
        self.create_timer(0.1, self.update_window)

    def listener_callback(self, msg: LaserScan):
        self.hit_detected = any(distance < 5.0 for distance in msg.ranges)
        self.publish_collision_status()

    def publish_collision_status(self):
        msg = Bool()
        msg.data = self.hit_detected
        self.publisher.publish(msg)

    def update_window(self):
        color = (0, 255, 0) if not self.hit_detected else (0, 0, 255)
        image = np.zeros((500, 500, 3), dtype=np.uint8)
        image[:] = color
        cv2.imshow("Color Window", image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ColorWindowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
