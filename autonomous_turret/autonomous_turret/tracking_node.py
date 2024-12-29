#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import Bool
from playsound import playsound
import os
import cv2
import numpy as np
from cv_bridge import CvBridge

class RedColorSoundNode(Node):
    def __init__(self):
        super().__init__('red_color_sound_node')
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )
        self.sound_playing = False  # Prevent overlapping sound plays
        self.sound_path = self.get_sound_path()
        self.bridge = CvBridge()

    def get_sound_path(self):
        package_name = 'autonomous_turret'
        sound_file = 'resource/buzzer.mp3'
        package_share_directory = self.get_package_share_directory(package_name)
        return os.path.join(package_share_directory, sound_file)

    def get_package_share_directory(self, package_name):
        from ament_index_python.packages import get_package_share_directory
        try:
            return get_package_share_directory(package_name)
        except Exception as e:
            self.get_logger().error(f"Could not find package {package_name}: {e}")
            return ''

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            red_pixels = self.count_red_pixels(cv_image)
            total_pixels = cv_image.shape[0] * cv_image.shape[1]
            red_percentage = (red_pixels / total_pixels) * 100
            self.get_logger().info(f"Red percentage: {red_percentage}")
            if red_percentage > 5 and not self.sound_playing:
                self.sound_playing = True
                self.play_sound()
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def count_red_pixels(self, image):
        red_lower = np.array([0, 0, 50])  # Lower bound for red in BGR
        red_upper = np.array([50, 50, 255])  # Upper bound for red in BGR
        red_mask = cv2.inRange(image, red_lower, red_upper)
        return cv2.countNonZero(red_mask)

    def play_sound(self):
        if not self.sound_path:
            self.get_logger().error("Sound path is invalid.")
            self.sound_playing = False
            return
        try:
            playsound(self.sound_path)
        except Exception as e:
            self.get_logger().error(f"Error playing sound: {e}")
        finally:
            self.sound_playing = False


def main(args=None):
    rclpy.init(args=args)
    red_color_node = RedColorSoundNode()
    try:
        rclpy.spin(red_color_node)
    except KeyboardInterrupt:
        red_color_node.get_logger().info('Shutting down RedColorSoundNode...')
    finally:
        red_color_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
