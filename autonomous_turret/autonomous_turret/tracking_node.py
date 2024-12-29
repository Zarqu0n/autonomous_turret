#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from playsound import playsound
import os

class CollisionSoundNode(Node):
    def __init__(self):
        super().__init__('collision_sound_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/main/scan/sim',
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(Bool, '/collision_detected', 10)
        self.hit_detected = False
        self.sound_playing = False  # Prevent overlapping sound plays
        self.sound_path = self.get_sound_path()

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

    def listener_callback(self, msg: LaserScan):
        self.hit_detected = any(distance < 5.0 for distance in msg.ranges)
        self.publish_collision_status()
        if self.hit_detected and not self.sound_playing:
            self.sound_playing = True
            self.play_sound()

    def publish_collision_status(self):
        msg = Bool()
        msg.data = self.hit_detected
        self.publisher.publish(msg)

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
    node = CollisionSoundNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
