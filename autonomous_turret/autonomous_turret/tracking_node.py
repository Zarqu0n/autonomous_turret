import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class TrackingNode(Node):
    def __init__(self):
        super().__init__('tracking_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.get_logger().info('Tracking Node Initialized')

    def image_callback(self, msg):
        try:
            # Görüntüyü OpenCV formatına çevir
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Görüntü işleme algoritmaları (örnek)
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            _, thresh = cv2.threshold(gray, 128, 255, cv2.THRESH_BINARY)

            # Hedef tespiti
            # (Buraya özel hedef algılama algoritmanızı yazabilirsiniz.)
            
            # Taret hareketi için kontrol komutu oluştur
            twist = Twist()
            twist.linear.x = 0.0  # Hedefe göre ayarlayın
            twist.angular.z = 0.1  # Hedefe göre ayarlayın
            
            self.publisher_.publish(twist)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = TrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
