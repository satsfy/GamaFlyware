#!/usr/bin/env python3
import sys
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageSubscriber(Node):
    def __init__(self, topic):
        super().__init__('image_subscriber')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            topic,
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info(f"Subscribed to {topic}")

    def listener_callback(self, msg):
        try:
            # Convert ROS image message to OpenCV image (BGR8)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        # Display the image using OpenCV
        cv2.imshow("Camera Frame", cv_image)
        # Wait 1ms; necessary to update the window.
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    # Default topic is /mavros/camera/image_captured; override via command line if provided.
    topic = "/camera"
    if len(sys.argv) > 1:
        topic = sys.argv[1]
    node = ImageSubscriber(topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
