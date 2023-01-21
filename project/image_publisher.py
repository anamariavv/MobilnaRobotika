import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import String

TIMER_PERIOD=0.5

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')

        self.cap = cv2.VideoCapture(0)
        self.br = CvBridge()

        self.publisher_ = self.create_publisher(
            Image,
            'image_publisher',
            1)
        self.timer = self.create_timer(
            TIMER_PERIOD,
            self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()

        if ret == True:
            self.publisher_.publish(self.br.cv2_to_imgmsg(frame))

            self.get_logger().info('Publishing image')

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
