import rclpy
import math

from rclpy.node import Node
from math import atan2
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
import cv_bridge
import numpy
from cv_bridge import CvBridge

CAMERA_U = 162
CAMERA_F = 290

SPEED_RED = 0.2
SPEED_BLUE = 0.2

TIMER_PERIOD = 2

class Move(Node):
    def __init__(self):
        super().__init__('move')

        self.target_exists = False
        self.target_color = None
        self.is_busy = False

        self.speed_linear = 0.0
        self.speed_angular = 0.0
        
        self.image_subscriber = self.create_subscription(
            Image, 
            "/image_publisher", 
            self.process_image,
            1
        )

        self.vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            1
        )

        self.timer_red = self.create_timer(
            TIMER_PERIOD,
            self.timer_red_callback)

        self.timer_red.cancel()

        self.timer_blue = self.create_timer(
            TIMER_PERIOD,
            self.timer_blue_callback)

        self.timer_blue.cancel()

    def calculate_angular_velocity(self, target_center):
        angle_to_target = (CAMERA_U - target_center) / CAMERA_F 
        angular_velocity = numpy.arctan(angle_to_target)

        return angular_velocity
   
    def process_image(self, img):

        if cv2.__version__.startswith('2.'):
            detector = cv2.SimpleBlobDetector()
        else:
            detector = cv2.SimpleBlobDetector_create()    

        frame = CvBridge().imgmsg_to_cv2(img)
        hsv_blue = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_blue = cv2.inRange(hsv_blue,(95, 125, 20), (130, 255, 255) )
        keypoint_blue = detector.detect(mask_blue)
        
        hsv_yellow = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_yellow = cv2.inRange(hsv_yellow,(28, 100, 20), (32, 255,255) )
        keypoint_yellow = detector.detect(mask_yellow)
        
        hsv_red = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_red = cv2.inRange(hsv_red,(0, 100, 20), (15, 255,255) )
        keypoint_red = detector.detect(mask_red)

        targetMask = None
        
        if len(keypoint_blue) > 0:
            self.get_logger().info('color is blue')
            self.rotate()
        elif len(keypoint_yellow) > 0:
            self.get_logger().info('color is yellow')
            self.target_exists = True
            targetMask = mask_yellow
        elif len(keypoint_red) > 0:
            self.get_logger().info('color is red')
            self.goForward()
        else:
            self.target_exists = False
                    
        if(self.target_exists):
            targetCenter = cv2.moments(targetMask)
            angular_velocity = None

            if targetCenter["m00"] != 0:
                targetCenter = int(targetCenter["m10"] / targetCenter["m00"])
                angular_velocity = self.calculate_angular_velocity(targetCenter)
                self.speed_angular = angular_velocity
                self.speed_linear = 0.0

        else:
            if(self.is_busy == False):
                self.speed_angular = 0.0

        msg = Twist()
        msg.linear.x = self.speed_linear
        msg.angular.z = self.speed_angular
        self.get_logger().info(f'Twist message is {msg}!')
        self.vel_publisher.publish(msg)

    def timer_red_callback(self):
        self.speed_angular = 0.0
        self.speed_linear = 0.0
        self.get_logger().info('STOP! red')
        self.timer_red.cancel()
        self.is_busy = False

    def timer_blue_callback(self):
        self.speed_angular = 0.0
        self.speed_linear = 0.0
        self.get_logger().info('STOP! blue')
        self.timer_blue.cancel()
        self.is_busy = False

    def goForward(self):
        if (self.is_busy):
            return

        self.is_busy = True

        self.speed_linear = SPEED_RED

        self.timer_red.reset()
        self.get_logger().info('GO FORWARD! Crvena')

    def rotate(self):
        if (self.is_busy):
            return

        self.is_busy = True
        self.speed_angular = SPEED_BLUE

        self.timer_blue.reset()
        self.get_logger().info('START ROTATING! Plava')

def main(args=None):
    rclpy.init(args=args)

    move = Move()

    rclpy.spin(move)
    move.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()