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
from std_msgs.msg import String

Kp = 0.9
MAX_ANGLE_DIFF = 0.3
DIST_STOP = 0.3
LINEAR_VEL_BLUE = 0.5
LINEAR_VEL_YELLOW = 0.2
ANGULAR_IDLE = 0.3
RED = "Red"
BLUE = "Blue"
YELLOW = "Yellow"
CAMERA_U = 160.5
CAMERA_F = 290

SPEED_RED = 0.2
SPEED_BLUE = 0.2

TIMER_PERIOD=2

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

        self.timer = self.create_timer(
            TIMER_PERIOD,
            self.timer_callback)

        self.timer.cancel()

        self.timer_plava = self.create_timer(
            TIMER_PERIOD,
            self.timer_callback_angular)

        self.timer_plava.cancel()

    def set_linear(self):
        if self.target_color == RED:
            return 0.0
        elif self.target_color == YELLOW:
            return LINEAR_VEL_YELLOW
        else:
            return LINEAR_VEL_BLUE

    def move_robot(self, angular_velocity):
        linear_velocity = self.set_linear()

        msg = Twist()
        msg.linear.x = linear_velocity
        msg.angular.z = angular_velocity
        
        self.get_logger().info(f'Twist message is {msg}!')

        self.vel_publisher.publish(msg)
    
    def idle_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.z = ANGULAR_IDLE

        self.vel_publisher.publish(msg)

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
        hsv_BLUE = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_BLUE = cv2.inRange(hsv_BLUE,(95, 125, 20), (130, 255, 255) ) # 40, 100, 20, , 75, 255, 255 # (50, 150, 20), (70, 255,255)
        keypoint_BLUE = detector.detect(mask_BLUE)
        
        hsv_yellow = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_yellow = cv2.inRange(hsv_yellow,(28, 100, 20), (32, 255,255) )
        keypoint_yellow = detector.detect(mask_yellow)
        
        hsv_red = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_red = cv2.inRange(hsv_red,(0, 100, 20), (15, 255,255) )
        keypoint_red = detector.detect(mask_red)

        targetMask = None
        
        if len(keypoint_BLUE) > 0:
            self.target_color = BLUE
            self.get_logger().info('color is blue')
            self.rotate()
        elif len(keypoint_yellow) > 0:
            self.target_color = YELLOW
            self.get_logger().info('color is yellow')
            self.target_exists = True
            targetMask = mask_yellow

        elif len(keypoint_red) > 0:
            self.target_color = RED
            self.get_logger().info('color is red')
            targetMask = mask_red
            self.goForward()
        else:
            self.target_exists = False
            self.target_color = None                     
                    
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

    def timer_callback(self):
        self.speed_angular = 0.0
        self.speed_linear = 0.0
        self.get_logger().info('Crvena stani!')
        self.timer.cancel()
        self.is_busy = False
        return

    def timer_callback_angular(self):
        self.speed_angular = 0.0
        self.speed_linear = 0.0
        self.get_logger().info('Plava stani!')
        self.timer_plava.cancel()
        self.is_busy = False
        return

    def goForward(self):
        if (self.is_busy):
            return

        self.is_busy = True

        self.speed_linear = SPEED_RED

        self.timer.reset()
        self.get_logger().info('Kreni naprijed!')

        return

    def rotate(self):
        if (self.is_busy):
            return

        self.is_busy = True

        self.speed_angular = SPEED_BLUE

        self.timer_plava.reset()
        self.get_logger().info('Kreni se rotirati!')

        return

def main(args=None):
    rclpy.init(args=args)

    move = Move()

    rclpy.spin(move)
    move.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()