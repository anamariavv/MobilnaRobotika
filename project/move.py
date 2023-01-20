import rclpy
import math

from rclpy.node import Node
from math import atan2
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2
import cv_bridge
from cv_bridge import CvBridge

Kp = 0.9
MAX_ANGLE_DIFF = 0.3
DIST_STOP = 0.3
LINEAR_VEL_GREEN = 0.5
LINEAR_VEL_YELLOW = 0.2
ANGULAR_IDLE = 0.3
RED = "Red"
GREEN = "Green"
YELLOW = "Yellow"

#Utility funkcija
def euler_from_quaternion(q):
        x = q.x
        y = q.y
        z = q.z
        w = q.w

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z

class Move(Node):
    def __init__(self):
        super().__init__('move')

        self.target_exists = False
        self.target_color = None

        self.image_subscriber = self.create_subscription(
            Image, 
            "/image_publisher", 
            self.process_image,
            10)

        self.vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

    def calculate_distance_angle(self):
        #TODO napisati računanje udaljenosti i kuta do predmeta
        return True

    def set_linear(self):
        if self.target_color == RED:
            return 0.0
        elif self.target_color == YELLOW:
            return LINEAR_VEL_YELLOW
        else:
            return LINEAR_VEL_GREEN  

    def move_robot(self, dist, angle):
    
        linear_velocity = 0.0
        angular_velocity = 0.0
       
        if abs(angle) > MAX_ANGLE_DIFF:
            linear_velocity = 0.0
            angular_velocity = Kp * (angle)
        else:
            if dist > DIST_STOP:
                linear_velocity = self.set_linear()
            elif dist <= DIST_STOP:
                linear_velocity = 0.0

        msg = Twist()
        msg.linear.x = linear_velocity
        msg.angular.z = angular_velocity
        
        self.vel_publisher.publish(msg)
    
    def idle_robot(self):
        msg = Twist()
        msg.linear.x = 0
        msg.linear.z = ANGULAR_IDLE

        self.vel_publisher.publish(msg)
    
    def process_image(self, img):
        self.get_logger().info('Received image message!')
        if cv2.__version__.startswith('2.'):
            detector = cv2.SimpleBlobDetector()
        else:
            detector = cv2.SimpleBlobDetector_create()    

        frame = CvBridge().imgmsg_to_cv2(img)
        hsv_green = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_green = cv2.inRange(hsv_green,(40, 100, 20), (75, 255,255) )
        keypoint_green = detector.detect(mask_green)
        
        hsv_yellow = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_yellow = cv2.inRange(hsv_yellow,(28, 100, 20), (32, 255,255) )
        keypoint_yellow = detector.detect(mask_yellow)
        
        hsv_red = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_red = cv2.inRange(hsv_red,(0, 100, 20), (15, 255,255) )
        keypoint_red = detector.detect(mask_red)
        
        if len(keypoint_green) > 0:
            self.target_color = GREEN
            self.get_logger().info('color is green')
            self.target_exists = True   
        elif len(keypoint_yellow) > 0:
            self.target_color = YELLOW
            self.get_logger().info('color is yellow')
            self.target_exists = True
        elif len(keypoint_red) > 0:
            self.target_color = RED
            self.get_logger().info('color is red')
            self.target_exists = True
        else:
            self.target_exists = False
            self.target_color = None                     
        
        # if(self.target_exists):
        #     #pozvati calculate_distance_angle za računanje udaljenosti i kuta do predmeta
        #     self.calculate_distance_angle()
        #     #pozvati move_robot sa izračunatom udaljenosti i kutom
        #     self.move_robot()
        # else:
        #     self.idle_robot()

def main(args=None):
    rclpy.init(args=args)

    move = Move()

    rclpy.spin(move)
    move.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


       
       
