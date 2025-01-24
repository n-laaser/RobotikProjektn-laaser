import rclpy
import rclpy.executors
import rclpy.node
import cv2
import numpy as np

from stopper import Stopper
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

class LaserTurn(rclpy.node.Node):
    def __init__(self):
        super().__init__('laserturn')

        # definition of the parameters that can be changed at runtime
        self.declare_parameter('distance_to_turn', 0.5)
        self.declare_parameter('speed_drive', 0.15)
        self.declare_parameter('speed_turn', 0.5)
        self.declare_parameter('laser_front', 0)
        self.declare_parameter('turn_time', 2) # must ideally equal to an integer when divided by timer_period 

        # definition of the QoS in order to receive data despite WiFi
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        # create subscribers for laser scan data with changed qos
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scanner_callback,
            qos_profile=qos_policy)
        self.subscription  # prevent unused variable warning

        # create publisher for driving commands
        self.publisher_laserturn = self.create_publisher(Twist, 'laser', 1)
        self.is_turning = False
        # create timer to periodically invoke the driving logic
        self.timer_period = 0.5  # seconds
        self.counter = 0
        self.turn_time = round(self.get_parameter('turn_time').get_parameter_value().integer_value / self.timer_period)
        self.my_timer = self.create_timer(self.timer_period, self.timer_callback)
    
    # handling received laser scan data
    def scanner_callback(self, msg):

        # saving the required sensor value, no further processing at this point
        self.front_distance = msg.ranges[self.get_parameter('laser_front').get_parameter_value().integer_value]
        
    # driving logics
    def timer_callback(self):

        turn_dist = self.get_parameter('distance_to_turn').get_parameter_value().double_value
        turn_speed = self.get_parameter('speed_turn').get_parameter_value().double_value
        #drive_speed = self.get_parameter('speed_drive').get_parameter_value().double_value
        speed = 0.0
        if self.counter <= 0:
            self.is_turning = False
        # no or far away obstacle
        if(self.front_distance <= turn_dist and self.front_distance!=0):
            turn = turn_speed
            self.is_turning = True
            self.counter = self.turn_time 
            
        elif (self.front_distance > turn_dist or self.front_distance == 0) and self.is_turning==False:
            turn = 0.0
            self.is_turning = False

        # create message
        self.counter -= 1
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = turn

        # send message
        self.publisher_laserturn.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaserTurn()

    try:
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        stop = Stopper()

    finally:
        LaserTurn.destroy_node()
        stop.destroy_node()
        rclpy.shutdown()
        print('Shutting Down LaserTurn')
