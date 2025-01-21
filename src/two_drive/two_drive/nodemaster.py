import linefollow as LineFollow
import laserturn as LaserTurn
import rclpy
import rclpy.executors
import rclpy.node
import cv2
import numpy as np

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

class Driver(rclpy.node.Node):
    def __init__(self, node_follow, node_turn):
        super().__init__('driver')
        self.declare_parameter('laserscan_beam_front', 0)
        self.declare_parameter('laserscan_beam_back', 180)
        self.declare_parameter('distance_turn', 5)
        self.declare_parameter('time_to_turn', 5.0)
        self.front_distance = 0 
        self.back_distance = 0
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        self.node_follow = node_follow
        self.node_turn = node_turn
        # Zustandsvariable
        self.current_state = 'NODE_Turn'

        # create subscribers for laser scan data with changed qos
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            qos_profile=qos_policy)
        self.subscription  # prevent unused variable warning
        timer_period = 0.5  # seconds
        self.my_timer = self.create_timer(timer_period, self.timer_callback)
        # Welche Node soll anfangen?

        self.node_follow.activate()
        self.node_turn.deactivate()
        

    def laser_callback(self,msg):
        self.front_distance = msg.ranges[self.get_parameter('laserscan_beam_front').get_parameter_value().integer_value]
        #self.back_distance = msg.ranges[self.get_parameter('laserscan_beam_back').get_parameter_value().integer_value]

    def timer_callback(self):
        print('Main TimerCallback')
        turn_dist = self.get_parameter('distance_turn').get_parameter_value().integer_value
        ttt = self.get_parameter('time_to_turn').get_parameter_value().integer_value
        front_dist = self.front_distance # abstand zum nächsten objekt for dem Roboter
        if front_dist < turn_dist and front_dist!=0:  # Threshold distance in meters
            if not self.node_turn.active:
                self.get_logger().info(f"Activating LaserTurn (distance: {self.front_distance})")
                self.node_follow.deactivate()
                self.node_turn.activate()
        else:
            if not self.node_follow.active:
                self.get_logger().info(f"Activating LineFollow (distance: {self.front_distance})")
                self.node_turn.deactivate()
                self.node_follow.activate()
        #self.front_distance -= 1 
        #print(self.front_distance)

        ''' if(front_dist < turn_dist and front_dist!=0 and self.current_state == 'NODE_Follow'):
            node_follow.destroy_node()
            self.get_logger().info('Switching to NODE_Turn.')
            self.activate_node_turn()
        else: #elif front_dist >= turn_dist and self.current_state == 'NODE_Turn':
            node_turn.destroy_node()
            self.get_logger().info('Switching to NODE_Follow.')
            self.activate_node_follow() '''

class Stopper(rclpy.node.Node):
    def __init__(self):
        super().__init__('stopper')
        self.publisher_stop = self.create_publisher(Twist, 'cmd_vel', 1)
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        print('Stop msg sent')
        # send message
        self.publisher_stop.publish(msg)

def main(args=None):

    print('Hi from NodeMaster')
    rclpy.init(args=args, signal_handler_options=rclpy.SignalHandlerOptions.NO)
    laser_turn_node = LaserTurn()
    line_follow_node = LineFollow()
    driver_node = Driver(line_follow_node, laser_turn_node)
    
    # Use a single executor to manage all nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(driver_node)
    executor.add_node(laser_turn_node)
    executor.add_node(line_follow_node)
    try:
        rclpy.spin(line_follow_node)
        #executor.spin()
        
    except KeyboardInterrupt:
        stop = Stopper()
        #print('except')
    finally:
        stop = Stopper()
        driver_node.destroy_node()
        Stopper().destroy_node()
        rclpy.shutdown()
        print('Shutting Down')


if __name__ == '__main__':
    main()