import linefollow as LineFollow
import laserturn as LaserTurn
import rclpy
import rclpy.executors
import rclpy.node

from stopper import Stopper
from geometry_msgs.msg import Twist
from enum import Enum

class State(Enum):
    FollowLine = 1
    TurnOnObject = 2
    Error = 3

class Driver(rclpy.node.Node):
    def __init__(self):
        super().__init__('driver')

        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
                
        self.line_msg = 0
        self.laser_msg = 0

        self.state = State()

        self.subscription_laser = self.create_subscription(Twist, 'laser', self.laser_callback, qos_profile=qos_policy)
        self.subscription_line = self.create_subscription(Twist, 'line', self.line_callback, qos_profile=qos_policy)

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)

        timer_period = 0.1
        self.my_timer = self.create_timer(timer_period, self.timer_callback )

    def laser_callback(self, msg):

        if(msg.angular.x == 0.0):
            self.state = State.FollowLine
        elif(msg.angular.x != 0.0):
            self.state = State.TurnOnObject
        else:
            self.state = State.Error
        self.laser_msg = msg

    def line_callback(self, msg):

        self.line_msg = msg

    def timer_callback(self):

        if(self.state == 'FollowLine'):
            msg = self.line_msg
        elif(self.state == 'TurnOnObject'):
            msg = self.laser_msg
        else:
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.x = 0.0
        
        self.publisher_.publish(msg)

def main(args=None):

    print('Hi from NodeMaster')
    rclpy.init(args=args, signal_handler_options=rclpy.SignalHandlerOptions.NO)
    laser_turn_node = LaserTurn()
    line_follow_node = LineFollow()
    driver_node = Driver()
    
    try:
        rclpy.spin(driver_node)

    except KeyboardInterrupt:
        stop = Stopper()

    finally:
        driver_node.destroy_node()
        stop.destroy_node()
        rclpy.shutdown()
        print('Shutting Down NodeMaster')


if __name__ == '__main__':
    main()