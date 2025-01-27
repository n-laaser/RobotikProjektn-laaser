import rclpy
import rclpy.executors
import rclpy.node

from two_drive.stopper import Stopper
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
        self.stateint = 1 # 1 is for Follow 2 is for Turn 3 is Error
        self.line_msg = Twist()
        self.laser_msg = Twist()

        #self.state = State(1)

        self.subscription_laser = self.create_subscription(Twist, 'laser', self.laser_callback, qos_profile=qos_policy)
        self.subscription_line = self.create_subscription(Twist, 'line', self.line_callback, qos_profile=qos_policy)

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)

        timer_period = 0.05
        self.my_timer = self.create_timer(timer_period, self.timer_callback )

    def laser_callback(self, msg):

        if(msg.angular.z == 0.0):
            #self.state = State.FollowLine
            self.stateint = 1
        elif(msg.angular.z != 0.0):
            #self.state = State.TurnOnObject
            self.stateint = 2
        else:
            #self.state = State.Error
            self.stateint = 3
        self.laser_msg = msg

    def line_callback(self, msg):

        self.line_msg = msg

    def timer_callback(self):
        #self.get_logger().info("DriverLogic")
        if(self.stateint == 1):
            #self.get_logger().info("using LineMsg")
            msg = self.line_msg
        elif(self.stateint == 2):
            #self.get_logger().info("using LaserMsg")
            msg = self.laser_msg	
        else:
            self.get_logger().info('Error State entered in Driving Logic')
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        self.get_logger().info(f'Speed: {msg.linear.x} Turn: {msg.angular.z}')
        self.publisher_.publish(msg)

def main(args=None):

    print('Hi from NodeMaster')
    rclpy.init(args=args, signal_handler_options=rclpy.SignalHandlerOptions.NO)
    driver_node = Driver()
    
    try:
        rclpy.spin(driver_node)

    except KeyboardInterrupt:
        driver_node.destroy_node()
        stop = Stopper()

    finally:
        driver_node.destroy_node()
        stop.destroy_node()
        rclpy.shutdown()
        print('Shutting Down NodeMaster')


if __name__ == '__main__':
    main()
