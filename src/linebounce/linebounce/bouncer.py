
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

    


class LaserTurn(rclpy.node.Node):
    def __init__(self):
        super().__init__('turner')
        # definition of the parameters that can be changed at runtime
        self.declare_parameter('distance_to_turn', 0.5)
        self.declare_parameter('speed_drive', 0.15)
        self.declare_parameter('speed_turn', 0.4)
        self.declare_parameter('laser_front', 0)
        self.declare_parameter('laser_back', 180)
        # variable for the last sensor reading
        self.back_distance = 0.0
        self.active = False
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
        self.publisher_laserturn = self.create_publisher(Twist, 'cmd_vel', 1)

        # create timer to periodically invoke the driving logic
        self.timer_period = 0.5  # seconds
        #self.my_timer = self.create_timer(timer_period, self.timer_callback)
        self.my_timer = None
    
    def activate(self):
        if not self.active:
            self.active = True
            self.my_timer = self.create_timer(self.timer_period, self.timer_callback)
            self.get_logger().info("Node_turn activated")

    def deactivate(self):
        if self.active:
            self.active = False
            if self.my_timer is not None:
                self.my_timer.cancel()  # Stop the timer
                self.my_timer = None
                self.get_logger().info("Node_turn deactivated")
            else:
                self.get_logger().info("Node_turn deactivated failed")  
            


    # handling received laser scan data
    def scanner_callback(self, msg):

        # saving the required sensor value, no further processing at this point
        #self.front_distance = msg.ranges[self.get_parameter('laser_front').get_parameter_value().integer_value]
        self.back_distance = msg.ranges[self.get_parameter('laser_back').get_parameter_value().integer_value]
        


    # driving logic
    def timer_callback(self):
        if self.active == True:
            self.get_logger().info("TurnerLogik")
            turn_dist = self.get_parameter('distance_to_turn').get_parameter_value().double_value
            turn_speed = self.get_parameter('speed_turn').get_parameter_value().double_value
            drive_speed = self.get_parameter('speed_drive').get_parameter_value().double_value
            #front_dist = self.front_distance
            back_dist = self.back_distance
            # no or far away obstacle
            
            if(back_dist <= turn_dist):
                turn = 0.0
                speed= drive_speed
            else:
                turn = turn_speed
                speed = 0.0
            # create message
            msg = Twist()
            msg.linear.x = speed
            msg.angular.z = turn

            # send message
            self.publisher_laserturn.publish(msg)


class LineFollow(rclpy.node.Node):

    def __init__(self):
        super().__init__('follower')
        
        # definition of the parameters that can be changed at runtime
        self.declare_parameter('boundary_left', 200)
        self.declare_parameter('boundary_right', 440)
        self.declare_parameter('threshold_line', 100) #is now light_lim
        self.declare_parameter('speed_drive', 0.075)
        self.declare_parameter('speed_turn', 0.2)
        self.declare_parameter('light_lim', 100)
        self.declare_parameter('middle_tol', 20)
        self.declare_parameter('speed_turn_adjust',0.3)
        self.lineposition = 0
        self.active = False
        # init openCV-bridge
        self.bridge = CvBridge()
        #self.img_row = np.array([0, 64, 128, 192, 255], dtype=np.uint8) # Beispiel
        self.img_row = np.array([0],dtype=np.uint8)
        # definition of the QoS in order to receive data despite WiFi
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        self.last_spin = False # False == gegen UHrzeigersinn True==mit Uhrzeigersinn
        # create subscribers for image data with changed qos
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.scanner_callback,
            qos_profile=qos_policy)
        self.subscription  # prevent unused variable warning


        # create publisher for driving commands
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)

        # create timer to periodically invoke the driving logic
        self.timer_period = 0.2  # seconds
        #self.my_timer = self.create_timer(timer_period, self.timer_callback)
        self.my_timer = None

    def activate(self):
        if not self.active:
            self.active = True
            self.my_timer = self.create_timer(self.timer_period, self.timer_callback)
            self.get_logger().info("Node_Follow activated")

    def deactivate(self):
        if self.active:
            self.active = False
            if self.my_timer is not None:
                self.my_timer.cancel()  # Stop the timer
                self.my_timer = None
                self.get_logger().info("Node_Follow deactivated")
            else:
                self.get_logger().info("Node_Follow deactivated failed") 
    
    # handling received image data
    def scanner_callback(self, data):
        # convert message to opencv image
        img_cv = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding = 'passthrough')

        # convert image to grayscale
        img_gray = cv2.cvtColor(img_cv, cv2.COLOR_BGR2GRAY)

        # get image size
        height, width = img_gray.shape[:2]

        # get the lowest row from image
        img_row = img_gray[height-1,:]
        self.img_row = img_row 
        # show image
        cv2.imshow("IMG", img_gray)
        cv2.imshow("IMG_ROW", img_row)
        cv2.waitKey(1)


    # driving logic for linefollowing
    def timer_callback(self):
        if self.active == True:
            self.get_logger().info("FollowerLogik")
        # caching the parameters for reasons of clarity
            boundary_left = self.get_parameter('boundary_left').get_parameter_value().integer_value
            boundary_right = self.get_parameter('boundary_right').get_parameter_value().integer_value
            speed_drive = self.get_parameter('speed_drive').get_parameter_value().double_value
            speed_turn = self.get_parameter('speed_turn').get_parameter_value().double_value
            speed_turn_adjust = self.get_parameter('speed_turn_adjust').get_parameter_value().double_value
            light_lim = self.get_parameter('light_lim').get_parameter_value().integer_value
            middle_tol = self.get_parameter('middle_tol').get_parameter_value().integer_value
            last_spin = self.last_spin
            img_row_original = self.img_row
            #print(len(img_row_original))

            img_row = self.img_row[boundary_left:boundary_right]
            #20 Größten Werte nach Größe geordnet
            img_row_sorted = np.argsort(img_row)[-20:]

            # Werte der 20 größten Elemente
            largest_values = img_row[img_row_sorted]

            # Index des mittleren Wertes der 20 größten im Teil-Array
            middle_index_in_subset = np.argsort(largest_values)[len(largest_values) // 2]

            # Index des mittleren Wertes im Original-Array
            middle_index_in_original = img_row_sorted[middle_index_in_subset]

            max_avg = np.mean(img_row_sorted)
            #closest_index = img_row[np.argmin(np.abs(img_row_sorted - max_avg))]
            #closest_index = np.argmin(np.abs(np.arange(len(img_row)) - max_avg))
            closest_value = img_row[middle_index_in_original]
            middle_pix = 320
            speed = 0.0
            turn = 0.0
            brightest = max(img_row)
            #bright_pos = np.where(img_row == closest_value)[0][0]
            line_pos = middle_index_in_original + boundary_left

            self.get_logger().info(f"Hellster Wert: {brightest}")
            self.get_logger().info(f"Durchschnittlich hellster Wert: {closest_value}")
            #self.get_logger().info(f"Durchschnittlich hellster Index: {closest_index}")
            self.get_logger().info(f"Mittlerer Index der hellsten Pixel: {middle_index_in_original})")
            print(img_row)

            if(brightest < light_lim  and last_spin == False):        
                #no white in bottom line of image 
                speed= 0.0
                turn = speed_turn
            elif(brightest < light_lim and last_spin == True):
                speed= 0.0
                turn = -speed_turn
            else:
                speed=speed_drive
                #white pixel in image line / and bright_pos > boundary_left)
                if line_pos < (middle_pix) :
                    #left side is brightest: turn left '-'
                    turn = speed_turn
                    self.last_spin = True
                # and bright_pos < boundary_right)
                elif line_pos > (middle_pix) :
                    # right side right turn '+'
                    turn = -speed_turn
                    self.last_spin = False
                else :
                        # bright pixel is in the middle 
                        turn = 0.0

            # create message
            msg = Twist()
            msg.linear.x = speed
            msg.angular.z = turn

            # send message
            self.publisher_.publish(msg)

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

    print('Hi from linebounce Bouncer')
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
