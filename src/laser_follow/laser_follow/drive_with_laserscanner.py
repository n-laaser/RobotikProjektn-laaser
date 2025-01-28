"""
Simple driving node that is based on the driving behavior of a simple vacuum cleaner robot: The robot turns as long as
an obstacle is detected in the defined area, otherwise it drives straight ahead. To detect obstacles, only one measurement
value is used per scan of the laser scanner.
"""

import rclpy
import rclpy.node
import numpy as np
import numpy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

#Zeit bis der Robo Stoppt
time_to_die = 100

class SimpleDriving(rclpy.node.Node):
    def __init__(self):
        
        super().__init__('drive_with_scanner')

        # definition of the parameters that can be changed at runtime
        self.declare_parameter('distance_to_stop', 0.3)
        self.declare_parameter('distance_to_turn', 0.6)
        self.declare_parameter('distance_to_slow_down', 1.0)
        self.declare_parameter('speed_fast', 0.15)
        self.declare_parameter('speed_slow', 0.1)
        self.declare_parameter('speed_turn', 0.4)

        # variable for the last sensor reading
        self.last_distance = 0.0

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
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)

        # create timer to periodically invoke the driving logic
        timer_period = 0.5  # seconds
        self.my_timer = self.create_timer(timer_period, self.timer_callback)


    # handling received laser scan data
    def scanner_callback(self, msg):
        # saving the required sensor value, no further processing at this point
        
        #Kopie vom ranges Array(gefüllt mit Distanzwerten), dabei sind alle 0 zu 100 geändert 
        #Um später Minimum leichter bestimmen zu können
        ranges_copy = msg.ranges
        i = 0
        for x in msg.ranges:
            if x == 0:
                ranges_copy[i] = 100
            i += 1

        #self.last_distance = kleinste gemessene Distanz der vorderen 60 Werte
        self.last_distance = numpy.ma.min([numpy.ma.min(ranges_copy[:30]), numpy.ma.min(ranges_copy[-30:])])
        

        #self.last_angle = Welcher der 60 vorderen Werte die kleinste Distanz hat
        self.last_angle = ranges_copy.index(self.last_distance)

        
        print(self.last_distance,self.last_angle)



    # driving logic
    def timer_callback(self):
        global time_to_die
        # caching the parameters for reasons of clarity
        distance_slow = self.get_parameter('distance_to_slow_down').get_parameter_value().double_value
        distance_turn = self.get_parameter('distance_to_turn').get_parameter_value().double_value
        distance_stop = self.get_parameter('distance_to_stop').get_parameter_value().double_value
        slow = self.get_parameter('speed_slow').get_parameter_value().double_value

        # send message
        if(time_to_die == 0):
            speed = 0.0
            turn = 0.0
            print('died')
        else:
            time_to_die -= 1

            #schnelles Fahren, wenn nichts vor dem Robo gemessen wird
            if self.last_distance > distance_turn:
                speed = self.get_parameter('speed_fast').get_parameter_value().double_value
                turn = 0.0
                print('drive fast', speed)

            # obstacle close enough to reduce speed
            elif (self.last_distance <= distance_slow) and (self.last_distance > distance_stop):
                print("slowmode")
                if self.last_angle == 0:
                    speed = slow
                    turn = 0.0
                    print('drive slow straight')
                elif self.last_angle <= 180:
                    speed = slow
                    turn = 0.2
                    print('drive slow right')
                elif self.last_angle > 180:
                    speed = slow
                    turn = -0.2
                    print('drive slow left')

            else:
                speed = 0.0
                turn = 0.0
                print('stop')

            # create message
        msg = Twist()
        msg.linear.x = speed #NOCHMAL ÄNDERN ZU speed
        msg.angular.z = turn #NOCHMAL ÄNDERN ZU turn
        
        self.publisher_.publish(msg)

        

def main(args=None):
    try:
        print('Hi from robotik_projekt.')
        rclpy.init(args=args)

        node = SimpleDriving()

        rclpy.spin(node)

        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        print("Keyboard AHHHHH")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
