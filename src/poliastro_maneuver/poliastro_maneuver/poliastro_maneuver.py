import rclpy
from rclpy.node import Node
import time
import numpy as np

from . import util_maneuver as maneuver

from std_msgs.msg import Float64MultiArray, Float64

class poliastro_maneuver(Node):

    def __init__(self):
        super().__init__('dynamics')
        
        self.state_pub = self.create_publisher(Float64MultiArray , '/poliastro_maneuver/state', 10)

        self.get_logger().info(f"Starting poliastro_maneuver version = 0.0.0")

        #Defining states
        #Initial orbit parameters
        self.declare_parameter('r_init', 100.0) #Initial orbit altitude
        self.declare_parameter('r_final',1000.0) #Target orbit altitude

        self.declare_parameter('publish_freq', 10.0)   

        self.r_init = self.get_parameter('r_init').get_parameter_value().double_value
        self.r_final = self.get_parameter('r_final').get_parameter_value().double_value

        # Defining encounter for publisher
        self.i = 0        

        #################################
        # Calling simulation function using parameters declared above
        self.res_orb =  maneuver.run(self, 
                                      r_init = self.r_init,
                                      r_final = self.r_final)
        
        self.state_msg = Float64MultiArray()
        timer_period = 1/self.get_parameter('publish_freq').get_parameter_value().double_value  # frequency of publishing
        self.timer = self.create_timer(timer_period, self.timer_callback)
        

    def timer_callback(self):

        self.state_msg.data = [self.res_orb[self.i][0], self.res_orb[self.i][1], self.res_orb[self.i][2]]

        self.state_pub.publish(self.state_msg)
        self.get_logger().info(f"Publishing = {self.state_msg.data}")

        self.i += 1
        if self.i==len(self.res_orb):
            self.get_logger().info('All data published successfully')
            exit()
        
        
def main(args=None):
    rclpy.init(args=args)
    dynamics = poliastro_maneuver()
    rclpy.spin(dynamics)
    dynamics.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()