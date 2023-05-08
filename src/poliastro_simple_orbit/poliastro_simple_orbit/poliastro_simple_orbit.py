import rclpy
from rclpy.node import Node
import time
import numpy as np

from . import util_simple_orbit as atmo_drag

from std_msgs.msg import Float64MultiArray, Float64

class poliastro_simple_orbit(Node):

    def __init__(self):
        super().__init__('dynamics')
        
        self.state_pub = self.create_publisher(Float64MultiArray , '/poliastro_simple_orbit/state', 10)

        self.get_logger().info(f"Starting poliastro_simple_orbit version = 0.0.0")

        #Defining states
        #Initial orbit parameters
        self.declare_parameter('apo_r', 10000.0)
        self.declare_parameter('peri_r',10000.0) 
        self.declare_parameter('start_t', "2022-01-01 00:00") # Time must be in format 'YYYY-MM-DD HH:MM'
        self.declare_parameter('finish_t', "2022-01-01 02:00") 

        self.declare_parameter('publish_freq', 10.0)   

        self.apo_r = self.get_parameter('apo_r').get_parameter_value().double_value
        self.peri_r = self.get_parameter('peri_r').get_parameter_value().double_value
        self.start_t = self.get_parameter('start_t').get_parameter_value().string_value
        self.finish_t = self.get_parameter('finish_t').get_parameter_value().string_value

        # Defining encounter for publisher
        self.i = 0        

        #################################
        # Calling simulation function using parameters declared above
        self.res_orb =  atmo_drag.run(self, 
                                      apo_r = self.apo_r,
                                      peri_r = self.peri_r,
                                      start_t = self.start_t,
                                      finish_t = self.finish_t)
        
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
    dynamics = poliastro_simple_orbit()
    rclpy.spin(dynamics)
    dynamics.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()