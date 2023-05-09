import rclpy
from rclpy.node import Node
import time
import numpy as np

from . import util_atmo_drag as atmo_drag

from std_msgs.msg import Float64MultiArray, Float64

class poliastro_atmo_drag(Node):

    def __init__(self):
        super().__init__('dynamics')
        
        self.state_pub = self.create_publisher(Float64MultiArray , '/poliastro_atmo_drag/state', 10)
        self.res_pub = self.create_publisher(Float64 , '/poliastro_atmo_drag/res', 10)

        self.get_logger().info(f"Starting poliastro_atmo_drag version = 0.0.0")

        #Defining states  a, ecc, inc, raan, argp, nu, c_d, t_limit 6.3781366e6
        #Initial orbit parameters
        self.declare_parameter('earth_r', 6378136.6)
        self.declare_parameter('a', 6600.85876692) 
        self.declare_parameter('ecc', 0.0005480) 
        self.declare_parameter('inc', 51.6399) 
        self.declare_parameter('raan', 142.7584) 
        self.declare_parameter('argp', 17.8906) 
        self.declare_parameter('nu', 0.81325021) 
        self.declare_parameter('c_d', 2.2) 

        self.declare_parameter('t_limit', 1000.0)

        self.declare_parameter('publish_freq', 10.0)   

        self.earth_r = self.get_parameter('earth_r').get_parameter_value().double_value
        self.a = self.get_parameter('a').get_parameter_value().double_value
        self.ecc = self.get_parameter('ecc').get_parameter_value().double_value
        self.inc = self.get_parameter('inc').get_parameter_value().double_value
        self.raan = self.get_parameter('raan').get_parameter_value().double_value
        self.argp = self.get_parameter('argp').get_parameter_value().double_value
        self.nu = self.get_parameter('nu').get_parameter_value().double_value
        self.c_d = self.get_parameter('c_d').get_parameter_value().double_value
        
        self.t_limit = self.get_parameter('t_limit').get_parameter_value().double_value

        # Defining encounter for publisher
        self.i = 0        

        #################################
        # Calling simulation function using parameters and functions declared above
        self.res_orb, self.res_t =  atmo_drag.run(self, 
                                                  earth_r=self.earth_r, 
                                                  a=self.a, 
                                                  ecc=self.ecc, 
                                                  inc=self.inc, 
                                                  raan=self.raan,
                                                  argp=self.argp,
                                                  nu=self.nu,
                                                  c_d=self.c_d, 
                                                  t_limit=self.t_limit)
        
        self.state_msg = Float64MultiArray()
        self.res_msg = Float64()
        timer_period = 1/self.get_parameter('publish_freq').get_parameter_value().double_value  # frequency of publishing
        self.timer = self.create_timer(timer_period, self.timer_callback)
        

    def timer_callback(self):

        self.state_msg.data = [self.res_orb[self.i][0], self.res_orb[self.i][1], self.res_orb[self.i][2]]

        self.state_pub.publish(self.state_msg)
        self.get_logger().info(f"Publishing = {self.state_msg.data}")

        self.i += 1
        if self.i==len(self.res_orb):
            self.res_msg.data = self.res_t
            self.res_pub.publish(self.res_msg)
            self.get_logger().info(f"Publishing res_t = {self.res_t}")
            self.get_logger().info('All data published successfully')
            exit()
        
        
def main(args=None):
    rclpy.init(args=args)
    dynamics = poliastro_atmo_drag()
    rclpy.spin(dynamics)
    dynamics.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()