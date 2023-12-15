
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from irobot_create_msgs.msg import IrIntensityVector
from rclpy.qos import ReliabilityPolicy, QoSProfile
import sys
import math

class IR(Node):
    def __init__(self):
        super().__init__("Infrared_Node")
        #self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        #self.subscription = self.create_subscription(IrIntensityVector,'/ir_intensity', self.ir_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.timer = self.create_timer(0.1, self.check_hazard)
        self.ir_intensity = IrIntensity()

    def ir_callback(self, data):
        self.ir_intensity = data
        print("funciona")

    def check_hazard(self):
        #print(self.ir_intensity[0].value)
        print("funciona")
        
def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = IR()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
