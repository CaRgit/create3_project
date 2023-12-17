
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
        self.subscription = self.create_subscription(IrIntensityVector,'/ir_intensity', self.ir_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

    def ir_callback(self, msg):
        for reading in msg.readings:
            intensity_value = reading.value
            self.get_logger().info(f'Intensidad: {intensity_value}')
        
def main(args=None):
    rclpy.init(args=args)
    ir_node = IR()
    rclpy.spin_once(ir_node)
    ir_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
