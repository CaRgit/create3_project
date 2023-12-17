
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
        self.ir_1 = []
        self.ir_2 = []
        self.ir_3 = []
        self.ir_4 = []
        self.ir_5 = []
        self.ir_6 = []
        self.ir_7 = []
        
    def ir_callback(self, msg):
        for i, reading in enumerate(msg.readings):
            intensity_value = reading.value
            if i == 0:
                self.ir_1.append(intensity_value)
            elif i == 1:
                self.ir_2.append(intensity_value)
            elif i == 2:
                self.ir_3.append(intensity_value)
            elif i == 3:
                self.ir_4.append(intensity_value)
            elif i == 4:
                self.ir_5.append(intensity_value)
            elif i == 5:
                self.ir_6.append(intensity_value)
            elif i == 6:
                self.ir_7.append(intensity_value)
                
            self.get_logger().info(f'Lectura {i + 1}: Intensidad: {intensity_value}')
            media = np.mean(intensity_value)
            print(media)

        
def main(args=None):
    rclpy.init(args=args)
    ir_node = IR()
    rclpy.spin_once(ir_node)
    ir_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
