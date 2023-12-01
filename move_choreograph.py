import math

import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import SetParameters

from geometry_msgs.msg import Twist

class Move():
    def __init__(self, x_m_s, theta_degrees_second):
        """
        Parameters
        ----------
        x_m_s : float
            The speed to drive the robot forward (positive) /backwards (negative) in m/s    
        theta_degrees_second : float
            The speed to rotate the robot counter clockwise (positive) / clockwise (negative) in deg/s
        """
        self.x = x_m_s
        self.theta = math.radians(theta_degrees_second)

class MoveChoreographer():
    def get_next_action(self):
        action = Move(0.1, 90)
        return action

class MoveCommandPublisher(Node):
    def __init__(self):
        super().__init__('move_command_publisher')
        self.vel.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
    def timer_callback(self):
        next_action = self.move_choreographer.get_next_action()
        twist = Twist()
        twist.linear.x = next_action.x
        twist.angular.z = next_action.theta
        self.vel_publisher.publish(twist)
