
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile
import sys
import math

class GTG(Node):
    def __init__(self):
        super().__init__("Infrared_Node")
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry,'/odom', self.odom_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.timer = self.create_timer(0.1, self.go_to_goal)
        self.odom = Odometry()

        self.path = []
        with open('path.txt', 'r') as file:
            for line in file:
                x, y = map(float, line.split())
                self.path.append((x, y))
        self.current_goal_index = 0

    def odom_callback(self, data):
        self.odom = data

    def go_to_goal(self):
        goal = Odometry()
        goal.pose.pose.position.x, goal.pose.pose.position.y = self.path[self.current_goal_index]

        new_vel = Twist()

        # Ecludian Distance 
        distance_to_goal = math.sqrt((goal.pose.pose.position.x - self.odom.pose.pose.position.x)**2  + (goal.pose.pose.position.y - self.odom.pose.pose.position.y)**2)
        # Angle to Goal
        angle_to_goal = math.atan2(goal.pose.pose.position.y - self.odom.pose.pose.position.y , goal.pose.pose.position.x - self.odom.pose.pose.position.x)

        distance_tolerance = 0.1

        yaw = math.atan2(2*(self.odom.pose.pose.orientation.w * self.odom.pose.pose.orientation.z + self.odom.pose.pose.orientation.x * self.odom.pose.pose.orientation.y), 1 - 2 * (self.odom.pose.pose.orientation.y**2 + self.odom.pose.pose.orientation.z**2))
        angle_error = angle_to_goal - yaw
        if angle_error > math.pi:
            angle_error -= 2 * math.pi
        elif angle_error < -math.pi:
            angle_error += 2 * math.pi

        kp_ang = 10
        kp_lin = 5

        if (abs(distance_to_goal) > distance_tolerance):
            # new_vel.angular.z = max(min(kp_ang * angle_error, 1.0), -1.0)
            new_vel.angular.z = kp_ang * angle_error
            #if abs(angle_error) < math.pi:
                # new_vel.linear.x = max(min(kp_lin * distance_to_goal, 1.0), 0.0)
            new_vel.linear.x = (1 - abs(angle_error) / math.pi) * kp_lin * distance_to_goal
            #else:
            #    new_vel.linear.x = 0.0
        else:
            self.get_logger().info("Objetivo {} alcanzado".format(self.current_goal_index))
            self.get_logger().info("Estoy en: {}, {}".format(self.odom.pose.pose.position.x, self.odom.pose.pose.position.y))
            self.current_goal_index += 1

        if self.current_goal_index >= len(self.path):
            new_vel.linear.x = 0.0
            new_vel.angular.z = 0.0
            self.get_logger().info("Final de la lista de objetivos alcanzado ({})".format(self.current_goal_index))
            self.get_logger().info("Estoy en: {}, {}".format(self.odom.pose.pose.position.x, self.odom.pose.pose.position.y))
            quit()
        
        self.cmd_vel_pub.publish(new_vel)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = GTG()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
