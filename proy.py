import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile
import sys
import math

class GTG(Node):
    def __init__(self):
        super().__init__("Go_to_Goal_Node")
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
        kp_ang = 1.5
        kp_lin = 0.55

        if (abs(distance_to_goal) > distance_tolerance):
            new_vel.angular.z = kp_ang * angle_error
            new_vel.linear.x = kp_lin * distance_to_goal
        else:
            self.get_logger().info("Objetivo ", self.current_goal_index ," alcanzado")
            self.get_logger().info("Estoy en: ", self.odom.pose.pose.position.x ," ", self.odom.pose.pose.position.y)
            self.current_goal_index += 1

        if self.current_goal_index >= len(self.path):
            new_vel.linear.x = 0.0
            new_vel.angular.z = 0.0
            self.get_logger().info("Final de la lista de objetivos alcanzado (", self.current_goal_index ,")")
            self.get_logger().info("Estoy en: ", self.odom.pose.pose.position.x ,", ", self.odom.pose.pose.position.y)
            quit()
        
        self.cmd_vel_pub.publish(new_vel)

def main(args=None):
    rclpy.init(args=args)
    #subprocess.run(["python3", "rrt.py"])
    minimal_publisher = GTG()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    






