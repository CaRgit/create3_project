import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from irobot_create_msgs.msg import IrIntensityVector
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math
import cv2
import numpy as np
import random
import time

class GoToGoalInitializer(Node):
    def __init__(self):
        super().__init__("GoToGoalInitializerNode")
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        #self.initial_position_set = False
        self.start_time = time.time()

    def odom_callback(self, data):
        current_time = time.time()
        while not (current_time - self.start_time) >= 1:
            time.sleep(0.1)
            current_time = time.time()
        self.initial_position = (data.pose.pose.position.x, data.pose.pose.position.y)
        self.get_logger().info(f"Initial position set: {self.initial_position}")

class GoToGoal(Node):
    def __init__(self, points):
        super().__init__("GoToGoalNode")
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscription = self.create_subscription(IrIntensityVector,'/ir_intensity', self.ir_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.timer = self.create_timer(0.1, self.go_to_goal)
        self.odom = Odometry()
        self.path = points
        self.media = 0
        self.ir=[]
        self.current_goal_index = 0
        self.end_of_goals = False

    def odom_callback(self, data):
        self.odom = data

    def ir_callback(self, msg):
        self.ir = []
        for reading in msg.readings:
            intensity_value = reading.value
            self.ir.append(intensity_value)
        #self.media = np.mean([self.ir[2], self.ir[3], self.ir[4]])    

    def go_to_goal(self):
        goal = Odometry()
        goal.pose.pose.position.y, goal.pose.pose.position.x = self.path[self.current_goal_index]
        new_vel = Twist()

        distance_to_goal = math.hypot(goal.pose.pose.position.x - self.odom.pose.pose.position.x, goal.pose.pose.position.y - self.odom.pose.pose.position.y)
        angle_to_goal = math.atan2(goal.pose.pose.position.y - self.odom.pose.pose.position.y, goal.pose.pose.position.x - self.odom.pose.pose.position.x)
        distance_tolerance = 0.1

        yaw = math.atan2(2 * (self.odom.pose.pose.orientation.w * self.odom.pose.pose.orientation.z + self.odom.pose.pose.orientation.x * self.odom.pose.pose.orientation.y), 1 - 2 * (self.odom.pose.pose.orientation.y ** 2 + self.odom.pose.pose.orientation.z ** 2))
        angle_error = angle_to_goal - yaw

        if angle_error > math.pi:
            angle_error -= 2 * math.pi
        elif angle_error < -math.pi:
            angle_error += 2 * math.pi

        kp_ang, kp_lin = 10, 5

        if abs(distance_to_goal) > distance_tolerance:
            new_vel.angular.z = kp_ang * angle_error
            new_vel.linear.x = (1 - abs(angle_error) / math.pi) * kp_lin * distance_to_goal
        else:
            self.current_goal_index += 1
            self.get_logger().info(f"Goal {self.current_goal_index} reached")
            self.get_logger().info(f"Current position: {self.odom.pose.pose.position.x}, {self.odom.pose.pose.position.y}")
            
        if self.current_goal_index >= len(self.path):
            new_vel.linear.x = 0.0
            new_vel.angular.z = 0.0
            self.cmd_vel_pub.publish(new_vel)
            self.get_logger().info(f"End of the goal list ({self.current_goal_index})")
            self.end_of_goals = True

        if any(lectura > 600 for lectura in self.ir): #self.media >= 250:
            new_vel.linear.x = 0.0
            new_vel.angular.z = 0.0
            print('ESTORBAAAS')

        self.cmd_vel_pub.publish(new_vel)


class RRTStarNode:
    def __init__(self, x, y):
        self.x, self.y = x, y
        self.parent = None
        self.cost = 0.0

def is_valid_point(img, x, y, robot_radius):
    mask = cv2.circle(np.zeros_like(img, dtype=np.uint8), (x, y), robot_radius, 255, thickness=1)
    return not np.any(img[mask == 255] == 0) and 0 <= x < img.shape[1] and 0 <= y < img.shape[0] and img[y, x] != 0

def nearest_node(nodes, x, y):
    distances = [(node.x - x) ** 2 + (node.y - y) ** 2 for node in nodes]
    return nodes[np.argmin(distances)]

def new_point(x_rand, y_rand, x_near, y_near, step_size):
    theta = math.atan2(y_rand - y_near, x_rand - x_near)
    return x_near + step_size * math.cos(theta), y_near + step_size * math.sin(theta)

def has_collision(img, x1, y1, x2, y2, robot_radius):
    points = np.column_stack((np.linspace(x1, x2, 100), np.linspace(y1, y2, 100)))
    return any(not is_valid_point(img, int(x), int(y), robot_radius) for x, y in points)

def rrt_star(img, start, goal, step_size_cm, max_iter, rewiring_radius_cm, robot_radius):
    nodes, img_with_path, points = [RRTStarNode(*start)], np.copy(img), []
    goal_node = None

    for _ in range(max_iter):
        x_rand, y_rand = random.randint(0, img.shape[1] - 1), random.randint(0, img.shape[0] - 1)
        nearest = nearest_node(nodes, x_rand, y_rand)
        x_new, y_new = new_point(x_rand, y_rand, nearest.x, nearest.y, step_size_cm)

        if is_valid_point(img, int(x_new), int(y_new), robot_radius):
            node_new = RRTStarNode(int(x_new), int(y_new))
            near_nodes = [node for node in nodes if math.hypot(node.x - node_new.x, node.y - node_new.y) < rewiring_radius_cm]
            min_cost_node = nearest_node(near_nodes, x_new, y_new)

            if not has_collision(img, min_cost_node.x, min_cost_node.y, node_new.x, node_new.y, robot_radius):
                node_new.parent = min_cost_node
                node_new.cost = min_cost_node.cost + math.hypot(node_new.x - min_cost_node.x, node_new.y - min_cost_node.y)

                for near_node in near_nodes:
                    new_cost = node_new.cost + math.hypot(node_new.x - near_node.x, node_new.y - near_node.y)
                    if new_cost < near_node.cost and not has_collision(img, node_new.x, node_new.y, near_node.x, near_node.y, robot_radius):
                        near_node.parent, near_node.cost = node_new, new_cost

                nodes.append(node_new)
                cv2.line(img_with_path, (min_cost_node.x, min_cost_node.y), (node_new.x, node_new.y), (200, 200, 200), 1)

                if not points and not has_collision(img, node_new.x, node_new.y, goal[0], goal[1], robot_radius):
                    goal_node = RRTStarNode(*goal)
                    goal_node.parent = node_new
                    goal_node.cost = node_new.cost + math.hypot(goal_node.x - node_new.x, goal_node.y - node_new.y)
                    nodes.append(goal_node)
                    cv2.line(img_with_path, (node_new.x, node_new.y), (goal_node.x, goal_node.y), (0, 255, 0), 2)

                if goal_node in nodes:
                    current_node = goal_node
                    while current_node.parent is not None:
                        cv2.line(img_with_path, (current_node.x, current_node.y), (current_node.parent.x, current_node.parent.y), (0, 255, 0), 2)
                        points.append((float(current_node.x * 0.01), float(current_node.y * 0.01)))
                        current_node = current_node.parent
                    points.reverse()

                    return img_with_path, points, start, goal

    return img_with_path, points, start, goal

def mouse_callback(event, x, y, flags, params):
    if event == cv2.EVENT_LBUTTONUP:
        img_with_markers, goal = params
        goal.append((x, y))
        draw_marker_on_image(img_with_markers, 'goal', goal[0])

def draw_marker_on_image(img_with_markers, label, point):
    cv2.drawMarker(img_with_markers, point, (0, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=10, thickness=2)
    cv2.putText(img_with_markers, label, (int(point[0]) + 10, int(point[1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    return img_with_markers

def main(args=None):
    rclpy.init(args=args)
    first = True
    end_program = True
    choice=[]

    img_path = './mapa.png'
    img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    step_size_cm = float(input("Enter step size (in cm): "))
    max_iterations = 1000000
    rewiring_radius_cm = float(input("Enter rewiring radius (in cm): "))
    robot_radius = int(input("Enter robot radius (in cm): "))
    robot_radius = robot_radius + 5
    

    while end_program:            
        if first or (choice == 'C'):
            if (choice == 'C'):
                rclpy.init(args=args)
                
            initializer = GoToGoalInitializer()
            rclpy.spin_once(initializer)
            initializer.destroy_node()
            start = (int(initializer.initial_position[1] * 100), int(initializer.initial_position[0] * 100))
        
            img_with_path = np.copy(img)
            draw_marker_on_image(img_with_path, 'start', start)
            cv2.imshow("Map RRT*", img_with_path)
        
            goal = []
            cv2.setMouseCallback("Map RRT*", mouse_callback, [img_with_path, goal])
            while len(goal) < 1:
                cv2.waitKey(1)
            goal = goal[0]
        
            img_with_path, trajectory, _, _ = rrt_star(img, start, goal, step_size_cm, max_iterations, rewiring_radius_cm, robot_radius)
        
            draw_marker_on_image(img_with_path, 'start', start)
            draw_marker_on_image(img_with_path, 'goal', goal)
            cv2.imshow("Map RRT*", img_with_path)
            cv2.waitKey(1)
            cv2.imwrite("final_solution.png", img_with_path, [int(cv2.IMWRITE_PNG_COMPRESSION), 9])

            
            #minimal_publisher = GoToGoal(trajectory)
            #rclpy.spin(minimal_publisher)
            #print('HOLA')
            #minimal_publisher.destroy_node()
            #rclpy.shutdown()
        
            minimal_publisher = GoToGoal(trajectory)
            while rclpy.ok() and not minimal_publisher.end_of_goals:
                rclpy.spin_once(minimal_publisher, timeout_sec=0.1)
            minimal_publisher.destroy_node()
            rclpy.shutdown()

            cv2.destroyAllWindows()
        elif (choice== 'E'):
            return
        else:
            if first:
                return
            else:
                print('Invalid choice.')

        if first:
            first = False
        
        choice = input("C --> Continue, E --> End: ")

if __name__ == '__main__':
    main()

