import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
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
        self.initial_position_set = False
        self.start_time = time.time()

    def odom_callback(self, data):
        current_time = time.time()
        while not (current_time - self.start_time) >= 1:
            time.sleep(0.1)
            current_time = time.time()
        position = data.pose.pose.position
        self.initial_position = (position.x, position.y)
        self.get_logger().info("Initial position set: {}".format(self.initial_position))
            


class GoToGoal(Node):
    def __init__(self, initial_position):
        super().__init__("GoToGoalNode")
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.timer = self.create_timer(0.1, self.go_to_goal)
        self.odom = Odometry()
        self.path = []
        self.current_goal_index = 0
        self.start_time = time.time()
        self.initial_run = True
        self.initial_position = initial_position
        self.goal=goal

    def odom_callback(self, data):
        self.odom = data

    def go_to_goal(self):
        current_time = time.time()
        if self.initial_run and (current_time - self.start_time) < 1:
            return
        if self.initial_run:
            self.get_logger().info("Execution start after 1 seconds.")
            self.initial_run = False

        goal = Odometry()
        goal.pose.pose.position.x, goal.pose.pose.position.y = self.path[self.current_goal_index]

        new_vel = Twist()

        # Calculate euclidean distance to the goal
        distance_to_goal = math.sqrt((goal.pose.pose.position.x - self.odom.pose.pose.position.x) ** 2 + (goal.pose.pose.position.y - self.odom.pose.pose.position.y) ** 2)

        # Calculate the angle to the goal
        angle_to_goal = math.atan2(goal.pose.pose.position.y - self.odom.pose.pose.position.y, goal.pose.pose.position.x - self.odom.pose.pose.position.x)

        distance_tolerance = 0.1

        # Calculate angle error
        yaw = math.atan2(2 * (self.odom.pose.pose.orientation.w * self.odom.pose.pose.orientation.z + self.odom.pose.pose.orientation.x * self.odom.pose.pose.orientation.y), 1 - 2 * (self.odom.pose.pose.orientation.y ** 2 + self.odom.pose.pose.orientation.z ** 2))
        angle_error = angle_to_goal - yaw

        if angle_error > math.pi:
            angle_error -= 2 * math.pi
        elif angle_error < -math.pi:
            angle_error += 2 * math.pi

        kp_ang = 1.5
        kp_lin = 0.55

        if abs(distance_to_goal) > distance_tolerance:
            new_vel.angular.z = max(min(kp_ang * angle_error, 1.0), -1.0)
            new_vel.linear.x = max(min(kp_lin * distance_to_goal, 1.0), 0.0)
        else:
            self.handle_goal_reached()

        if self.current_goal_index >= len(self.path):
            self.handle_final_goal_reached()

        self.cmd_vel_pub.publish(new_vel)

    def handle_goal_reached(self):
        self.get_logger().info("Goal {} reached".format(self.current_goal_index))
        self.get_logger().info("Current position: {}, {}".format(self.odom.pose.pose.position.x, self.odom.pose.pose.position.y))
        self.current_goal_index += 1

    def handle_final_goal_reached(self):
        new_vel = Twist()
        new_vel.linear.x = 0.0
        new_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(new_vel)
        self.get_logger().info("End of the goal list ({})".format(self.current_goal_index))
        self.get_logger().info("Current position: {}, {}".format(self.odom.pose.pose.position.x, self.odom.pose.pose.position.y))

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
    nodes = [RRTStarNode(*start)]
    img_with_path = np.copy(img)
    goal_reached = False

    for _ in range(max_iter):
        x_rand, y_rand = random.randint(0, img.shape[1] - 1), random.randint(0, img.shape[0] - 1)
        nearest = nearest_node(nodes, x_rand, y_rand)
        x_new, y_new = new_point(x_rand, y_rand, nearest.x, nearest.y, step_size_cm)

        if is_valid_point(img, int(x_new), int(y_new), robot_radius):
            node_new = RRTStarNode(int(x_new), int(y_new))
            near_nodes = [node for node in nodes if math.sqrt((node.x - node_new.x) ** 2 + (node.y - node_new.y) ** 2) < rewiring_radius_cm]
            min_cost_node = nearest_node(near_nodes, x_new, y_new)

            if not has_collision(img, min_cost_node.x, min_cost_node.y, node_new.x, node_new.y, robot_radius):
                node_new.parent = min_cost_node
                node_new.cost = min_cost_node.cost + math.sqrt((node_new.x - min_cost_node.x) ** 2 + (node_new.y - min_cost_node.y) ** 2)

                for near_node in near_nodes:
                    new_cost = node_new.cost + math.sqrt((node_new.x - near_node.x) ** 2 + (node_new.y - near_node.y) ** 2)
                    if new_cost < near_node.cost and not has_collision(img, node_new.x, node_new.y, near_node.x, near_node.y, robot_radius):
                        near_node.parent, near_node.cost = node_new, new_cost

                nodes.append(node_new)
                cv2.line(img_with_path, (min_cost_node.x, min_cost_node.y), (node_new.x, node_new.y), (200, 200, 200), 1)

                if not goal_reached and not has_collision(img, node_new.x, node_new.y, goal[0], goal[1], robot_radius):
                    goal_node = RRTStarNode(*goal)
                    goal_node.parent = node_new
                    goal_node.cost = node_new.cost + math.sqrt((goal_node.x - node_new.x) ** 2 + (goal_node.y - node_new.y) ** 2)
                    nodes.append(goal_node)
                    cv2.line(img_with_path, (node_new.x, node_new.y), (goal_node.x, goal_node.y), (0, 255, 0), 2)
                    goal_reached = True

                if goal_reached:
                    current_node = goal_node
                    while current_node.parent is not None:
                        cv2.line(img_with_path, (current_node.x, current_node.y), (current_node.parent.x, current_node.parent.y), (0, 255, 0), 2)
                        current_node = current_node.parent

                    for node in nodes:
                        if node.parent is not None:
                            cv2.circle(img_with_path, (node.x, node.y), 2, (0, 0, 255), -1)

                    return img_with_path, nodes, start, goal

    return img_with_path, nodes, start, goal





def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONUP:
        click_coordinates, img_with_markers = param
        marker_type = cv2.MARKER_CROSS
        marker_size, thickness = 10, 2

        if not click_coordinates:
            start = (x, y)
            click_coordinates.append(start)
        else:
            goal = (x, y)
            click_coordinates.append(goal)

        for point, label in zip(click_coordinates, ['start (auto)', 'goal']):
            cv2.drawMarker(img_with_markers, point, (0, 0, 255), markerType=marker_type, markerSize=marker_size, thickness=thickness)
            cv2.putText(img_with_markers, label, (point[0] + 10, point[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

def draw_marker_on_image(img_with_markers, marker_params):
    for point, label in marker_params:
        cv2.drawMarker(img_with_markers, (int(point[0]), int(point[1])), (0, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=10, thickness=2)
        cv2.putText(img_with_markers, label, (int(point[0]) + 10, int(point[1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    return img_with_markers
    

def main(args=None):
    rclpy.init(args=args)

    img_path = f'./{"mapa.png"}'
    img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    step_size_cm = float(input("Enter step size (in cm): "))
    max_iterations = int(1000000)
    rewiring_radius_cm = float(input("Enter rewiring radius (in cm): "))
    robot_radius = int(input("Enter robot radius (in cm): "))

    # GET INITIAL POSITION
    initializer = GoToGoalInitializer()
    rclpy.spin_once(initializer)
    initializer.destroy_node()
    start= initializer.initial_position

    # Muestra la imagen con la posición inicial marcada
    img_with_markers = np.copy(img)
    cv2.drawMarker(img_with_markers, (int(start[0]), int(start[1])), (0, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=10, thickness=2)
    cv2.imshow("Map", img_with_markers)
    
    click_coordinates = []
    cv2.setMouseCallback("Map", mouse_callback, [click_coordinates, img_with_markers])
    while len(click_coordinates) < 1:
        cv2.imshow("Map", img_with_markers)
        cv2.waitKey(1)
        
    goal = click_coordinates[0]
    print(int(start)*0.01)
    print(goal)

    img_with_path, nodes, _, _ = rrt_star(img, start, goal, step_size_cm, max_iterations, rewiring_radius_cm, robot_radius)

    for point in [start, goal]:
        cv2.drawMarker(img_with_path, (int(point[0]), int(point[1])), (0, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=10, thickness=2)

    cv2.destroyAllWindows()

    cv2.imshow("Map with RRT*", img_with_path)
    cv2.waitKey(0)

    img_final_with_markers = draw_marker_on_image(img_with_path, start, goal)
    if len(img_final_with_markers.shape) == 2 or img_final_with_markers.shape[2] == 1:
        img_final_with_markers = cv2.cvtColor(img_final_with_markers, cv2.COLOR_GRAY2BGR)

    cv2.imwrite("final_solution.png", img_final_with_markers, [int(cv2.IMWRITE_PNG_COMPRESSION), 9])

    initial_position = None  # Initial position is set automatically
    minimal_publisher = GoToGoal(initial_position)
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
