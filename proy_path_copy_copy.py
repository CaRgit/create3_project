import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from irobot_create_msgs.msg import LedColor
from irobot_create_msgs.msg import LightringLeds
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
    def __init__(self, points, step_size_cm):
        super().__init__("GoToGoalNode")
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscription = self.create_subscription(IrIntensityVector,'/ir_intensity', self.ir_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.timer = self.create_timer(0.1, self.go_to_goal)
        self.odom = Odometry()
        self.path = points
        self.step_size = step_size_cm
        self.media = 0
        self.ir=[]
        self.current_goal_index = 0
        self.end_of_goals = False

        self.lights_publisher = self.create_publisher(LightringLeds, 'cmd_lightring', 10)
        self.last_lightring = LightringLeds()
        self.last_lightring.override_system = True        

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

        lightring = self.last_lightring
        cp = ColorPalette()

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
            new_vel.linear.x = (1 - abs(angle_error)*2 / math.pi) * kp_lin * distance_to_goal
        elif abs(distance_to_goal) < self.step_size/4: 
            self.current_goal_index += 1
            self.get_logger().info(f"Looking for goal {self.current_goal_index}")
            self.get_logger().info(f"Current position: {self.odom.pose.pose.position.x}, {self.odom.pose.pose.position.y}")
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

            lightring = LightringLeds()
            lightring.override_system = True
            lightring.leds = [cp.green, cp.green, cp.green, cp.green, cp.green, cp.green]
            self.last_lightring = lightring

        else:
            if any(lectura > 600 for lectura in self.ir): #self.media >= 250:
                new_vel.linear.x = 0.0
                new_vel.angular.z = 0.0
                print('ESTORBAAAS')

                lightring = LightringLeds()
                lightring.override_system = True
                lightring.leds = [cp.red, cp.red, cp.red, cp.red, cp.red, cp.red]
                self.last_lightring = lightring
            
            elif(self.last_lightring.override_system == True):
                lightring = LightringLeds()
                lightring.override_system = True
                lightring.leds = [cp.blue, cp.blue, cp.blue, cp.blue, cp.blue, cp.blue]
                self.last_lightring = lightring

        self.cmd_vel_pub.publish(new_vel)

        self.lights_publisher.publish(lightring)


class RRTStarNode:
    def __init__(self, x, y):
        self.x, self.y = x, y
        self.parent = None
        self.cost = 0.0

def is_valid_point(img, x, y, robot_radius):
    mask = cv2.circle(np.zeros_like(img, dtype=np.uint8), (x, y), robot_radius, 255, thickness=1)
    return not np.any(img[mask == 255] == 0) and 0 <= x < img.shape[1] and 0 <= y < img.shape[0] and img[y, x] != 0

def nearest_node(nodes, x, y):
    distances = np.sqrt((np.array([node.x for node in nodes]) - x)**2 + (np.array([node.y for node in nodes]) - y)**2)
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
        if random.uniform(0, 1) < 0.2:  # Ajusta el umbral según tus necesidades
            x_rand, y_rand = goal
        else:
            if random.uniform(0, 1) < 0.8: # Genera puntos cercanos al objetivo con una probabilidad más alta
                x_rand = random.uniform(max(0, goal[0] - 50), min(img.shape[1] - 1, goal[0] + 50))
                y_rand = random.uniform(max(0, goal[1] - 50), min(img.shape[0] - 1, goal[1] + 50))
            else:
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
        
            minimal_publisher = GoToGoal(trajectory, step_size_cm)
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

class ColorPalette():
    """ Helper Class to define frequently used colors"""
    def __init__(self):
        self.red = LedColor(red=255,green=0,blue=0)
        self.green = LedColor(red=0,green=255,blue=0)
        self.blue = LedColor(red=0,green=0,blue=255)
        self.yellow = LedColor(red=255,green=255,blue=0)
        self.pink = LedColor(red=255,green=0,blue=255)
        self.cyan = LedColor(red=0,green=255,blue=255)
        self.purple = LedColor(red=127,green=0,blue=255)
        self.white = LedColor(red=255,green=255,blue=255)
        self.grey = LedColor(red=189,green=189,blue=189)
        self.default = LedColor(red=1,green=1,blue=1)

class Lights():
    """ Class to tell the robot to set lightring lights as part of dance sequence"""
    def __init__(self, led_colors):
        """
        Parameters
        ----------
        led_colors : list of LedColor
            The list of 6 LedColors corresponding to the 6 LED lights on the lightring
        """
        self.led_colors = led_colors

if __name__ == '__main__':
    main()
