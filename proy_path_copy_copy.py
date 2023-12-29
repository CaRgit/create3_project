import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from irobot_create_msgs.msg import LedColor
from irobot_create_msgs.msg import LightringLeds
from irobot_create_msgs.msg import IrIntensityVector
from irobot_create_msgs.msg import AudioNoteVector
from irobot_create_msgs.msg import AudioNote
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math
import cv2
import numpy as np
import random
import time

### GET INITIAL POSITION ###

class GetInitialPosition(Node):
    def __init__(self):
        super().__init__("GetInitialPosition")
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.start_time = time.time()

    def odom_callback(self, data):
        current_time = time.time()
        self.get_logger().info(f"Getting initial position...")
        while not (current_time - self.start_time) >= 2:
            time.sleep(0.1)
            current_time = time.time()
        self.initial_position = (data.pose.pose.position.x, data.pose.pose.position.y)
        self.get_logger().info(f"Initial position set: {self.initial_position}")

### GO TO GOAL ###

class GoToGoal(Node):
    def __init__(self, points):
        super().__init__("GoToGoalNode")
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lights_publisher = self.create_publisher(LightringLeds, '/cmd_lightring', 10)
        self.audio_publisher = self.create_publisher(AudioNoteVector, '/cmd_audio', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscription = self.create_subscription(IrIntensityVector,'/ir_intensity', self.ir_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
        self.timer = self.create_timer(0.1, self.go_to_goal)
        
        self.odom = Odometry()
        self.last_lightring = LightringLeds()
        self.last_lightring.override_system = True 
        ### PRUEBA CON AUDIO ###
        self.audio_msg = AudioNoteVector()
        self.audio_msg.header.stamp = self.odom.header.stamp
        self.audio_msg.append = True 
        # Agregar notas para reproducir un sonido específico
        # Ajustar la frecuencia y la duración de las notas
        self.note = AudioNote()
        self.note.frequency = 1000  # Ajustar la frecuencia
        self.note.max_runtime.sec = 2  # Ajustar la duración en segundos
        self.audio_msg.notes.append(self.note)
        ### PRUEBA CON AUDIO ###
        
        self.path = points
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
        #Para acceder a los valores ir por separado: ([self.ir[2], self.ir[3], self.ir[4]], ...)    

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

        kp_ang, kp_lin = 8, 4

        if abs(distance_to_goal) > distance_tolerance:
            new_vel.angular.z = kp_ang * angle_error
            new_vel.linear.x = float(max(0.0, (1 - abs(angle_error)*2 / math.pi) * kp_lin * distance_to_goal))
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

            ### PRUEBA CON AUDIO ###
            self.audio_publisher.publish(self.audio_msg)
            ### PRUEBA CON AUDIO ###

        else:
            if any(lectura > 800 for lectura in self.ir): 
                new_vel.linear.x = 0.0
                new_vel.angular.z = 0.0
                print('Obstacle detected')

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

### GET PATH (RRT STAR) ###

class RRTStarNode:
    def __init__(self, x, y):
        self.x, self.y = x, y
        self.parent = None
        self.cost = 0.0

def is_valid_point(img, x, y, diametro_robot):
    mask = cv2.circle(np.zeros_like(img, dtype=np.uint8), (x, y), int(2*diametro_robot/3), 255, thickness=1)
    return not np.any(img[mask == 255] == 0) and 0 <= x < img.shape[1] and 0 <= y < img.shape[0] and img[y, x] != 0

def nearest_node(nodes, x, y):
    distances = np.sqrt((np.array([node.x for node in nodes]) - x)**2 + (np.array([node.y for node in nodes]) - y)**2)
    return nodes[np.argmin(distances)]

def new_point(x_rand, y_rand, x_near, y_near, step_size):
    theta = math.atan2(y_rand - y_near, x_rand - x_near)
    return x_near + step_size * math.cos(theta), y_near + step_size * math.sin(theta)

def has_collision(img, x1, y1, x2, y2, diametro_robot):
    points = np.column_stack((np.linspace(x1, x2, 100), np.linspace(y1, y2, 100)))
    return any(not is_valid_point(img, int(x), int(y), diametro_robot) for x, y in points)

def simplify_path(nodes, img, diametro_robot):
    simplified_nodes = [nodes[0]]  
    for i in range(1, len(nodes)):
        current_node = simplified_nodes[-1]
        next_node = nodes[i]
        while i < len(nodes) and not has_collision(img, current_node.x, current_node.y, next_node.x, next_node.y, diametro_robot):
            i += 1
            if i < len(nodes):
                next_node = nodes[i]
            else:
                break
        if i < len(nodes):
            simplified_nodes.append(nodes[i-1])
    simplified_nodes.append(nodes[-1])
    return simplified_nodes
        
def rrt_star(img, start, goal, step_size_cm, max_iter, diametro_robot):
    nodes = [RRTStarNode(*start)]
    img_with_path = np.copy(img)
    goal_reached = False

    for _ in range(max_iter):
        if random.uniform(0, 1) < 0.2:
            x_rand = random.uniform(max(0, goal[0] - 50), min(img.shape[1] - 1, goal[0] + 50))
            y_rand = random.uniform(max(0, goal[1] - 50), min(img.shape[0] - 1, goal[1] + 50))
        else:
            x_rand, y_rand = random.randint(0, img.shape[1] - 1), random.randint(0, img.shape[0] - 1)

        nearest = nearest_node(nodes, x_rand, y_rand)
        x_new, y_new = new_point(x_rand, y_rand, nearest.x, nearest.y, step_size_cm)

        if is_valid_point(img, int(x_new), int(y_new), diametro_robot):
            node_new = RRTStarNode(int(x_new), int(y_new))

            if not has_collision(img, nearest.x, nearest.y, node_new.x, node_new.y, diametro_robot):
                node_new.parent = nearest
                node_new.cost = nearest.cost + math.sqrt((node_new.x - nearest.x)**2 + (node_new.y - nearest.y)**2)

                nodes.append(node_new)

                cv2.circle(img_with_path, (node_new.x, node_new.y), 1, (0, 0, 255), -1)
                cv2.line(img_with_path, (node_new.x, node_new.y), (node_new.parent.x, node_new.parent.y), (0, 255, 0), 1)
                
                if not has_collision(img, node_new.x, node_new.y, goal[0], goal[1], diametro_robot) and ((math.sqrt((goal[0] - node_new.x)**2 + (goal[1] - node_new.y)**2)) <= step_size_cm):
                    if not goal_reached:
                        penult_nodo = node_new
                        coste_total = node_new.cost + math.sqrt((goal[0] - node_new.x)**2 + (goal[1] - node_new.y)**2)
                        goal_reached = True
                    if (node_new.cost + math.sqrt((goal[0] - node_new.x)**2 + (goal[1] - node_new.y)**2)) < coste_total:
                        penult_nodo = node_new
                        coste_total = node_new.cost + math.sqrt((goal[0] - node_new.x)**2 + (goal[1] - node_new.y)**2)
                   
    if goal_reached:
        goal_node = RRTStarNode(*goal)
        goal_node.parent = penult_nodo
        goal_node.cost = coste_total
        nodes.append(goal_node)

        start_node = RRTStarNode(*start)
        start_node.parent = None
        start_node.cost = 0.0

        nodos = []
        current_node = goal_node
        while current_node.parent is not None:
            nodos.insert(0, current_node)
            current_node = current_node.parent
        nodos.insert(0, start_node)
        
        nodos_simp=simplify_path(nodos, img, diametro_robot)
        
        for node in nodos:
            if node.parent is not None:
                cv2.line(img_with_path, (node.x, node.y), (node.parent.x, node.parent.y), (0, 255, 0), 2)
            cv2.circle(img_with_path, (node.x, node.y), 2, (0, 0, 255), -1)  
        for i in range(1, len(nodos_simp)):
            cv2.line(img_with_path, (nodos_simp[i - 1].x, nodos_simp[i - 1].y), (nodos_simp[i].x, nodos_simp[i].y), (0, 255, 0), 3)
            cv2.circle(img_with_path, (nodos_simp[i].x, nodos_simp[i].y), 3, (0, 0, 255), -1)

        path=[]
        for nodo_simp in nodos_simp:
            path.append((float(nodo_simp.x * 0.01), float(nodo_simp.y * 0.01)))
            
        return img_with_path, path

def mouse_callback(event, x, y, flags, params):
    if event == cv2.EVENT_LBUTTONUP:
        img_with_markers, goal = params
        goal.append((x, y))
        draw_marker_on_image(img_with_markers, 'goal', goal[0])

def draw_marker_on_image(img_with_markers, label, point):
    cv2.drawMarker(img_with_markers, point, (0, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=10, thickness=2)
    cv2.putText(img_with_markers, label, (int(point[0]) + 10, int(point[1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    return img_with_markers

### MAIN FUNCTION ###

def main(args=None):
    rclpy.init(args=args)
    first = True
    end_program = True
    choice=[]

    img_path = './mapa.png'
    img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    step_size_cm = 20 #float(input("Enter step size (in cm): "))
    max_iterations = 250 #int(input("Max iterations for RRT star: "))
    robot_diameter = 40 #int(input("Enter robot diameter (in cm): "))

    while end_program:            
        if first or (choice == 'C'):
            if (choice == 'C'):
                rclpy.init(args=args)
                
            initializer = GetInitialPosition()
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
        
            img_with_path, nodes = rrt_star(img, start, goal, step_size_cm, max_iterations, robot_diameter)
        
            draw_marker_on_image(img_with_path, 'start', start)
            draw_marker_on_image(img_with_path, 'goal', goal)
            cv2.imshow("Map RRT*", img_with_path)
            cv2.waitKey(1)
            cv2.imwrite("final_solution.png", img_with_path, [int(cv2.IMWRITE_PNG_COMPRESSION), 9])
        
            minimal_publisher = GoToGoal(nodes)
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
