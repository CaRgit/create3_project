import cv2
import numpy as np
import random
import math

class Node:
    def __init__(self, x, y):
        self.x, self.y = x, y
        self.parent = None
        self.cost = 0.0

def is_valid_point(img, x, y, diametro_robot):
    mask = cv2.circle(np.zeros_like(img, dtype=np.uint8), (x, y), diametro_robot, 255, thickness=1)
    return not np.any(img[mask == 255] == 0) and 0 <= x < img.shape[1] and 0 <= y < img.shape[0] and img[y, x] != 0

def nearest_node(nodes, x, y):
    distances = np.sqrt((np.array([node.x for node in nodes]) - x)**2 + (np.array([node.y for node in nodes]) - y)**2)
    return nodes[np.argmin(distances)]

def new_point(x_rand, y_rand, x_near, y_near, step_size):
    theta = math.atan2(y_rand - y_near, x_rand - x_near)
    return x_near + step_size * math.cos(theta), y_near + step_size * math.sin(theta)

def has_collision(img, x1, y1, x2, y2, diametro_robot, num_points_collision=100):
    points = np.column_stack((np.linspace(x1, x2, num_points_collision), np.linspace(y1, y2, num_points_collision)))
    return any(not is_valid_point(img, int(x), int(y), diametro_robot) for x, y in points)

def simplify_path(nodes, img, diametro_robot, num_points_simplify=100):
    simplified_nodes = [nodes[0]]
    i = 0
    while i < len(nodes) - 1:
        current_node = nodes[i]
        next_node = nodes[i + 1]
        while i < len(nodes) and not has_collision(img, current_node.x, current_node.y, next_node.x, next_node.y, diametro_robot):
            next_node = nodes[i]
            i += 1
            
        simplified_nodes.append(nodes[i - 1])
    return simplified_nodes

def draw_path_on_image(img, nodes, nodes_simplified):
    img_with_path = np.copy(img)
    for node in nodes:
        if node.parent is not None:
            cv2.line(img_with_path, (node.x, node.y), (node.parent.x, node.parent.y), (0, 255, 0), 2)
        cv2.circle(img_with_path, (node.x, node.y), 3, (0, 0, 255), -1)  
    for i in range(1, len(nodes_simplified)):
        cv2.line(img_with_path, (nodes_simplified[i - 1].x, nodes_simplified[i - 1].y), (nodes_simplified[i].x, nodes_simplified[i].y), (0, 255, 0), 3)
        cv2.circle(img_with_path, (nodes_simplified[i].x, nodes_simplified[i].y), 4, (0, 0, 255), -1)
    return img_with_path

def rrt_star(img, start, goal, step_size_cm, max_iter, diametro_robot):
    nodes = [Node(*start)]
    img_with_path = np.copy(img)
    goal_reached = False

    for _ in range(max_iter):
        x_rand, y_rand = generate_random_point(goal, img.shape[1], img.shape[0])

        nearest = nearest_node(nodes, x_rand, y_rand)
        x_new, y_new = new_point(x_rand, y_rand, nearest.x, nearest.y, step_size_cm)

        if is_valid_point(img, int(x_new), int(y_new), diametro_robot):
            node_new = Node(int(x_new), int(y_new))

            if not has_collision(img, nearest.x, nearest.y, node_new.x, node_new.y, diametro_robot):
                node_new.parent = nearest
                node_new.cost = nearest.cost + math.sqrt((node_new.x - nearest.x)**2 + (node_new.y - nearest.y)**2)

                nodes.append(node_new)

                cv2.circle(img_with_path, (node_new.x, node_new.y), 2, (0, 0, 255), -1)
                cv2.line(img_with_path, (node_new.x, node_new.y), (node_new.parent.x, node_new.parent.y), (0, 255, 0), 1)
                
                if not has_collision(img, node_new.x, node_new.y, goal[0], goal[1], diametro_robot) and ((math.sqrt((goal[0] - node_new.x)**2 + (goal[1] - node_new.y)**2)) <= step_size_cm):
                    if not goal_reached:
                        penult_nodo = node_new
                        coste_total = node_new.cost + math.sqrt((goal[0] - node_new.x)**2 + (goal[1] - node_new.y)**2)
                        goal_reached = True
                        print(coste_total)
                    if (node_new.cost + math.sqrt((goal[0] - node_new.x)**2 + (goal[1] - node_new.y)**2)) < coste_total:
                        penult_nodo = node_new
                        coste_total = node_new.cost + math.sqrt((goal[0] - node_new.x)**2 + (goal[1] - node_new.y)**2)
                        print(coste_total)
                   
    if goal_reached:
        goal_node = Node(*goal)
        goal_node.parent = penult_nodo
        goal_node.cost = coste_total
        nodes.append(goal_node)

        start_node = Node(*start)
        start_node.parent = None
        start_node.cost = 0.0
        
        current_node = goal_node
        while current_node.parent is not None:
            nodos.insert(0, current_node)
            current_node = current_node.parent
        nodos.insert(0, start_node)
        
        nodos_simp = simplify_path(nodos, img, diametro_robot)
        
        img_with_path = draw_path_on_image(img, nodos, nodos_simp)
        
        return img_with_path, nodos_simp

def generate_random_point(goal, img_width, img_height, exploration_radius=50):
    if random.uniform(0, 1) < 0.1:
        x_rand = random.uniform(max(0, goal[0] - exploration_radius), min(img_width - 1, goal[0] + exploration_radius))
        y_rand = random.uniform(max(0, goal[1] - exploration_radius), min(img_height - 1, goal[1] + exploration_radius))
    else:
        x_rand, y_rand = random.randint(0, img_width - 1), random.randint(0, img_height - 1)
    return x_rand, y_rand

def main():
    img_path = f'./{"mapa.png"}'
    img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)

    step_size_cm = float(15)
    max_iterations = int(500)
    diametro_robot = int(15)

    cv2.imshow("Mapa", img)

    click_coordinates = []
    img_with_markers = np.copy(img)
    cv2.setMouseCallback("Mapa", mouse_callback, [click_coordinates, img_with_markers])

    while len(click_coordinates) < 2:
        cv2.imshow("Mapa", img_with_markers)
        cv2.waitKey(1)

    start, goal = click_coordinates[0], click_coordinates[1]

    img_with_path, nodes = rrt_star(img, start, goal, step_size_cm, max_iterations, diametro_robot)

    for point in [start, goal]:
        cv2.drawMarker(img_with_path, (int(point[0]), int(point[1])), (0, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=10, thickness=2)

    cv2.destroyAllWindows()

    cv2.imshow("Mapa con RRT*", img_with_path)
    cv2.waitKey(0)

    img_final_with_markers = draw_markers_on_image(img_with_path, start, goal)

    if len(img_final_with_markers.shape) == 2 or img_final_with_markers.shape[2] == 1:
        img_final_with_markers = cv2.cvtColor(img_final_with_markers, cv2.COLOR_GRAY2BGR)

    cv2.imwrite("final_solution.png", img_final_with_markers, [int(cv2.IMWRITE_PNG_COMPRESSION), 9])

    save_path_to_txt(nodes, 'path.txt', scale=0.01)

if __name__ == "__main__":
    main()


