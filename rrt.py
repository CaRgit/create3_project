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

def has_collision(img, x1, y1, x2, y2, diametro_robot):
    points = np.column_stack((np.linspace(x1, x2, 100), np.linspace(y1, y2, 100)))
    return any(not is_valid_point(img, int(x), int(y), diametro_robot) for x, y in points)

def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONUP:
        click_coordinates, img_with_markers = param

        if not click_coordinates:
            start = (x, y)
            click_coordinates.append(start)
        else:
            goal = (x, y)
            click_coordinates.append(goal)

        for point, label in zip(click_coordinates, ['ini', 'fin']):
            cv2.drawMarker(img_with_markers, point, (0, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=10, thickness=2)
            cv2.putText(img_with_markers, label, (point[0] + 10, point[1] + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

def rrt_star(img, start, goal, step_size_cm, max_iter, diametro_robot):
    nodes = [Node(*start)]
    img_with_path = np.copy(img)
    goal_reached = False

    for _ in range(max_iter):
        if random.uniform(0, 1) < 0.2:
            x_rand, y_rand = goal
        elif random.uniform(0, 1) < 0.8:
            x_rand = random.uniform(max(0, goal[0] - 50), min(img.shape[1] - 1, goal[0] + 50))
            y_rand = random.uniform(max(0, goal[1] - 50), min(img.shape[0] - 1, goal[1] + 50))
        else:
            x_rand, y_rand = random.randint(0, img.shape[1] - 1), random.randint(0, img.shape[0] - 1)

        nearest = nearest_node(nodes, x_rand, y_rand)
        x_new, y_new = new_point(x_rand, y_rand, nearest.x, nearest.y, step_size_cm)

        if is_valid_point(img, int(x_new), int(y_new), diametro_robot):
            node_new = Node(int(x_new), int(y_new))
            near_nodes = [node for node in nodes]
            min_cost_node = nearest_node(near_nodes, x_new, y_new)

            if not has_collision(img, min_cost_node.x, min_cost_node.y, node_new.x, node_new.y, diametro_robot):
                node_new.parent = min_cost_node
                node_new.cost = min_cost_node.cost + math.sqrt((node_new.x - min_cost_node.x)**2 + (node_new.y - min_cost_node.y)**2)

                for near_node in near_nodes:
                    new_cost = node_new.cost + math.sqrt((node_new.x - near_node.x)**2 + (node_new.y - near_node.y)**2)
                    if new_cost < near_node.cost and not has_collision(img, node_new.x, node_new.y, near_node.x, near_node.y, diametro_robot):
                        near_node.parent, near_node.cost = node_new, new_cost

                nodes.append(node_new)

                cv2.line(img_with_path, (node_new.x, node_new.y), (min_cost_node.x, min_cost_node.y), (255, 0, 0), 1)

                for existing_node in nodes:
                    if existing_node.parent is not None:
                        cv2.line(img_with_path, (existing_node.x, existing_node.y), (existing_node.parent.x, existing_node.parent.y), (0, 255, 0), 1)

                if not goal_reached and not has_collision(img, node_new.x, node_new.y, goal[0], goal[1], diametro_robot):
                    goal_node = Node(*goal)
                    goal_node.parent = node_new
                    goal_node.cost = node_new.cost + math.sqrt((goal_node.x - node_new.x)**2 + (goal_node.y - node_new.y)**2)
                    nodes.append(goal_node)
                    goal_reached = True

                if goal_reached:
                    current_node = goal_node
                    while current_node.parent is not None:
                        cv2.line(img_with_path, (current_node.x, current_node.y), (current_node.parent.x, current_node.parent.y), (0, 255, 0), 2)
                        current_node = current_node.parent

                    for node in nodes:
                        if node.parent is not None:
                            cv2.circle(img_with_path, (node.x, node.y), 2, (0, 0, 255), -1)

                    return img_with_path, nodes

    return img_with_path, nodes


def save_path_to_txt(nodes, filename, scale=0.01):
    with open(filename, 'w') as file:
        for node in nodes:
            if node.parent is not None:
                x, y = round(node.x * scale, 2), round(node.y * scale, 2)
                file.write(f'{x} {y}\n')

def draw_markers_on_image(img, start, goal):
    img_with_markers = np.copy(img)
    for point, label in zip([start, goal], ['ini', 'fin']):
        cv2.drawMarker(img_with_markers, (int(point[0]), int(point[1])), (0, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=10, thickness=2)
        cv2.putText(img_with_markers, label, (int(point[0]) + 10, int(point[1]) + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    return img_with_markers

def main():
    img_path = f'./{"mapa.png"}'
    img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)

    step_size_cm = float(input("Ingrese el tamaño del paso (en cm): "))
    max_iterations = int(1000)
    #rewiring_radius_cm = float(input("Ingrese el radio de rewiring (en cm): "))
    diametro_robot = int(input("Ingrese el diametro del robot (en cm): "))

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
        cv2.drawMarker(img_with_path, (int(point[0]), int(point[1])), (0, 0, 255),markerType=cv2.MARKER_CROSS, markerSize=10, thickness=2)

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



