# Chance-Constrained RRT*
# Preliminary Implementation for Project Proposal
# 2022-03-18

import matplotlib.pyplot as plt
import numpy as np
import math
import random
import scipy

# Class for each tree node
class Node:
    def __init__(self, row, col):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.parent = None    # parent node
        self.cost = 0.0       # cost

    def __add__(self, other):
        return Node(self.row + other.row, self.col + other.col)

    def __sub__(self, other):
        return Node(self.row - other.row, self.col - other.col)

    def __mul__(self, scalar):
        return Node(self.row * scalar, self.col * scalar)

    def __truediv__(self, scalar):
        return Node(self.row / scalar, self.col / scalar)


# Class for CC_RRT_Star
class CC_RRT_Star:
    # Constructor
    def __init__(self, map_array, start: tuple, goal: tuple):
        self.map_array = map_array # 1->free, 0->obstacle
        self.size_row = map_array.shape[0]
        self.size_col = map_array.shape[1]

        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.vertices: list[Node] = []
        self.found = False
        self.current_path:list[Node] = []

        self.goal_threshold = 10
        self.probability_threshold = 0
        self.do_original_rewire = False
        self.do_better_rewire = True
        self.use_intelligent_sampling = False
        self.use_informed = False

        self.neighborhood_size = 20


    def init_map(self):
        '''Intialize the map before each search
        '''
        self.found = False
        self.vertices = []
        self.vertices.append(self.start)
    

    def setDoBetterRewire(self, value):
        self.do_better_rewire = value

    def setDoOriginalRewire(self, value):
        self.do_original_rewire = value

    def setUseIntelligentSampling(self, value):
        self.use_intelligent_sampling = value

    def setUseInformed(self, value):
        self.use_informed = value

    def setProbabilityThreshold(self, probability_threshold):
        self.probability_threshold = probability_threshold

    def setNeighborhoodSize(self, size):
        self.neighborhood_size = size
        
    def setObstacleSource(self, obstacles):
        # obstacles should be a class that contains a getPDF(x, y, t) method
        self.obstacles = obstacles

    def distance(self, node1: Node, node2: Node) -> float:
        '''Calculate the euclidean distance between two nodes
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            euclidean distance between two nodes
        '''
        return math.hypot(node2.row - node1.row, node2.col - node1.col)
    

    def line_crosses_map_obstacles(self, node1: Node, node2: Node) -> bool:
        '''Check if the path between two nodes collide with obstacles
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            True if the new node is valid to be connected
        '''
        steps = int(self.distance(node1, node2) * 2)
        for step in range(0, steps):
            row = round(node1.row + (step/steps) * (node2.row-node1.row))
            col = round(node1.col + (step/steps) * (node2.col-node1.col))
            if self.map_array[row, col] == 0:
                return True
        return False


    def line_crosses_moving_obstacles(self, node1: Node, node2: Node) -> bool:
        # node1 must be the node from the earlier point in time
        steps = int(self.distance(node1, node2) * 1)
        for step in range(0, steps):
            temp_node = Node(
                round(node1.row + (step/steps) * (node2.row-node1.row)),
                round(node1.col + (step/steps) * (node2.col-node1.col)))
            t = node1.cost + self.distance(node1, temp_node)
            pdf = self.obstacles.getPDF(temp_node.col, temp_node.row, t)
            if pdf > self.probability_threshold:
                return True
        return False


    def get_new_point(self, goal_bias) -> Node:
        '''Choose the goal or generate a random point
        arguments:
            goal_bias - the possibility of choosing the goal instead of a random point

        return:
            point - the new point
        '''
        if self.use_intelligent_sampling:
            return self.get_new_point_intelligent_sampling(goal_bias)
        else:
            if self.found or not self.use_informed:
                return self.get_new_point_in_ellipsoid(goal_bias, self.goal.cost)
            else:
                return self.get_new_point_normal(goal_bias)


    def get_new_point_in_ellipsoid(self, goal_bias, c_best):
        '''Choose the goal or generate a random point in an ellipsoid
           defined by start, goal and current best length of path
        arguments:
            goal_bias - the possibility of choosing the goal instead of a random point
            c_best - the length of the current best path

        return:
            point - the new point
        '''
        # Select goal
        if np.random.random() < goal_bias:
            point = [self.goal.row, self.goal.col]

        # Generate a random point in an ellipsoid
        else:
            # Compute the distance between start and goal - c_min
            c_min = self.distance(self.start, self.goal)

            # Calculate center of the ellipsoid - x_center
            x_center = (self.start + self.goal) / 2

            # Compute rotation matrix from elipse to world frame - C
            theta = math.atan2(self.goal.col - self.start.col, self.goal.row - self.start.row)
            c, s = np.cos(theta), np.sin(theta)
            C = np.array(((c, -s), (s, c)))

            # Compute diagonal matrix - L
            r1 = c_best / 2
            r2 = math.sqrt(c_best ** 2 - c_min ** 2) / 2
            L = np.diag([r1, r2])

            # Cast a sample from a unit ball - x_ball
            row, col = np.random.uniform(-1, 1), np.random.uniform(-1, 1)
            while math.hypot(row, col) > 1:
                row, col = np.random.uniform(-1, 1), np.random.uniform(-1, 1)
            x_ball = [row, col]
            
            # Map ball sample to the ellipsoid - x_rand
            x_rand = np.dot(np.dot(C, L), x_ball)
            x_rand += [x_center.row, x_center.col]
            point = x_rand

        row = max(0, min(self.size_row-1, int(point[0])))
        col = max(0, min(self.size_col-1, int(point[1])))
        return Node(row, col)


    def get_new_point_normal(self, goal_bias) -> Node:
        use_goal = random.random() < goal_bias
        if use_goal:
            return self.goal
        else:
            return Node(random.randint(0, self.size_row-1), random.randint(0, self.size_col-1))


    def get_new_point_intelligent_sampling(self, goal_bias) -> Node:
        # randomly decide if we're going to do an intelligent or normal sample
        use_biased_sample = (random.randint(0, 100) > 50) if self.found else False

        # pick a node on the current solution path, and sample around that node
        if use_biased_sample:
            index = random.randint(1, len(self.current_path)-1)
            center = self.current_path[index]
            row = max(0, min(self.size_row-1, center.row + random.randint(-20, 20)))
            col = max(0, min(self.size_col-1, center.col + random.randint(-20, 20)))
            return Node(row, col)

        # do normal sampling
        else:
            return self.get_new_point_normal(goal_bias)


    def get_nearest_node(self, node: Node) -> Node:
        '''Find the nearest node in self.vertices with respect to the new node
        arguments:
            node - the new node

        return:
            the nearest node
        '''
        best_distance = math.inf
        best_node = None
        for vertice in self.vertices:
            distance = self.distance(node, vertice)
            if distance < best_distance:
                best_distance = distance
                best_node = vertice
        return best_node


    def get_neighbors(self, node: Node, neighbor_size: float) -> "list[Node]":
        '''Get the neighbors that are within the neighbor distance from the node
        arguments:
            node - a new node
            neighbor_size - the neighbor distance

        return:
            neighbors - a list of neighbors that are within the neighbor distance 
        '''
        output = []
        for vertice in self.vertices:
            if self.distance(node, vertice) < neighbor_size:
                output.append(vertice)
        return output


    def rewire(self, new_node: Node, neighbors: "list[Node]"):
        '''Rewire the new node and all its neighbors
        arguments:
            new_node - the new node
            neighbors - a list of neighbors that are within the neighbor distance from the node

        Rewire the new node if connecting to a new neighbor node will give least cost.
        Rewire all the other neighbor nodes.
        '''
        for node in neighbors:
            if node.cost < new_node.parent.cost:
                if not self.line_crosses_map_obstacles(node, new_node):
                    if not self.line_crosses_moving_obstacles(node, new_node):
                        new_node.parent = node
                        new_node.cost = new_node.parent.cost + self.distance(new_node, new_node.parent)

    def rewire_solution(self):
        node = self.goal
        while node != self.start and node is not None:
            self.rewire_solution_node(node)
            node = node.parent

    def rewire_solution_node(self, node):
            parent = node.parent
            if parent is not None:
                    parent_parent = parent.parent
                    if parent_parent is not None:
                        parent_parent_edge_crosses_map_obstacles = self.line_crosses_map_obstacles(node, parent_parent)
                        parent_parent_edge_crosses_moving_obstacles = self.line_crosses_moving_obstacles(parent_parent, node)
                        parent_parent_edge_ok = not parent_parent_edge_crosses_map_obstacles and not parent_parent_edge_crosses_moving_obstacles
                        if parent_parent_edge_ok:
                            node.parent = parent_parent
                            node.cost = parent_parent.cost + self.distance(node, parent_parent)
                            self.rewire_solution_node(node)


    
    def draw_map(self, t, draw_graph=True):
        plt = self.get_map(t, draw_graph)
        plt.show()


    def save_map(self, t, filename, dpi, draw_graph=True):
        plt = self.get_map(t, draw_graph)
        plt.savefig(filename, dpi=dpi)
        plt.close('all')


    def get_map(self, t, draw_graph=True):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots(1)

        # Map of the fixed obstacles
        imgfixed = 255 * np.dstack((self.map_array, self.map_array, self.map_array))

        # Map of the moving probabilistic obstacles
        imgpdf = self.obstacles.get_pdf_map(80, 80, t)
        imgpdf = scipy.ndimage.zoom(imgpdf, len(imgfixed) / len(imgpdf), order=3)

        # Merge and show the two maps
        maxpdf = np.amax(imgpdf)
        img = []
        for x in range(0, len(imgpdf)):
            imgx = []
            for y in range(0, len(imgpdf[0])):
                if imgfixed[x][y][0] == 0:
                    imgx.append(maxpdf)
                else:
                    imgx.append(imgpdf[x][y])
            img.append(imgx)

        ax.imshow(img)

        # Draw Trees or Sample points, if this is a static-time image
        if draw_graph:
            for node in self.vertices[1:-1]:
                plt.plot(node.col, node.row, markersize=3, marker='o', color='w')
                plt.plot([node.col, node.parent.col], [node.row, node.parent.row], color='w', linewidth=1)
        
        # Draw Final Path if found, if this is a static-time image
        if draw_graph:
            if self.found:
                cur = self.goal
                while cur.col != self.start.col or cur.row != self.start.row:
                    plt.plot([cur.col, cur.parent.col], [cur.row, cur.parent.row], color='g', linewidth=3)
                    cur = cur.parent
                    plt.plot(cur.col, cur.row, markersize=3, marker='o', color='g')

        # Draw the current point if this is a specific-time image
        if not draw_graph:
            point_at_time = self.get_point_at_time(t)
            plt.plot(point_at_time.col, point_at_time.row, markersize=10, marker='o', color='w')    

        # Draw start and goal
        plt.plot(self.start.col, self.start.row, markersize=5, marker='o', color='g')
        plt.plot(self.goal.col, self.goal.row, markersize=5, marker='o', color='r')

        return plt


    def get_point_at_time(self, t):
        path = []
        cur = self.goal
        path.append(cur)
        while cur.col != self.start.col and cur.row != self.start.row:
            cur = cur.parent
            path.append(cur)
        path.reverse()

        distance = 0
        for i in range(0, len(path)-1):
            distance_to_next = self.distance(path[i], path[i+1])
            if distance + distance_to_next > t:
                interp_position = (t - distance) / distance_to_next
                node = Node(
                    path[i].row * (1 - interp_position) + path[i+1].row * interp_position,
                    path[i].col * (1 - interp_position) + path[i+1].col * interp_position)
                return node
            else:
                distance += distance_to_next
        return self.goal


    def update_current_path(self):
        self.current_path = []
        node = self.goal
        while node != self.start:
            node = node.parent
            if node != self.start: self.current_path.append(node)


    def CC_RRT_star(self, neighbor_size=20):
        '''RRT* search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            neighbor_size - the neighbor distance
        
        In each step, extend a new node if possible, and rewire the node and its neighbors
        '''

        # In each step,
        # get a new point, 
        # get its nearest node, 
        # extend the node and check collision to decide whether to add or drop,
        # if added, rewire the node and its neighbors,
        # and check if reach the neighbor region of the goal if the path is not found.
        new_node = self.get_new_point(0)
        nearest_node = self.get_nearest_node(new_node)
        if not self.line_crosses_map_obstacles(new_node, nearest_node):
            if not self.line_crosses_moving_obstacles(new_node, nearest_node):
                new_node.parent = nearest_node
                new_node.cost = new_node.parent.cost + self.distance(new_node, new_node.parent)
                self.vertices.append(new_node)
                neighbors = self.get_neighbors(new_node, self.neighborhood_size)
                if self.do_original_rewire:
                    self.rewire(new_node, neighbors)
                if self.distance(new_node, self.goal) < self.goal_threshold:
                    self.found = True
                    self.goal.parent = new_node.parent
                    self.goal.cost = new_node.cost
                    self.update_current_path()
                    if self.do_better_rewire:
                        self.rewire_solution()

    def print_conclusion(self):
        # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("It took %d nodes to find the current path" %steps)
            print("The path length is %.2f" %length)
        else:
            print("No path found")

