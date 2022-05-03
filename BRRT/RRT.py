#!/usr/bin/python
# -*- coding: utf-8 -*-
# Standard Algorithm Implementation
# Sampling-based Algorithms RRT and RRT*

import matplotlib.pyplot as plt
import numpy as np
import random
import math
from scipy import spatial
from Obstacles import *
# from test import Obstacles_info


# Class for each tree node

class Node:

    def __init__(
        self,
        row,
        col,
        parent=None,
        cost=0.0,
        ):
        self.row = row  # coordinate
        self.col = col  # coordinate
        self.parent = parent  # parent node
        self.cost = cost  # cost


# Class for RRT

class RRT:

    # Constructor

    def __init__(
        self,
        map_array,
        start,
        goal,
        ):
        self.map_array = map_array  # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]  # map size
        self.size_col = map_array.shape[1]  # map size

        self.start = Node(start[0], start[1])  # start node
        self.start_point = start
        self.goal = Node(goal[0], goal[1])  # goal node
        self.goal_point = goal
        self.vertices1 = []  # list of nodes
        self.vertices2 = []
        self.points1 = [start]
        self.points2 = [goal]
        self.found = False  # found flag
        self.cost = 0
        
        self.distance = 10  #Extend Distance
        self.probability_threshold = 0.001

    def init_map(self):
        '''Intialize the map before each search
        '''

        self.found = True
        self.vertices1 = []
        self.vertices1.append(self.start)
        self.vertices2 = []
        self.vertices2.append(self.goal)
        self.points1 = []
        self.points2 = []

    def setObstacleSource(self, obstacles):
        # obstacles should be a class that contains a getPDF(x, y, t) method
        self.obstacles = obstacles

    def dis(self, node1, node2):
        '''Calculate the euclidean distance between two nodes
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            euclidean distance between two nodes
        '''

        return math.pow(math.pow(node1.row - node2.row, 2)
                        + math.pow(node1.col - node2.col, 2), 0.5)

    def check_collision(self, node1, node2):
        '''Check if the path between two nodes collide with obstacles
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            True if the new node is valid to be connected
        '''

        x1 = node1.row
        x2 = node2.row
        y1 = node1.col
        y2 = node2.col
        angle = math.atan2(y2 - y1, x2 - x1)
        path = np.linspace([x1, y1], [x2, y2], num=10, dtype=int)
        for point in path:
            if self.map_array[point[0]][point[1]] == 0:
                return True
        return False

    def get_new_point(self):
        '''Choose the goal or generate a random point
        arguments:
            goal_bias - the possibility of choosing the goal instead of a random point

        return:
            point - the new point
        '''

        return (random.randint(0, self.size_row), random.randint(0,
                self.size_col))

    def get_nearest_node_from_start(self, point):
        '''Find the nearest node in self.vertices1 with respect to the new point
        arguments:
            point - the new point

        return:
            the nearest node
        '''

        nearest_point = None
        low = math.inf
        for v_node in self.vertices1:
            if self.dis(Node(point[0], point[1]), v_node) < low:
                nearest_point = v_node
                low = self.dis(Node(point[0], point[1]), v_node)
        return nearest_point

    def get_nearest_node_from_goal(self, point):
        '''Find the nearest node in self.vertices1 with respect to the new point
        arguments:
            point - the new point

        return:
            the nearest node
        '''

        nearest_point = None
        low = math.inf
        for v_node in self.vertices2:
            if self.dis(Node(point[0], point[1]), v_node) < low:
                nearest_point = v_node
                low = self.dis(Node(point[0], point[1]), v_node)
        return nearest_point



    # def retrace_path(self, dict_parent, curr_var, sstart):
    #     # path list storing the trace of path from
    #     # start to goal
    #     path = [curr_var]
    #     while curr_var != sstart:
    #         for var in dict_parent:
    #             # print(var)
    #             if var == curr_var:
    #                 # print((curr_var.row,curr_var.col))
                    
    #                 curr_var = dict_parent[var]
    #                 path.append(curr_var)
    #             elif curr_var == sstart:
    #                 path.append(sstart)
    #                 # print((curr_var.row,curr_var.col))                   
    #                 return path[::-1]
    #             # return path

    def retrace_path(self,dict_parent, curr_var, sstart):
        path = [curr_var]
        while dict_parent[curr_var] is not None:
            # print((curr_var.row,curr_var.col))
            parent = dict_parent[curr_var]
            path.append(parent)
            curr_var = parent
            if curr_var == sstart:
                break
        # path.append(sstart)
        return path                  

    def extend(self,node,point,distance):
        angle = math.atan2(point[1] - node.col, point[0] - node.row)
        new_row = round(node.row + distance * np.cos(angle))
        new_col = round(node.col + distance * np.sin(angle))

        if new_row > self.size_row-1:
            new_row = self.size_row-1
        elif new_row < 0:
            new_row = 0
        
        if new_col > self.size_col-1:
            new_col = self.size_col-1
        elif new_col < 0:
            new_col = 0


        extension_point = Node(new_row, new_col) 
        
        return extension_point



    def line_crosses_moving_obstacles(self, node1: Node, node2: Node) -> bool:
        # steps = int(self.distance(node1, node2) * 1)
        steps = 25
        for step in range(0, steps):
            temp_node = Node(
                round(node1.row + (step/steps) * (node2.row-node1.row)),
                round(node1.col + (step/steps) * (node2.col-node1.col)))

            t = node1.cost + self.dis(node1, temp_node)
            pdf = self.obstacles.getPDF(temp_node.col, temp_node.row, t)
            if pdf > self.probability_threshold:
                return True
        return False
    

    def RRT(self, n_pts=1000):
        '''RRT main search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        In each step, extend a new node if possible, and check if reached the goal
        '''

        # Remove previous result

        self.init_map()

        # ## YOUR CODE HERE ###
        path1 = None
        path2 = None
        # In each step,
        # get a new point,
        distance = 10
        dict_parent = {}
        count = 0
        extension_point = None
        (fig, ax) = plt.subplots(1)
        img = 255 * np.dstack((self.map_array, self.map_array,
                              self.map_array))
        ax.imshow(img)

        Obstacles_info = Obstacles()

        for i in range(n_pts):
            point = self.get_new_point()
            node_point = Node(point[0], point[1])
            temp_node_start = self.get_nearest_node_from_start(point)
            temp_node_goal = self.get_nearest_node_from_goal(point)

            if i == 0:   
                Obstacles_info.setMapSize(self.size_col, self.size_row)
                Obstacles_info.setObstacleSize(100, 100)
                Obstacles_info.setMaxVelocity(0)
                moving_obstacles = Obstacles_info.generateObstacles(20)
                self.setObstacleSource(Obstacles_info)
            else:
                moving_obstacles = Obstacles_info.getObstaclesAtTime(i/100)

            obstacleList_moving =[]
            for obstacle in moving_obstacles:
                obstacleList_moving.append((obstacle.p_row, obstacle.p_col, obstacle.s_row, obstacle.s_col))
                # plt.plot(obstacle.p_col, obstacle.p_row, color='grey', marker='o', markersize = 5)

            # print(len(obstacleList_moving))

            if self.dis(temp_node_start,node_point) < self.dis(temp_node_goal,node_point):
                extension_point = self.extend(temp_node_start,point,self.distance)
    
                node = temp_node_start
                if not self.check_collision(node,extension_point):

                    self.vertices1.append(extension_point)
                    extension_point.parent = node
                    extension_point.cost = round(extension_point.parent.cost + self.dis(node,extension_point))

                    if not self.line_crosses_moving_obstacles(node,extension_point):
                        # print("Entered Path")
                        self.points1.append((extension_point.row,extension_point.col))
                        plt.plot([node.col, extension_point.col], [node.row, extension_point.row], color = 'orange')
                        plt.plot(extension_point.col, extension_point.row, color='orange', marker='o', markersize = 2)

                        dict_parent[extension_point] = node
                    # dict_parent[(extension_point.row,extension_point.col)] = (node.row, node.col)



            else:
                extension_point = self.extend(temp_node_goal,point,self.distance)
                node = temp_node_goal
                if not self.check_collision(node,extension_point):
                    extension_point.parent = node
                    extension_point.cost = round(extension_point.parent.cost + self.dis(node,extension_point))
                    if not self.line_crosses_moving_obstacles(node,extension_point):
                        self.vertices2.append(extension_point)
                        self.points2.append((extension_point.row,extension_point.col))
                        plt.plot([node.col, extension_point.col], [node.row, extension_point.row], color = 'white')
                        plt.plot(extension_point.col, extension_point.row, color='white', marker='o', markersize = 2)
                        
                        dict_parent[extension_point] = node
                    # dict_parent[(extension_point.row,extension_point.col)] = (node.row, node.col)

                    # if _%10 == 0:
                    #     for node in self.vertices1:
                    #         if self.dis(extension_point,node) < self.distance:
                    #             self.found = True
                    #             path_goal = self.retrace_path(dict_parent,extension_point, self.goal)
                    #             path_start = self.retrace_path(dict_parent,node,self.start)
                    #             print(path_start)
                    #             print(path_goal)
                    #             path1 = path_start.extend(path_goal[::-1])
                    #             break
   
            # if i%25 == 0:
            #     all_points = self.points1+self.points2
            #     # print(all_points)
            #     kdtree = spatial.KDTree(all_points)
            #     temp_pairs = kdtree.query_pairs(self.distance,p=30,eps=0,output_type='ndarray')
            #     # temp_pairs = kdtree.query_ball_point(10,p=30,eps=0,)
            #     temp_pairs = np.ndarray.tolist(temp_pairs)
            #     #print(pairs)
            #     pairs = []
            #     # while self.found ==False:
            #     for pair in temp_pairs:
            #         point1_idx = pair[0]
            #         point2_idx = pair[1]
            #         # print(point1_idx)
            #         # print(point2_idx)

            #         if point1_idx < len(self.points1)-1 and point2_idx > len(self.points1)-1 :

            if i%25 == 0:
                # print(self.vertices1)
                # print(len(self.vertices2))
                for node1 in reversed(self.vertices1):
                    for node2 in reversed(self.vertices2):

                        # print((node1.row,node1.col))
                        # print((node2.row,node2.col))
                        # print(self.dis(node1,node2))
                        if self.dis(node1,node2) < self.distance+20:
                            # print('Inside the loop')                
                            if not self.check_collision(node1,node2):
                                # point1 = all_points[point1_idx]
                                # point2 = all_points[point2_idx]
                                # point2_idx = len(all_points)+1-pair[1]
                                # print(point1)
                                # print((self.vertices1[point1_idx].row,self.vertices1[point1_idx].col))
                                # print(point2)

                                # print((self.vertices2[point2_idx].row,self.vertices2[point2_idx].col))
                                # dict_parent[point2] = Node(point1[0], point1[1])
                                # dict_parent[node1] = node2
                                try:
                                    path_start = self.retrace_path(dict_parent,node1,self.start)
                                    path_goal = self.retrace_path(dict_parent,node2, self.goal)
                                except:
                                    continue
                                # print(dict_parent)
                                # path_start = self.retrace_path(dict_parent,all_points[point1_idx],self.start_point)
                                # path_goal = self.retrace_path(dict_parent,all_points[point2_idx], self.goal_point)
                                path1 = path_start[::-1]+path_goal
                                self.goal.cost = node1.cost + node2.cost + round(self.dis(node1,node2))
                                # if not self.check_collision(Node(point1[0],point1[1]),Node(point2[0],point2[1])):
                                self.found = True   
                                break

                    if self.found:
                        break
                        
                    # elif point1_idx > len(self.points1)-1 and point2_idx < len(self.points1)-1:
                    #         self.found = True
                    #         point1 = self.points1[point2_idx]
                    #         point2 = self.points2[point1_idx]
                    #         dict_parent[point2] = Node(point1[0], point1[1])
                    #         path_goal = self.retrace_path(dict_parent,node, self.goal)
                    #         path_start = self.retrace_path(dict_parent,extension_point,self.start)
                    #         path1 = path_start+path_goal[::-1]         
            # if _%25 == 0:
            #     for node1 in self.vertices1:
            #         for node2 in self.vertices2:
            #             if self.dis(node1,node) < self.distance:
            #                 self.found = True
            #                 print(self.found)
            #                 path_goal = self.retrace_path(dict_parent,node, self.goal)
            #                 path_start = self.retrace_path(dict_parent,extension_point,self.start)
            #                 print(path_start)
            #                 print(path_goal)
            #                 path1 = path_start.extend(path_goal[::-1])
            #                 break
                        
            #     for nodes in self.vertices1:
                    
            
            
            

            # if not self.check_collision(extension_point, node):
            #     self.vertices1.append(extension_point)
            #     plt.plot([node.col, extension_point.col], [node.row, extension_point.row], color = 'orange')
            #     extension_point.parent = node
            #     dict_parent[extension_point] = node

            # for nodes in self.vertices1: 
            #     if self.dis(node2, extension_point) < 2:
            #         plt.plot([node2.col, extension_point.col], [node2.row, extension_point.row], color= 'orange')

                    # if count%2 == 0:
                    #     path1 = self.retrace_path(dict_parent, extension_point, self.start)
                    #     path2 = self.retrace_path(dict_parent, node2, self.goal)

                    # if count%2 == 1:
                    #     path1 = self.retrace_path(dict_parent, node2, self.start)
                    #     path2 = self.retrace_path(dict_parent, extension_point, self.goal)

                    # path1.extend(path2[::-1])
                    # self.vertices1.extend(self.vertices2)
                    # break


            # self.vertices1, self.vertices2 = self.vertices2[:], self.vertices1[:]

            # count += 1

        if not path1:
            self.vertices1.extend(self.vertices2)
            self.found = False


        
        # for var in self.vertices1:
        #     plt.plot(var.col, var.row, color='orange', marker='o', markersize = 2)
        # for var in self.vertices2:
        #     plt.plot(var.col, var.row, color='orange', marker='o', markersize = 2)


        # Plotting the final path and the nodes from start to goal
        # print(self.found)
        # print(path1)
        # for obstacle in moving_obstacles:
            # plt.plot(obstacle.p_col, obstacle.p_row, color='grey', marker='o', markersize = 10)

        if self.found:
            for i in range(1,len(path1)):
                plt.plot(path1[i].col, path1[i].row, color='green', marker='o', markersize = 1)
                plt.plot([path1[i].col, path1[i-1].col],[path1[i].row, path1[i-1].row], color = 'green')

        plt.plot(self.start.col, self.start.row, color='green', marker='o')
        plt.plot(self.goal.col, self.goal.row, color='red', marker='o')
     
        imgfixed = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        imgpdf = self.obstacles.get_pdf_map(80, 80, 0)
        imgpdf = scipy.ndimage.zoom(imgpdf, len(imgfixed) / len(imgpdf), order=3)

        # fig, ax = plt.subplots(1)
        
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

        plt.show()
        plt.savefig("plot", dpi=300)
         
        if self.found:
            steps = len(self.vertices1) - 2
            length = self.goal.cost
            print('It took %d nodes to find the current path' % steps)
            print('The path length is %.2f' % length)
        else:
            print('No path found with the specified parameters. Either adjust the parameters or try again')


