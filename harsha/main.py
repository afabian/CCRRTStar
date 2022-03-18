import random
import math
import pygame
import matplotlib.pyplot as plt
import numpy as np
import math
import random
import copy
import scipy.stats


class RRTMap:
	def __init__(self, start, goal, map_rows, map_cols, num_obstacles, obstacle_radius):
		self.start = start
		self.goal = goal
		self.map_rows = map_rows
		self.map_cols = map_cols
		self.num_obstacles = num_obstacles
		self.obstacle_radius = obstacle_radius

		# Initialze a pygame window 
		self.window_name = "Obstacle Avoidance RRT*"
		pygame.display.set_caption(self.window_name)
		self.map = pygame.display.set_mode((self.map_rows, self.map_cols))
		self.map.fill((255, 255, 255))

		# Define the colors
		self.red = (255, 0, 0)
		self.green = (0, 255, 0)
		self.gray = (70, 70, 70)



	def draw_map(self, obstacles):
		# Draw the starting and end nodes on the map along with the obstacles
		self.map.fill((255, 255, 255))
		pygame.draw.circle(self.map, self.green, self.start, 5)
		pygame.draw.circle(self.map, self.red, self.goal, 5)
		self.draw_obstacles(obstacles)
		pygame.display.update()


	def draw_path(self):
		pass

	def draw_obstacles(self, obstacles):
		obstaclesList = obstacles.copy()
		while(len(obstaclesList) > 0): 
			obstacle = obstaclesList.pop(0)
			pygame.draw.circle(self.map, self.gray, (obstacle.p_row, obstacle.p_col), self.obstacle_radius)



class RRTGraph: 
	def __init__(self, start, goal, map_rows, map_cols, num_obstacles, obstacle_radius):
		self.start = start
		self.goal = goal
		self.map_rows = map_rows
		self.map_cols = map_cols
		self.num_obstacles = num_obstacles
		self.obstacle_radius = obstacle_radius

		self.goal = False
		
		self.x = []
		self.y = []
		self.parent = []

		self.x.append(start[0])
		self.y.append(start[1])
		self.parent.append(None)

		self.obstacles = []
		self.path = []

	def get_random_circle_coordinates(self): 
		circle_x = int(random.uniform(0, self.map_rows-self.obstacle_radius))
		circle_y = int(random.uniform(0, self.map_cols-self.obstacle_radius))

		return (circle_x, circle_y)

	def make_obstacles(self):
		obstacles = [] 
		border_width = 0
		for i in range(self.num_obstacles): 
			coordinates = self.get_random_circle_coordinates()
			circle = (coordinates[0], coordinates[1])
			obstacles.append(circle)
		self.obstacles = obstacles.copy()
		return obstacles

	def add_node(self):
		pass

	def remove_node(self): 
		pass

class Obstacle:
    def __init__(self):
        self.p_row = 0 # position at time 0
        self.p_col = 0 # position at time 0
        self.v_row = 0 # velocity
        self.v_col = 0 # velocity
        self.s_row = 0 # size/variance
        self.s_col = 0 # size/variance

class Obstacles:
    def __init__(self, map_size_row, map_size_col, obstacle_size_row, obstacle_size_col, max_velocity, n_obstacles):
        self.map_size_row = map_size_row
        self.map_size_col = map_size_col
        self.obstacle_size_row = obstacle_size_row
        self.obstacle_size_col = obstacle_size_col
        self.max_velocity = max_velocity
        self.n_obstacles = n_obstacles
        self.obstacles = []

    def generateObstacles(self):
        for i in range(0, self.n_obstacles):
            obstacle = Obstacle()
            obstacle.p_row = random.randrange(0, self.map_size_col)
            obstacle.p_col = random.randrange(0, self.map_size_row)
            obstacle.v_row = (random.random() * 2 - 1) * self.max_velocity
            obstacle.v_col = (random.random() * 2 - 1) * self.max_velocity
            obstacle.s_row = random.random() * self.obstacle_size_col
            obstacle.s_col = random.random() * self.obstacle_size_row
            self.obstacles.append(obstacle)
        return self.obstacles

    def getObstaclesAtTime(self, t) -> "list[Obstacle]":
        output = []
        for obstacle in self.obstacles:
            obstacle_at_time = copy.copy(obstacle)
            obstacle_at_time.p_row += obstacle_at_time.v_row * t
            obstacle_at_time.p_col += obstacle_at_time.v_col * t
            output.append(obstacle_at_time)
        return output

    def getPDF(self, col, row, t) -> float:
        pdfsum = 0
        for obstacle in self.obstacles:
            pdf = scipy.stats.norm(obstacle.p_col, obstacle.s_col).pdf(col)
            pdfsum += pdf
        return pdfsum
