# Simulated Probabilistic Obstacles

import matplotlib.pyplot as plt
import numpy as np
import math
import random
import copy
import scipy.stats

# Type for a single obstacle
class Obstacle:
    def __init__(self):
        self.p_row = 0 # position at time 0
        self.p_col = 0 # position at time 0
        self.v_row = 0 # velocity
        self.v_col = 0 # velocity
        self.s_row = 0 # size/variance
        self.s_col = 0 # size/variance

# Class that creates and computes the position / PDF of a set of obstacles
class Obstacles:
    def __init__(self):
        self.map_size_col = 0
        self.map_size_row = 0
        self.obstacle_size_col = 0
        self.obstacle_size_row = 0
        self.max_velocity = 0
        self.n_obstacles = 0
        self.obstacles:list[Obstacle] = []

    def setMapSize(self, map_size_col, map_size_row):
        self.map_size_col = map_size_col
        self.map_size_row = map_size_row

    def setObstacleSize(self, obstacle_size_col, obstacle_size_row):
        self.obstacle_size_col = obstacle_size_col
        self.obstacle_size_row = obstacle_size_row

    def setMaxVelocity(self, max_velocity):
        self.max_velocity = max_velocity

    def generateObstacles(self, n_obstacles):
        self.n_obstacles = n_obstacles
        self.obstacles = []
        for i in range(0, self.n_obstacles):
            obstacle = Obstacle()
            obstacle.p_row = random.randrange(0, self.map_size_col)
            obstacle.p_col = random.randrange(0, self.map_size_row)
            obstacle.v_row = (random.random() * 2 - 1) * self.max_velocity
            obstacle.v_col = (random.random() * 2 - 1) * self.max_velocity
            obstacle.s_row = random.random() * self.obstacle_size_col
            obstacle.s_col = random.random() * self.obstacle_size_row
            self.obstacles.append(obstacle)

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