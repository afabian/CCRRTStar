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
            obstacle.p_row = random.randrange(-self.map_size_col, 2*self.map_size_col)
            obstacle.p_col = random.randrange(-self.map_size_row, 2*self.map_size_row)
            obstacle.v_row = (random.random() * 2 - 1) * self.max_velocity
            obstacle.v_col = (random.random() * 2 - 1) * self.max_velocity
            obstacle.s_row = (random.random() * 0.8 + 0.2) * self.obstacle_size_col
            obstacle.s_col = (random.random() * 0.8 + 0.2) * self.obstacle_size_row
            self.obstacles.append(obstacle)

    def getObstaclesAtTime(self, t) -> "list[Obstacle]":
        output = []
        for obstacle in self.obstacles:
            obstacle_at_time = copy.copy(obstacle)
            obstacle_at_time.p_row += obstacle_at_time.v_row * t
            obstacle_at_time.p_col += obstacle_at_time.v_col * t
            output.append(obstacle_at_time)
        return output

    def paper_pdf(self, cov, pos, expect):
        numerator = np.exp(-1/2 * np.matmul(np.matmul(np.transpose((pos-expect)), np.linalg.inv(cov)), (pos-expect)))[0][0]
        denominator = (2 * np.pi) * (float(np.linalg.det(cov))**0.5)
        result = numerator/denominator
        return result

    def getPDF(self, col, row, t) -> float:
        pdfsum = 0
        for obstacle in self.obstacles:
            p_row_at_t = obstacle.p_row + obstacle.v_row * t
            p_col_at_t = obstacle.p_col + obstacle.v_col * t
            cov = np.array([[obstacle.s_col,0],[0,obstacle.s_row]])
            expect = np.array([[p_col_at_t], [p_row_at_t]])
            pos = np.array([[col], [row]])
            pdf = self.paper_pdf(cov, pos, expect) 
            pdfsum += pdf
        return pdfsum

    def get_pdf_map(self, col_resolution, row_resolution, t):
        img = []
        for col_index in range(0, col_resolution):
            col = col_index / col_resolution * self.map_size_col
            img_col = []
            for row_index in range(0, row_resolution):
                row = row_index / row_resolution * self.map_size_row
                img_col.append(self.getPDF(row, col, t))
            img.append(img_col)
        return img

    def draw_pdf_map(self, col_resolution, row_resolution, t):
        img = self.get_pdf_map()
        fig, ax = plt.subplots(1)
        ax.imshow(img)
        plt.show()