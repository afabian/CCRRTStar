from main import RRTMap
from main import RRTGraph
from main import Obstacle
from main import Obstacles
import pygame
import time
def main(): 
	grid_rows = 500
	grid_cols = 500
	start = (50, 50)
	goal = (275, 300)
	obstacle_radius = 20
	num_obstacles = 20
	max_velocity = 10

	pygame.init()
	map_main = RRTMap(start, goal, grid_rows, grid_cols, num_obstacles, obstacle_radius)
	graph_main = RRTGraph(start, goal, grid_rows, grid_cols, num_obstacles, obstacle_radius)
	obstacle_info = Obstacles(grid_rows, grid_cols, obstacle_radius, obstacle_radius, max_velocity, num_obstacles)


	obstacles = obstacle_info.generateObstacles()
	map_main.draw_map(obstacles)
	counter = 1
	while True:
		obstacles = obstacle_info.getObstaclesAtTime(counter)
		map_main.draw_map(obstacles)
		pygame.display.update()
		counter += 1
		time.sleep(1)

	pygame.event.clear()
	pygame.event.wait(0)

if __name__ == '__main__': 
	main()
