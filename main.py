# Main and helper function

from PIL import Image
import numpy as np
from CC_RRT_Star import CC_RRT_Star
from Obstacles import Obstacles
import sys
import matplotlib.pyplot as plt


def load_map(file_path, resolution_scale):
    ''' Load map from an image and return a 2D binary numpy array
        where 0 represents obstacles and 1 represents free space
    '''
    # Load the image with grayscale
    img = Image.open(file_path).convert('L')
    # Rescale the image
    size_x, size_y = img.size
    new_x, new_y  = int(size_x*resolution_scale), int(size_y*resolution_scale)
    img = img.resize((new_x, new_y), Image.ANTIALIAS)

    map_array = np.asarray(img, dtype='uint8')

    # Get bianry image
    threshold = 127
    map_array = 1 * (map_array > threshold)

    # Result 2D numpy array
    return map_array


if __name__ == "__main__":
    # Load the map
    start = (200, 75)
    goal  = (30, 250)
    map_array = load_map("WPI_map.jpg", 0.3)

    # Planning class
    CC_RRT_planner = CC_RRT_Star(map_array, start, goal)
    CC_RRT_planner.init_map()
    CC_RRT_planner.setProbabilityThreshold(0.5)

    # Obstacle data
    Obstacle_info = Obstacles()
    Obstacle_info.setMapSize(CC_RRT_planner.size_col, CC_RRT_planner.size_row)
    Obstacle_info.setObstacleSize(100, 100)
    Obstacle_info.setMaxVelocity(10)
    Obstacle_info.generateObstacles(20)
    CC_RRT_planner.setObstacleSource(Obstacle_info)

    # TEST CODE
    # print(Obstacle_info.getPDF(10, 10, 0))
    # print(Obstacle_info.getPDF(10, 10, 1))
    # print(Obstacle_info.getPDF(20, 10, 0))
    Obstacle_info.draw_map(100, 100, 0)
    sys.exit()

    # Do search
    iterations = 100
    for i in range(0, iterations):
        CC_RRT_planner.CC_RRT_star()

    # Print results
    CC_RRT_planner.print_conclusion()
    CC_RRT_planner.draw_map()