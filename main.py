# Main and helper function

from PIL import Image
import numpy as np
from CC_RRT_Star import CC_RRT_Star
from Obstacles import Obstacles
import sys
import matplotlib.pyplot as plt
import random

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

    # seed the RNG so that results are repeatable
    random.seed(5)

    # Load the map
    start = (250, 50)
    goal  = (30, 250)
    map_array = load_map("two_rooms.jpg", 0.3)

    # Planning class
    CC_RRT_planner = CC_RRT_Star(map_array, start, goal)
    CC_RRT_planner.init_map()
    CC_RRT_planner.setProbabilityThreshold(0.0001)

    # Obstacle data
    Obstacle_info = Obstacles()
    Obstacle_info.setMapSize(CC_RRT_planner.size_col, CC_RRT_planner.size_row)
    Obstacle_info.setObstacleSize(100, 100)
    Obstacle_info.setMaxVelocity(1)
    Obstacle_info.generateObstacles(10)
    CC_RRT_planner.setObstacleSource(Obstacle_info)

    # Do first couple of frames, and then print results
    # This lets us abort early if the map is badly set up
    iterations = 2
    for i in range(0, iterations):
        CC_RRT_planner.CC_RRT_star()
    CC_RRT_planner.draw_map(0)

    # Do the normal bunch of iterations
    iterations = 800
    for i in range(0, iterations):
        CC_RRT_planner.CC_RRT_star()

    # Print results
    CC_RRT_planner.print_conclusion()
    # Use this line if doing a static obstacle (velocity=0) solution
    # CC_RRT_planner.draw_map(0)

    # Save series of images
    # Use this if doing a dynamic obstacle solution
    for t in range(1, 401, 10):
        CC_RRT_planner.save_map(t, "output/time" + str(t) + ".png", 300)
