# Project for Motion Planning class

Kevin Wrona
Andrew Fabian
Favian Mendes
Harshavardhan Dammalapati

## Project Structure

The code for this project is split into two branches, representing the two student teams doing research for the project.  

The first branch, "BRRT" contains a Bidirectional RRT implentation, layered on top of Chance-Constrained RRT (CCRRT).  

The second branch, "MultiRRT" contains a codebase that can be configured to run multiple combinations of RRT algorithms, again building on top of CCRRT:
- Rewire (RRT*)
- Path Optimization (RRT*-Smart)
- Intelligent Sampling (RRT*-Smart)
- Informed RRT*

Additionally, the obstacle field can be configured (number of obstacles, velocity, and field size).  The main RRT* system's neighborhood size and iteration count can be configured.

## How to Run

To simply run either of the two codebases, switch to their directory and execute main.py:

    cd BRRT
    python main.py

or

    cd MultiRRT
    python main.py

## Configuring MultiRRT

All of the configurable options are in a block within MultiRRT/main.py, starting at line 43:

    # Planning class
    CC_RRT_planner = CC_RRT_Star(map_array, start, goal)
    CC_RRT_planner.init_map()
    CC_RRT_planner.setProbabilityThreshold(0.0001)
    CC_RRT_planner.setDoOriginalRewire(True)
    CC_RRT_planner.setDoBetterRewire(True)
    CC_RRT_planner.setUseIntelligentSampling(True)
    CC_RRT_planner.setUseInformed(True)
    CC_RRT_planner.setNeighborhoodSize(50)
    iterations = 1000

    # Obstacle data
    Obstacle_info = Obstacles()
    Obstacle_info.setMapSize(CC_RRT_planner.size_col, CC_RRT_planner.size_row)
    Obstacle_info.setObstacleSize(100, 100)
    Obstacle_info.setObstacleFieldSizeCoeff(1.0)
    Obstacle_info.setMaxVelocity(0)
    Obstacle_info.generateObstacles(8)
    CC_RRT_planner.setObstacleSource(Obstacle_info)

- setDoOriginalRewire() enables RRT*-style rewiring.
- setDoBetterRewire() enables RRT*-SMART path optimization.
- setUseIntelligentSample() enables RRT*-SMART intelligent sampling.
- setUseInformed() enables InformedRRT* sampling.

All options can be mixed and match.  Good results are obtained (though not good time efficiency) by enabling all options.