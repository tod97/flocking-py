# Automation Lab

This project aims to simulate the movements of a small group of nodes that follow a leader maintaining a specific formation with relative distances.

## Installation

To be able to run this code, you need to use python 3. Furthermore, you need to install all the imports in the main.py (like readchar)

## Features

- The possible leader's trajectories area 'linear', 'circular' or 'manual'
    - In 'manual' mode, to move the leader, you can use the arrow keys in the console you're running the script. If you want to simulate a stationary situation just press any other keys.
    - The user can select the trajectory with the function select_trajectory (commented at the end of the main.py)
- In the plot_nodes function you can select the camera options so that you can follow or not the nodes
- At the end of the file you can choose to connect all the nodes to each other or to connect them based on the default formation.
- In the get_formation function you can select one of the available formations: 'point', 'line', 'triangle', 'square' (every formation defines the scaling_factors, the distances and the nodes)