import matplotlib.pyplot as plt
import numpy as np
import math
from readchar import readkey, key

leader_velocity = 0.5  # Leader's velocity (units of displacement per second)
leader_trajectory = 'linear'  # 'linear', 'circular', 'manual'

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __str__(self):
        return f"x: {self.x}, y: {self.y}"

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def distance(self, node):
        return np.sqrt((self.x - node.x)**2 + (self.y - node.y)**2)

def select_trajectory():
    global leader_trajectory
    while True:
        print("Menu:")
        print("1. Linear trajectory")
        print("2. Circular trajectory")
        print("3. Manual")
        print("0. Exit")
        choice = input("Enter the number corresponding to your choice: ")

        if choice == "1":
            leader_trajectory = 'linear'
            break
        elif choice == "2":
            leader_trajectory = 'circular'
            break
        elif choice == "3":
            leader_trajectory = 'manual'
            break
        elif choice == "0":
            return False
        else:
            print("Invalid choice. Please select a valid option.")
            return False
    return True

def get_formation(leader, mode):
    if mode == 'line':
        return [
            leader,
            Node(leader.x, leader.y-1),
            Node(leader.x, leader.y-2),
            Node(leader.x, leader.y-3),
            Node(leader.x, leader.y-4),
        ]
    if mode == 'triangle':
        return [
            leader,
            Node(leader.x-1, -1),
            Node(leader.x+1, -1),
            Node(leader.x-2, -2),
            Node(leader.x, -2),
            Node(leader.x+2, -2),
        ]
    if mode == 'square':
        return [
            leader,
            Node(leader.x, leader.y-2),
            Node(leader.x-2, leader.y-2),
            Node(leader.x-2, leader.y),
        ]
    return [leader]

def get_distances(nodes):
    n = len(nodes)
    distance_matrix = np.zeros((n, n))

    for i in range(n):
        for j in range(n):
            distance_matrix[i][j] = nodes[i].distance(nodes[j])

    return distance_matrix

def plot_nodes(nodes, leader_index):
    plt.clf()
    for i, node in enumerate(nodes):
        marker = 'o' if i == leader_index else 'x'
        plt.plot(node.x, node.y, marker)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Movement of Nodes')
    plt.xlim(nodes[0].x - 10, nodes[0].x + 10)  # Set the x-axis limits
    plt.ylim(nodes[0].y - 10, nodes[0].y + 10)  # Set the y-axis limits
    #plt.xlim(-10, 10)
    #plt.ylim(-10, 10)
    plt.grid(False)  # Disable grid lines
    plt.pause(0.1)  # Pause for 0.1 seconds to show the plot
    plt.draw()

def updateLeader(leader):
    if leader_trajectory == 'manual':
        key_pressed = readkey()
        if key_pressed == 'q':
            return False
        elif key_pressed == key.UP:
            leader.y += leader_velocity
        elif key_pressed == key.DOWN:
            leader.y -= leader_velocity
        elif key_pressed == key.RIGHT:
            leader.x += leader_velocity
        elif key_pressed == key.LEFT:
            leader.x -= leader_velocity
    
    elif leader_trajectory == 'linear':
        leader.x += leader_velocity

    elif leader_trajectory == 'circular':
        radius = 5
        current_angle = math.atan2(leader.y, leader.x+radius)
        new_angle = current_angle + 0.1
        leader.x = (radius * np.cos(new_angle)) - radius
        leader.y = (radius * np.sin(new_angle))

    else:
        print('Invalid trajectory')
        return False
    return True

def attractive_force(i, nodes, distances):
    print('#####################')
    force = np.array([0,0])
    vector_i = np.array([nodes[i].x, nodes[i].y])

    for j in range(0,len(nodes)):
        if i == j:
            continue
        vector_j = np.array([nodes[j].x, nodes[j].y])

        print(f'vector {i}: {vector_i}')
        print(f'vector {j}: {vector_j}')
        print(f'({np.linalg.norm(vector_i - vector_j)**2} - {distances[i][j]**2}) * ({vector_j - vector_i}) = {(np.linalg.norm(vector_i - vector_j)**2 - distances[i][j]**2) * (vector_j - vector_i)}')
        force = force + (np.linalg.norm(vector_i - vector_j)**2 - distances[i][j]**2) * (vector_j - vector_i)
        print('-------------------')

    scaling_factor = 0.035
    print(f'force: {force}')
    print(f'force scaled: {scaling_factor * force}')
    print(f'node before: {nodes[i]}')
    nodes[i].x = nodes[i].x + scaling_factor * force[0]
    nodes[i].y = nodes[i].y + scaling_factor * force[1]
    print(f'node after: {nodes[i]}')
    print('-------------------')
    print('#####################')

def repulsive_force(node, other_node, distance):
    print("TODO")

def simulate_movement(nodes, distances):
    plt.ion()  # Turn on interactive mode for live plot

    plot_nodes(nodes, leader_index=0)
    while True:
        res = updateLeader(nodes[0])
        if not res:
            break

        for i in range(1,len(nodes)):
            attractive_force(i, nodes, distances)

            """ for j in range(1,len(nodes)):
                  if i != j:
                     repulsive_force(nodes[i], nodes[j], distances[i][j]) """

        plot_nodes(nodes, leader_index=0)

nodes = get_formation(Node(0, 0), 'triangle')

""" if select_trajectory():
    distances = get_distances(nodes)
    simulate_movement(nodes, distances) """

leader_trajectory = 'manual'
distances = get_distances(nodes)
simulate_movement(nodes, distances)