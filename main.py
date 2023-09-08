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
        nodes = [leader, Node(leader.x, leader.y-1), Node(leader.x, leader.y-2), Node(leader.x, leader.y-3), Node(leader.x, leader.y-4)]
        distance_matrix = np.zeros((len(nodes), len(nodes)))
        set_distance(distance_matrix, 0, 1, nodes[0].distance(nodes[1]))
        set_distance(distance_matrix, 1, 2, nodes[1].distance(nodes[2]))
        set_distance(distance_matrix, 2, 3, nodes[2].distance(nodes[3]))
        set_distance(distance_matrix, 3, 4, nodes[3].distance(nodes[4]))
        return nodes, distance_matrix

    if mode == 'triangle':
        nodes = [leader, Node(leader.x-1, -1), Node(leader.x+1, -1), Node(leader.x-2, -2), Node(leader.x, -2), Node(leader.x+2, -2)]
        distance_matrix = np.zeros((len(nodes), len(nodes)))
        set_distance(distance_matrix, 0, 1, nodes[0].distance(nodes[1]))
        set_distance(distance_matrix, 0, 2, nodes[0].distance(nodes[2]))
        set_distance(distance_matrix, 1, 2, nodes[1].distance(nodes[2]))
        set_distance(distance_matrix, 1, 3, nodes[1].distance(nodes[3]))
        set_distance(distance_matrix, 1, 4, nodes[1].distance(nodes[4]))
        set_distance(distance_matrix, 2, 4, nodes[2].distance(nodes[4]))
        set_distance(distance_matrix, 2, 5, nodes[2].distance(nodes[5]))
        set_distance(distance_matrix, 3, 4, nodes[3].distance(nodes[4]))
        set_distance(distance_matrix, 4, 5, nodes[4].distance(nodes[5]))
        return nodes, distance_matrix

    if mode == 'square':
        nodes = [leader, Node(leader.x, leader.y-2), Node(leader.x-2, leader.y-2), Node(leader.x-2, leader.y)]
        distance_matrix = np.zeros((len(nodes), len(nodes)))
        set_distance(distance_matrix, 0, 1, nodes[0].distance(nodes[1]))
        set_distance(distance_matrix, 0, 3, nodes[0].distance(nodes[3]))
        set_distance(distance_matrix, 1, 2, nodes[1].distance(nodes[2]))
        set_distance(distance_matrix, 2, 3, nodes[2].distance(nodes[3]))
        return nodes, distance_matrix
    
    nodes = [leader]
    distance_matrix = np.zeros((len(nodes), len(nodes)))
    return nodes, distance_matrix

def get_all_distances(nodes):
    distance_matrix = np.zeros((len(nodes), len(nodes)))
    for i in range(len(nodes)):
        for j in range(i+1, len(nodes)):
            set_distance(distance_matrix, i, j, nodes[i].distance(nodes[j]))
    return distance_matrix

def set_distance(distances, i, j, value):
    distances[i][j] = value
    distances[j][i] = value

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
    #print('#####################')
    force = np.array([0,0])
    vector_i = np.array([nodes[i].x, nodes[i].y])
    scaling_factor = 0.035

    for j in range(0,len(nodes)):
        if distances[i][j] == 0 or i == j:
            continue
        vector_j = np.array([nodes[j].x, nodes[j].y])

        #print(f'vector {i}: {vector_i}')
        #print(f'vector {j}: {vector_j}')
        #print(f'({np.linalg.norm(vector_i - vector_j)**2} - {distances[i][j]**2}) * ({vector_j - vector_i}) = {(np.linalg.norm(vector_i - vector_j)**2 - distances[i][j]**2) * (vector_j - vector_i)}')
        force = force + (np.linalg.norm(vector_i - vector_j)**2 - distances[i][j]**2) * (vector_j - vector_i)
        #print('-------------------')

    #print(f'force: {force}')
    #print(f'force scaled: {scaling_factor * force}')
    #print(f'node before: {nodes[i]}')
    nodes[i].x = nodes[i].x + scaling_factor * force[0]
    nodes[i].y = nodes[i].y + scaling_factor * force[1]
    #print(f'node after: {nodes[i]}')
    #print('-------------------')
    #print('#####################')

def repulsive_force(i, nodes, distances):
    print('#####################')
    force = np.array([0,0])
    vector_i = np.array([nodes[i].x, nodes[i].y])
    beta = 2
    scaling_factor = 0.035

    for j in range(0,len(nodes)):
        if i == j:
            continue
        vector_j = np.array([nodes[j].x, nodes[j].y])
        print(f'vector {i}: {vector_i}')
        print(f'vector {j}: {vector_j}')

        norm = np.linalg.norm(vector_i - vector_j)
        delta = distances[i][j]
        delta_o = 4
        print(norm)
        if norm < delta_o:
            print(f'(1/{beta}) * (1/({norm} - {delta}) - (1/{delta_o} - {delta})))**{beta}')
            print(f'(1/{beta}) * (1/({norm - delta}) - (1/{delta_o - delta})))**{beta}')
            print(f'(1/{beta}) * ({1/(norm - delta)} - {1/(delta_o - delta)})**{beta}')
            print((1/beta) * (1/(norm - delta) - (1/(delta_o - delta)))**beta)
            force = force + (1/beta) * (1/(norm - delta) - (1/(delta_o - delta)))**beta
            print('-------------------')

    print(f'force: {force}')
    print(f'force scaled: {scaling_factor * force}')
    print(f'node before: {nodes[i]}')
    nodes[i].x = nodes[i].x - scaling_factor * force[0]
    nodes[i].y = nodes[i].y - scaling_factor * force[1]
    print(f'node after: {nodes[i]}')
    print('-------------------')
    print('#####################')

def simulate_movement(nodes, distances):
    plt.ion()  # Turn on interactive mode for live plot

    plot_nodes(nodes, leader_index=0)
    while True:
        res = updateLeader(nodes[0])
        if not res:
            break

        for i in range(1,len(nodes)):
            attractive_force(i, nodes, distances)
            #repulsive_force(i, nodes, distances)

        plot_nodes(nodes, leader_index=0)

nodes, distance_matrix = get_formation(Node(0, 0), 'triangle')
distance_matrix = get_all_distances(nodes)

leader_trajectory = 'manual'
#if select_trajectory():
#    simulate_movement(nodes, distance_matrix)

simulate_movement(nodes, distance_matrix)