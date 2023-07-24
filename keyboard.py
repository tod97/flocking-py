import matplotlib.pyplot as plt
import numpy as np
import math
from readchar import readkey, key

leader_velocity = 0.5  # Leader's velocity (units of displacement per second)
mode = 'linear'  # 'linear', 'circular', 'manual'

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __str__(self):
        return f"x: {self.x}, y: {self.y}"

    def distance(self, node):
        return np.sqrt((self.x - node.x)**2 + (self.y - node.y)**2)

def select_mode():
    global mode
    while True:
        print("Menu:")
        print("1. Linear trajectory")
        print("2. Circular trajectory")
        print("3. Manual")
        print("0. Exit")
        choice = input("Enter the number corresponding to your choice: ")

        if choice == "1":
            mode = 'linear'
            break
        elif choice == "2":
            mode = 'circular'
            break
        elif choice == "3":
            mode = 'manual'
            break
        elif choice == "0":
            break
        else:
            print("Invalid choice. Please select a valid option.")

def get_formation(leader, mode):
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
            Node(leader.x-1, leader.y+1),
            Node(leader.x+1, leader.y+1),
            Node(leader.x-1, leader.y-1),
            Node(leader.x+1, leader.y-1),
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
    if mode == 'linear':
        leader.x += leader_velocity

    elif mode == 'circular':
        radius = 5
        current_angle = math.atan2(leader.y, leader.x+radius)
        new_angle = current_angle + 0.1
        leader.x = (radius * np.cos(new_angle)) - radius
        leader.y = (radius * np.sin(new_angle))

    else:
        print('Invalid mode')

def attractive_force(node, leader, distance):
    angolo = math.atan2(node.y - leader.y, node.x - leader.x)
    node.x = leader.x + distance * math.cos(angolo)
    node.y = leader.y + distance * math.sin(angolo)

def repulsive_force(node, other_node, distance):
    # mantain the distance between nodes
    if node.distance(other_node) < distance:
        angolo = math.atan2(node.y - other_node.y, node.x - other_node.x)
        node.x = other_node.x + distance * math.cos(angolo)
        node.y = other_node.y + distance * math.sin(angolo)

def simulate_movement(nodes, distances):
    plt.ion()  # Turn on interactive mode for live plot

    plot_nodes(nodes, leader_index=0)
    while True:
        if mode == 'manual':
            key_pressed = readkey()
            if key_pressed == 'q':
                break
            elif key_pressed == key.UP:
                nodes[0].y += leader_velocity
            elif key_pressed == key.DOWN:
                nodes[0].y -= leader_velocity
            elif key_pressed == key.RIGHT:
                nodes[0].x += leader_velocity
            elif key_pressed == key.LEFT:
                nodes[0].x -= leader_velocity
        else:
            updateLeader(nodes[0])

        for i in range(1,len(nodes)):
            attractive_force(nodes[i], nodes[0], distances[i][0])

            for j in range(1,len(nodes)):
                  if i != j:
                     repulsive_force(nodes[i], nodes[j], distances[i][j])

        plot_nodes(nodes, leader_index=0)

nodes = get_formation(Node(0, 0), 'triangle')

select_mode()

distances = get_distances(nodes)
simulate_movement(nodes, distances)
