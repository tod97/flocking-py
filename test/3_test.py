import matplotlib.pyplot as plt
import numpy as np
import math

leader_velocity = 0.5  # Leader's velocity (units of displacement per second)
simulation_time = 200  # Simulation time in seconds
mode = 'circular'  # 'linear' or 'circular'

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __str__(self):
        return f"x: {self.x}, y: {self.y}"

    def distance(self, node):
        return np.sqrt((self.x - node.x)**2 + (self.y - node.y)**2)
    
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

def plot_nodes(nodes, leader_index):
    plt.clf()
    for i, node in enumerate(nodes):
        marker = 'o' if i == leader_index else 'x'
        plt.plot(node.x, node.y, marker)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Movement of Nodes')
    #plt.xlim(nodes[0].x - 10, nodes[0].x + 10)  # Set the x-axis limits
    #plt.ylim(nodes[0].y - 10, nodes[0].y + 10)  # Set the y-axis limits
    plt.xlim(-10, 10)
    plt.ylim(-10, 10)
    plt.grid(False)  # Disable grid lines
    plt.pause(0.1)  # Pause for 0.1 seconds to show the plot
    plt.draw()

def get_distances(nodes):
    n = len(nodes)
    distance_matrix = np.zeros((n, n))

    for i in range(n):
        for j in range(n):
            #distance_matrix[i][j] = abs(nodes[i].x - nodes[j].x)
            distance_matrix[i][j] = nodes[i].distance(nodes[j])

    return distance_matrix

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

    for _ in range(simulation_time):
        updateLeader(nodes[0])

        for i in range(1,len(nodes)):
            attractive_force(nodes[i], nodes[0], distances[i][0])

            for j in range(1,len(nodes)):
                if i != j:
                    repulsive_force(nodes[i], nodes[j], distances[i][j])

        plot_nodes(nodes, leader_index=0)

    plt.ioff()
    plt.show()

nodes = [
    Node(-5, 0),
    Node(4, -1),
    Node(6, -1),
    Node(3, -2),
    Node(5, -2),
    Node(7, -2),
    Node(5, -3)
]

distances = get_distances(nodes)

simulate_movement(nodes, distances)
