import matplotlib.pyplot as plt
import numpy as np
import math

leader_velocity = 0.5  # Leader's velocity (units of displacement per second)
simulation_time = 200  # Simulation time in seconds
mode = 'circular'  # 'linear' or 'circular'
k_att = 1.0  # Weight for the attractive force
k_rep = 5.0  # Weight for the repulsive force
k_form = 0.2  # Weight for the formation force
k_vel = 1.0  # Weight for the velocity force
desired_distance = 2.0  # Desired distance between nodes

class Node:
    def __init__(self, position):
        self.position = np.array(position, dtype=np.float64)
        self.velocity = np.zeros(2, dtype=np.float64)

    def __str__(self):
        return f"x: {self.position[0]}, y: {self.position[1]}"

    def distance(self, node):
        return np.linalg.norm(self.position - node.position)

def updateLeader(leader):
    if mode == 'linear':
        leader.position[0] += leader_velocity

    elif mode == 'circular':
        radius = 5
        angular_velocity = leader_velocity / radius
        theta = np.arctan2(leader.position[1], leader.position[0])
        theta += angular_velocity
        leader.position[0] = radius * np.cos(theta)
        leader.position[1] = radius * np.sin(theta)

    else:
        print('Invalid mode')

def plot_nodes(nodes, leader_index):
    plt.clf()
    for i, node in enumerate(nodes):
        marker = 'o' if i == leader_index else 'x'
        plt.plot(node.position[0], node.position[1], marker)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Movement of nodes')
    #plt.xlim(nodes[leader_index].position[0] - 10, nodes[leader_index].position[0] + 10)  # Set the x-axis limits
    #plt.ylim(nodes[leader_index].position[1] - 10, nodes[leader_index].position[1] + 10)  # Set the y-axis limits
    plt.xlim(-10, 10)
    plt.ylim(-10, 10)
    plt.grid(False)
    plt.pause(0.1)
    plt.draw()

def calculate_velocity_force(node, leader):
    desired_velocity = leader_velocity * (leader.position - node.position) / np.linalg.norm(leader.position - node.position)
    return k_vel * (desired_velocity - node.velocity)

def attractive_force(node, leader, k_att):
    direction = leader.position - node.position
    return k_att * direction

def repulsive_force(node, other_node, k_rep):
    distance = node.distance(other_node)
    if distance == 0:
        return np.zeros(2)
    direction = node.position - other_node.position
    return k_rep / distance**2 * direction

def formation_force(node, other_node, desired_distance, k_form):
    distance = node.distance(other_node)
    if distance == 0:
        return np.zeros(2)
    direction = other_node.position - node.position
    desired_direction = direction / distance * desired_distance
    return k_form * (desired_direction - direction)

def calculate_total_force(node, leader, other_nodes, k_att, k_rep, k_form):
    total_force = np.zeros(2)

    # Attractive force towards the leader
    total_force += attractive_force(node, leader, k_att)

    # Repulsive forces from other nodes
    for other_node in other_nodes:
        if other_node != node:
            total_force += repulsive_force(node, other_node, k_rep)

    # Formation forces with other nodes
    for other_node in other_nodes:
        if other_node != node:
            total_force += formation_force(node, other_node, desired_distance, k_form)

    # Velocity force to follow the leader
    total_force += calculate_velocity_force(node, leader)

    return total_force

def simulate_movement(nodes):
    plt.ion()

    for _ in range(simulation_time):
        updateLeader(nodes[0])

        for i in range(1, len(nodes)):
            total_force = calculate_total_force(nodes[i], nodes[0], nodes[1:], k_att, k_rep, k_form)
            nodes[i].velocity += total_force
            nodes[i].position += nodes[i].velocity

        plot_nodes(nodes, 0)

    plt.ioff()
    plt.show()

nodes = [
    Node([5, 0]),
    Node([4, -1]),
    Node([6, -1]),
    Node([3, -2]),
    Node([5, -2]),
    Node([7, -2])
]

simulate_movement(nodes)
