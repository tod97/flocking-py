import numpy as np

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.radius = 5
        self.center = Node(x - self.radius, y)

    def __str__(self):
        return f"x: {self.x}, y: {self.y}"

    def distance(self, node):
        return np.sqrt((self.x - node.x)**2 + (self.y - node.y)**2)

    def move_along_circle(self, angle):
        # Calcola le nuove coordinate (x, y) del nodo sulla circonferenza
        self.x = self.center.x + self.radius * np.cos(angle)
        self.y = self.center.y + self.radius * np.sin(angle)

# Esempio di utilizzo

# Crea un nodo con coordinate (10, 10)
node = Node(10, 10)
print("Posizione iniziale:", node)

# Muovi il nodo lungo la circonferenza con incrementi di 45 gradi
""" for angle_deg in range(0, 360):
   angle_rad = np.radians(angle_deg)
   #node.move_along_circle(angle_rad)
   print(f"Posizione dopo {angle_deg} gradi:", node)
 """