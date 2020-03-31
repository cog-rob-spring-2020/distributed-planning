import math

class Path():
    def __init__(self, nodes = []):
        self.nodes = nodes
        self.emergency_stops = set()

    def add_node(self, node):
        self.nodes.append(node)

    def mark_safe(self, node):
        self.emergency_stops.add(node)

    def cost(self):
        dist = 0
        for i in range(len(self.nodes)-1):
            p1 = self.nodes[i]
            p2 = self.nodes[i+1]
            dist += math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)
        return dist
