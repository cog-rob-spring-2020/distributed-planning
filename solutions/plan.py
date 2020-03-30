class Path():
    def __init__(nodes = []):
        self.nodes = nodes
        self.emergency_stops = set()

    def add_node(node):
        self.nodes.append(node)

    def mark_safe(node):
        self.emergency_stops.add(node)
