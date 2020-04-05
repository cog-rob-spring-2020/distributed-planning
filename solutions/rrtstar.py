""" RRT* implementation in python. Based heavily on this source code by AtsushiSakai:
    https://github.com/AtsushiSakai/PythonRobotics

    This is intended for use with a 2D map.
"""

import numpy as np
import random
import copy
from shapely.geometry import Point, Polygon, LineString

from utils.environment import Environment, plot_environment, plot_line, plot_poly

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.path_x = []
        self.path_y = []
        self.parent = None
        self.cost = 0.0

    def __str__(self):
        return "node xy: " + str(self.x) + ", " + str(self.y)

    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return (self.x == other.x) and (self.y == other.y)
        return False


class NodeStamped(Node):
    def __init__(self, x, y):
        super.__init__(x, y)
        self.stamp = None
    
    def __init__(self, node):
        self.x = copy.deepcopy(node.x)
        self.y = copy.deepcopy(node.y)
        self.path_x = copy.deepcopy(node.path_x)
        self.path_x = copy.deepcopy(node.path_y)
        self.path_y = copy.deepcopy(node.path_y)
        self.parent = copy.deepcopy(node.parent)
        self.cost = copy.deepcopy(node.cost)
        
        if isinstance(node, self.__class__):
            self.stamp = copy.deepcopy(node.stamp)
        else:
            self.stamp = None
    
    def __str__(self):
        return "node xy: " + str(self.x) + ", " + str(self.y) + \
               " | ts: " + str(self.stamp)


class Path():
    def __init__(self, nodes):
        self.nodes = nodes
        self.emergency_stops = set()
        self.ts_dict = {node.stamp : node for node in self.nodes}
        self.cost = self.nodes[-1].cost

    def mark_safe(self, node):
        self.emergency_stops.add(node)


class RRTstar:
    def __init__(self, start, goal, env, goal_dist=0.5, goal_sample_rate=0.7,
                 path_resolution=0.1, connect_circle_dist=20.0, max_iter=1000):
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.env = env
        self.goal_dist = goal_dist
        self.goal_sample_rate = goal_sample_rate
        self.path_resolution = path_resolution
        self.connect_circle_dist = connect_circle_dist
        self.max_iter = max_iter
        self.node_list = []
        self.curr_iter = 0

    def spin(self, return_first_path=False):
        """ Expand the tree and plan through the environment. """
        print("RRTstar >> starting spin...")

        self.node_list = [self.start]

        for iter in range(self.max_iter):
            path = self.spin_once(return_first_path)
            if path is not None:
                return path
            
        return self.get_path()

    def spin_once(self, return_first_path):
        """ Perform one iteration of tree-expansion and path-planning. """
        self.curr_iter += 1
        rand_node = self.get_rand_node()

        nearest_id = self.get_nearest_node_id(rand_node)
        near_node = self.node_list[nearest_id]
        new_node = self.steer(near_node, rand_node)

        if new_node is not None:
            if self.env.collision_free(new_node.x, new_node.y):
                near_ids = self.get_near_node_ids(new_node)
                new_node = self.choose_parent(new_node, near_ids)
                
                if new_node:
                    self.node_list.append(new_node)
                    self.rewire(new_node, near_ids)

            if return_first_path and new_node:
                return self.get_path()

        return None

    def steer(self, from_node, to_node):
        """ Drive the growth of the tree towards the goal point. """
        new_node = Node(from_node.x, from_node.y)
        d, theta = self.calc_dist_and_angle(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        extend_length = self.goal_dist
        if extend_length > d:
            extend_length = d

        n_expand = np.floor(extend_length / self.path_resolution)
        for _ in range(int(n_expand)):
            new_node.x += self.path_resolution * np.cos(theta)
            new_node.y += self.path_resolution * np.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_dist_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)

        new_node.parent = from_node

        if new_node != from_node:
            return new_node
        
        return None

    def choose_parent(self, new_node, near_ids):
        """ Choose parent for the new node based on nearest nodes. """
        if not near_ids:
            return None

        costs = []
        for id in near_ids:
            near_node = self.node_list[id]
            t_node = self.steer(near_node, new_node)

            if t_node and self.env.collision_free(t_node.x, t_node.y):
                costs.append(self.compute_new_cost(near_node, new_node))
            else:
                costs.append(np.inf)  # Collision nodes have inf cost.
            
        min_cost = min(costs)
        if min_cost == np.inf:
            return None
        
        min_id = near_ids[costs.index(min_cost)]
        new_node = self.steer(self.node_list[min_id], new_node)
        new_node.parent = self.node_list[min_id]
        new_node.cost = min_cost

        return new_node

    def compute_new_cost(self, from_node, to_node):
        """ Compute cost between two nodes. """
        d, _ = self.calc_dist_and_angle(from_node, to_node)
        return from_node.cost + d

    def search_best_goal_node(self):
        """ Identify the best goal node given the current path. """
        dist_to_goal_ls = [self.dist_to_goal(node) for node in self.node_list]
        goal_ids = [dist_to_goal_ls.index(i) \
                    for i in dist_to_goal_ls if i <= self.goal_dist]

        safe_goal_ids = []
        for id in goal_ids:
            t_node = self.steer(self.node_list[id], self.goal)

            if t_node and self.env.collision_free(t_node.x, t_node.y):
                safe_goal_ids.append(id)
        
        if len(safe_goal_ids) == 0:
            return None
        
        min_cost = min([self.node_list[i].cost for i in safe_goal_ids])
        for i in safe_goal_ids:
            if self.node_list[i].cost == min_cost:
                return i
        
        return None

    def rewire(self, new_node, near_ids):
        """ Rewire cost of nearby nodes based on cost of the new node. """
        for id in near_ids:
            near_node = self.node_list[id]
            edge_node = self.steer(new_node, near_node)

            if edge_node:
                edge_node.cost = self.compute_new_cost(new_node, near_node)

                if near_node.cost > edge_node.cost and \
                        self.env.collision_free(edge_node.x, edge_node.y):
                    self.node_list[id] = edge_node
                    self.propagate_cost(new_node)

    def propagate_cost(self, parent_node):
        """ Propagate cost down the tree to all leaves of the parent. """
        for node in self.node_list:
            if node.parent == parent_node:
                node.cost = self.compute_new_cost(parent_node, node)
                self.propagate_cost(node)

    def get_path(self):
        """ Attempt to find a path to a goal point. Return None if can't. """
        last_id = self.search_best_goal_node()
        if last_id:
            print("RRTstar >> found a path! on iter: ", self.curr_iter)
            return self.generate_final_path(last_id)
        
        return None

    def generate_final_path(self, id):
        """ Prune the final path once the goal has been found. """
        self.goal.cost = self.compute_new_cost(self.node_list[id], self.goal)
        node = self.node_list[id]
        path_nodes = [NodeStamped(self.goal)]

        while node.parent is not None:
            node_stamped = NodeStamped(node)
            path_nodes.append(node_stamped)
            node = node.parent
        
        path_nodes.append(NodeStamped(node))
        path_nodes.reverse()
        path_nodes = self.allocate_time(path_nodes)

        path = Path(path_nodes)
        return path

    def allocate_time(self, nodes):
        """ Attribute timestamps to each node in the nodes list. """
        
        def naive_alloc(nodes):
            """ Simply increment each stamp by one integer. """
            count = 0
            for node in nodes:
                node.stamp = count
                count += 1
            
            return nodes
        
        return naive_alloc(nodes)

    def dist_to_goal(self, node):
        """ Compute the Euclidean distance to the goal node. """
        dx = node.x - self.goal.x
        dy = node.y - self.goal.y

        return np.sqrt(np.square(dx) + np.square(dy))

    def get_rand_node(self):
        """ Return a random Node object within the prescribed confines. """
        if random.randint(0, 100) > self.goal_sample_rate:
            node = Node(random.uniform(self.env.bounds[0], self.env.bounds[2]),
                        random.uniform(self.env.bounds[1], self.env.bounds[3]))

        else:
            node = Node(self.goal.x, self.goal.y)
        
        return node

    def get_nearest_node_id(self, new_node):
        """ Return nearest node in the nodelist to the input node. """
        dlist = [(node.x - new_node.x)**2 + (node.y - new_node.y)**2 \
                for node in self.node_list]

        minid = dlist.index(min(dlist))
        return minid

    def get_near_node_ids(self, new_node):
        """ Return all near nodes in the nodelist to the input node. """
        n = len(self.node_list) + 1
        r = self.connect_circle_dist * np.sqrt((np.log(n) / n))
        r = min(r, self.goal_dist)

        dist_list = [(node.x - new_node.x)**2 + (node.y - new_node.y)**2 \
                    for node in self.node_list]
        near_ids = [dist_list.index(i) for i in dist_list if i <= r**2]

        return near_ids

    def calc_dist_and_angle(self, from_node, to_node):
        """ Compute the distance and angle between two nodes. """
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = np.sqrt(np.square(dx) + np.square(dy))
        theta = np.arctan2(dy, dx)

        return d, theta

    def plot_tree(self):
        """ Plot the entire RRT tree, final path, and start/goal nodes. """
        ax = plot_environment(self.env)

        plot_poly(ax, Point(
            (self.start.x, self.start.y)).buffer(0.3, resolution=3), "green")
        plot_poly(ax, Point(
            (self.goal.x, self.goal.y)).buffer(0.3, resolution=3), "red")

        for node in self.node_list:
            if node.parent is not None:
                plot_line(ax, LineString([[node.x, node.y],
                                          [node.parent.x, node.parent.y]]))

        path = self.get_path()
        if path is not None:
            path_xy = [[node.x, node.y] for node in path.nodes]
            line = LineString(path_xy)
            extended_line = line.buffer(0.3, resolution=3)
            plot_poly(ax, extended_line, "yellow", alpha=0.5)

            ax.set_title(f"Number of nodes in tree: {len(self.node_list)} \n \
                        Number of nodes in solution path: {len(path.nodes)} \
                        \n Path length: {path.cost}")

        else:
            ax.set_title(f"Number of nodes in tree: {len(self.node_list)}")
            print("No path was found.")
