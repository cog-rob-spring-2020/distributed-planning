""" RRT* implementation in python. Based heavily on this source code by AtsushiSakai:
    https://github.com/AtsushiSakai/PythonRobotics

    This is intended for use with a 2D map.
"""
import numpy as np
import random
import copy
import rospy
import tf.transformations as transformations


class Node(object):
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

    def __ne__(self, other):
        return not self.__eq__(other)


class NodeStamped(Node):
    def __init__(self, node):
        super(NodeStamped, self).__init__(node.x, node.y)
        self.path_x = node.path_x
        self.path_y = node.path_y
        self.parent = node.parent
        self.cost = node.cost
        self.stamp = None

    def __str__(self):
        return "node xy: " + str(self.x) + ", " + str(self.y) + \
               " | ts: " + str(self.stamp)

    def __eq__(self, other):
        if isinstance(other, self.__class__) or isinstance(other, Node):
            return (self.x == other.x) and (self.y == other.y)
        return False

    def __ne__(self, other):
        return not self.__eq__(other)


class Path:
    def __init__(self, nodes=[], start_node=None, goal_node=None, penalty=0):
        """ A representation of a path through a 2D map parametrized by time.

            This object should not be modified after initialization, except to add
            emergency stops. For subpaths, create a new path object. This is the
            reason all fields are immutable when possible.
            If all arguments are default, the ctor will give the path infinite
            cost. An empty path represents an infinite path, NOT a zero-cost path.

            Args:
                nodes: a list of NodeStamped objects representing the path.
                start_node: a NodeStamped representing the x-y position of the start.
                goal_node: a NodeStamped representing the x-y position of the goal.
                penalty: a float to be added to the cost of incomplete paths.
        """
        self.nodes = tuple(nodes)
        self.start_node = start_node
        self.goal_node = goal_node

        self.is_complete = False  # True if this Path goes from start to end
        self.cost = np.inf
        self.dist_to_goal = np.inf

        if self.nodes:
            if self.nodes[-1].x == self.goal_node.x and \
                    self.nodes[-1].y == self.goal_node.y:
                self.is_complete = True
                self.dist_to_goal = 0.0

            self.cost = self.nodes[-1].cost
            if not self.is_complete:
                self.cost += penalty  # Incomplete costs more than complete
                self.dist_to_goal = Path.dist_between_nodes(self.nodes[-1],
                                                            NodeStamped(self.goal_node))

        # There is no frozendict! Do not modify this!
        self.ts_dict = {node.stamp: node for node in self.nodes}

    def __str__(self):
        writ = "Path start: " + str(self.start_node) + \
               "\nPath goal: " + str(self.goal_node) + \
               "\nPath completion status: " + str(self.is_complete) + \
               "\nPath cost: " + str(self.cost) + \
               "\nPath dist_to_goal: " + str(self.dist_to_goal) + \
               "\nPath nodes length: " + str(len(self.nodes)) + \
               "\nPath nodes: "
        for node in self.nodes:
            writ += '\n\t' + str(node)

        return writ

    @staticmethod
    def dist_between_nodes(from_node, to_node):
        """ Compute the Euclidean distance between two nodes. """
        dx = from_node.x - to_node.x
        dy = from_node.y - to_node.y

        return np.sqrt(np.square(dx) + np.square(dy))

    def get_cost_between(self, from_id, to_id):
        """ Return the cost between two node IDs.

            Args:
                from_id: an integer representing the id of the node in
                    self.nodes from which the calculation is done.
                to_id: an integer representing the id of the node in
                    self.nodes that is the destination.

            Returns: a float representing the cost between the nodes.
        """
        if to_id > 0:
            assert(to_id > from_id)

        from_cost = self.nodes[from_id].cost
        to_cost = self.nodes[to_id].cost

        return to_cost - from_cost


class RRTstar:
    def __init__(self, start, goal, env, goal_dist=0.5, goal_sample_rate=0.7,
                 step_size=0.1, near_radius=5.0, max_iter=1000):
        # Planner states
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])

        self.map_data = None
        self.map_metadata = None
        self.map_bounds = None
        self.setup_map(env)

        self.curr_iter = 0
        self.node_list = [self.start]
        self.found_complete_path = False
        self.curr_time = 0

        # Planner parameters
        self.goal_dist = goal_dist
        self.goal_sample_rate = goal_sample_rate
        self.step_size = step_size
        self.near_radius = near_radius
        self.max_iter = max_iter

        # Persistent states
        self.best_path = Path()
        self.curr_pos = self.start
        self.nodes_traveled = [self.start]

    def spin(self, return_first_path=False):
        """ Expand the tree and plan through the environment. """
        # print("RRTstar >> starting spin...")
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
            if not self.check_collision_line(near_node, new_node):
                new_node.parent = near_node
                new_node.cost = RRTstar.compute_new_cost(near_node, new_node)

                near_ids = self.get_near_node_ids(new_node)
                new_node = self.choose_parent(new_node, near_ids)

                self.node_list.append(new_node)
                self.rewire(new_node, near_ids)

            if return_first_path and new_node:
                return self.get_path()

        return None

    def steer(self, from_node, to_node):
        """ Drive the growth of the tree towards the goal point. """
        new_node = Node(from_node.x, from_node.y)
        d, theta = RRTstar.calc_dist_and_angle(new_node, to_node)

        step = self.step_size
        if step > d:
            step = d

        new_node.x += step * np.cos(theta)
        new_node.y += step * np.sin(theta)

        if new_node != from_node:
            if not self.check_collision_line(new_node, from_node):
                new_node.parent = from_node
                return new_node

        return None

    def choose_parent(self, new_node, near_ids):
        """ Choose parent for the new node based on nearest nodes. """
        if not near_ids:
            new_node

        costs = []
        for id in near_ids:
            near_node = self.node_list[id]
            # t_node = self.steer(near_node, new_node)

            # if t_node and not self.check_collision_line(new_node, t_node):
            if near_node and \
                    not self.check_collision_line(near_node, new_node) and \
                    near_node != new_node:
                costs.append(RRTstar.compute_new_cost(near_node, new_node))
            else:
                costs.append(np.inf)  # Collision nodes have inf cost.

        if len(costs) > 0:
            min_cost = min(costs)
            if min_cost != np.inf:
                min_id = near_ids[costs.index(min_cost)]
                # new_node = self.steer(self.node_list[min_id], new_node)
                new_node.parent = self.node_list[min_id]
                new_node.cost = min_cost

        return new_node

    @staticmethod
    def compute_new_cost(from_node, to_node):
        """ Compute cost between two nodes. """
        d, _ = RRTstar.calc_dist_and_angle(from_node, to_node)
        return from_node.cost + d

    def search_best_goal_node(self):
        """ Identify the best goal node given the current path. """
        dist_to_goal_ls = [self.dist_to_goal(node) for node in self.node_list]
        goal_ids = [dist_to_goal_ls.index(i)
                    for i in dist_to_goal_ls if i <= 1.1 * self.goal_dist]

        safe_goal_ids = []
        for id in goal_ids:
            t_node = self.steer(self.node_list[id], self.goal)

            if t_node and not self.check_collision_line(self.node_list[id], t_node):
                safe_goal_ids.append(id)

        if len(safe_goal_ids) != 0:
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
                edge_node.cost = RRTstar.compute_new_cost(new_node, near_node)

                if near_node.cost > edge_node.cost and \
                        not self.check_collision_line(new_node, edge_node):
                    self.node_list[id] = edge_node
                    self.propagate_cost(new_node)

    def propagate_cost(self, parent_node):
        """ Propagate cost down the tree to all leaves of the parent. """
        for node in self.node_list:
            if node.parent == parent_node:
                node.cost = RRTstar.compute_new_cost(parent_node, node)
                self.propagate_cost(node)

    def get_path(self):
        """ Attempt to find a path to a goal point. Returns the best path.

            If the best path found is incomplete, then it still returns this
            path.

            Returns: a Path object representing the current best path to the
                goal node.
        """
        last_id = self.search_best_goal_node()
        new_path = self.generate_final_path(last_id, self.curr_time)

        # Get cost of new path starting from current position
        cur_node_id = new_path.nodes.index(self.curr_pos)
        new_path_cost_adj = new_path.get_cost_between(cur_node_id, -1)

        # Get cost of best path starting from current position
        cur_node_id = None
        best_path_cost_adj = self.best_path.cost
        if self.best_path.nodes:
            cur_node_id = self.best_path.nodes.index(self.best_path.start_node)
            best_path_cost_adj = self.best_path.get_cost_between(
                cur_node_id, -1)

        # Choose the new path if we have complete paths and it's cheaper
        if self.found_complete_path:
            if (new_path_cost_adj < best_path_cost_adj) and \
                    new_path.is_complete:
                self.best_path = new_path
        else:
            # Choose the new path if it's complete and the best isn't
            if (new_path.is_complete) and (not self.best_path.is_complete):
                self.found_complete_path = True
                self.best_path = new_path
            # Choose the new path if its closer to the goal point
            elif new_path.dist_to_goal < self.best_path.dist_to_goal:
                self.best_path = new_path

        return self.best_path

    def generate_final_path(self, id=None, curr_time=0):
        """ Prune the final path once the goal has been found.

            Args:
                id: an integer representing the index of the node closest
                    to the goal point. If None, the resultant path is
                    incomplete
                start_time: a timestamp representing the starting time for
                    the path, passed to the time allocation routine
        """
        node = None
        path_nodes = []

        if id:
            # Complete path
            goal_node = NodeStamped(self.goal)
            goal_node.cost = RRTstar.compute_new_cost(
                self.node_list[id], self.goal)
            path_nodes = [goal_node]
        else:
            # Incomplete path
            dlist = [self.dist_to_goal(node) for node in self.node_list]
            id = dlist.index(min(dlist))

        node = self.node_list[id]

        while node.parent is not None:
            node_stamped = NodeStamped(node)
            path_nodes.append(node_stamped)
            node = node.parent

        path_nodes.append(NodeStamped(node))
        path_nodes.reverse()

        # Weird edge case where nodes are empty when close to goal
        if not path_nodes:
            return self.best_path

        # Allocate time to this part of the path
        path_nodes = self.allocate_time(path_nodes, curr_time)

        # Add the path already traveled
        if self.nodes_traveled:
            path_parent = self.nodes_traveled[-1]
            path_nodes[0].parent = path_parent
            for node in path_nodes:
                node.cost += path_parent.cost

            nodes_traveled_stamped = [
                NodeStamped(node) for node in self.nodes_traveled]
            for node in nodes_traveled_stamped:
                node.stamp = 0  # Traveled nodes all have zero ts

            path_nodes = nodes_traveled_stamped + path_nodes

        path = Path(path_nodes,
                    NodeStamped(self.start),
                    NodeStamped(self.goal),
                    2.0 * max(self.map_bounds[0][1] ,self.map_bounds[1][1]))
        return path

    def allocate_time(self, nodes, start_time):
        """ Attribute timestamps to each node in the nodes list.

            Args:
                nodes: a list of NodeStamped objects
                start_time: the start time for the first node in the list
        """

        def naive_alloc():
            """ Simply increment each stamp by one integer. """
            count = start_time
            for node in nodes:
                node.stamp = count
                count += 1

        naive_alloc()
        return nodes

    def dist_to_goal(self, node):
        """ Compute the Euclidean distance to the goal node. """
        dx = node.x - self.goal.x
        dy = node.y - self.goal.y

        return np.sqrt(np.square(dx) + np.square(dy))

    def get_rand_node(self):
        """ Return a random Node object within the prescribed confines. """
        if random.randint(0, 100) > self.goal_sample_rate:
            node = Node(random.uniform(self.map_bounds[0][0], self.map_bounds[0][1]),
                        random.uniform(self.map_bounds[1][0], self.map_bounds[1][1]))

        else:
            node = Node(self.goal.x, self.goal.y)

        return node

    def get_nearest_node_id(self, new_node):
        """ Return nearest node in the nodelist to the input node. """
        dlist = [(node.x - new_node.x)**2 + (node.y - new_node.y)**2
                 for node in self.node_list]

        minid = dlist.index(min(dlist))
        return minid

    def get_near_node_ids(self, new_node):
        """ Return all near nodes in the nodelist to the input node. """
        n = len(self.node_list) + 1
        r = self.near_radius * np.sqrt((np.log(n) / n))

        dist_list = [(node.x - new_node.x)**2 + (node.y - new_node.y)**2
                     for node in self.node_list]
        near_ids = [dist_list.index(i) for i in dist_list if i <= r**2]

        return near_ids

    @staticmethod
    def calc_dist_and_angle(from_node, to_node):
        """ Compute the distance and angle between two nodes. """
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = np.sqrt(np.square(dx) + np.square(dy))
        theta = np.arctan2(dy, dx)

        return d, theta

    def update_pos(self, pos, curr_time, wipe_tree=True):
        """ Store self agent current position for new tree generation.

            Note that this will wipe the existing tree, including the
            node_list, if `wipe_tree` is set to True. This does not
            reset `best_path` or `nodes_traveled`.
            Member updates only happen if the incoming position is not the
            same as the current recorded position (i.e. we haven't moved).
            Tree wipe doesn't happen if we haven't moved.

            Args:
                pos: a 2-tuple representing the x-y coordinates of the agent
                    running the RRT algorithm.
                curr_time: a time representing the current time of the sim.
                wipe_tree: a boolean; if True, the RRT tree will be reset.
        """
        new_pos = Node(pos[0], pos[1])
        self.curr_time = curr_time
        if new_pos != self.curr_pos:
            new_pos.cost = RRTstar.compute_new_cost(self.nodes_traveled[-1],
                                                    new_pos)
            self.nodes_traveled.append(new_pos)
            self.curr_pos = new_pos

            if wipe_tree:
                self.start = new_pos
                self.curr_iter = 0
                self.node_list = [self.start]

    def setup_map(self, env):
        """
        """
        self.map_data = self.convert_map(env)
        self.map_metadata = env.info
        res = self.map_metadata.resolution

        maxx = (self.map_metadata.origin.position.x) + (self.map_metadata.width * res)
        maxy = (self.map_metadata.origin.position.y) + (self.map_metadata.height * res)
        minx = maxx - (self.map_metadata.width * res)
        miny = maxy - (self.map_metadata.height * res)
        self.map_bounds = ((minx, maxx), (miny, maxy))

    def check_collision_point(self, x, y):
        """
        """
        map_pos = self.convert_point_to_map((x, y))
        return self.map_data[map_pos[0]][map_pos[1]] > 0

    def check_collision_line(self, node1, node2):
        """
        """
        map_pos1 = self.convert_point_to_map((node1.x, node1.y))
        map_pos2 = self.convert_point_to_map((node2.x, node2.y))

        minx = min(map_pos1[0], map_pos2[0])
        miny = min(map_pos1[1], map_pos2[1])
        maxx = max(map_pos1[0], map_pos2[0])
        maxy = max(map_pos1[1], map_pos2[1])

        if (minx == maxx and miny == maxy):
            return self.check_collision_point(node1.x, node1.y)
        
        if (maxx - minx) > (maxy - miny):
            for x in range(minx, maxx+1):
                y = (map_pos2[1] - map_pos1[1]) * (x - minx) / (maxx - minx) + map_pos1[1]
                if self.map_data[x][y] > 0:
                    return True
        else:
            for y in range(miny, maxy+1):
                x = (map_pos2[0] - map_pos1[0]) * (y - miny) / (maxy - miny) + map_pos1[0]
                if self.map_data[x][y] > 0:
                    return True

        return False
    
    def convert_point_to_map(self, pos):
        """
        """
        x = pos[0]
        y = pos[1]

        x -= self.map_metadata.origin.position.x
        y -= self.map_metadata.origin.position.y
        roll, pitch, yaw = transformations.euler_from_quaternion([
            self.map_metadata.origin.orientation.x,
            self.map_metadata.origin.orientation.y,
            self.map_metadata.origin.orientation.z,
            self.map_metadata.origin.orientation.w]
        )

        x = x * np.cos(yaw) + y * np.sin(yaw)
        y = -x * np.sin(yaw) + y * np.cos(yaw)

        return (int(x / self.map_metadata.resolution),
                int(y / self.map_metadata.resolution))

    def convert_map(self, map_data):
        """
        """
        height = map_data.info.height
        width = map_data.info.width

        new_map = np.transpose(np.reshape(map_data.data, (height, width)))

        return new_map
