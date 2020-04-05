import time
from rrtstar import Node, NodeStamped, RRTstar
from solutions.antenna import Antenna, TOPIC_BIDS, TOPIC_WAYPOINTS


class Agent():
    def __init__(self, mode, x_start, y_start, x_goal, y_goal, environment, bounds, goal_dist, tolerance):
        self.antenna = Antenna()
        self.antenna.find_peers()

        self.token_holder = False

        self.check_estops = (mode == 'cooperative')

        # keeps track of other agents' current bids for PPI
        # (potential path improvement) at any given time:
        self.bids = {id:None for id in self.antenna.peers}

        # keeps track of other agents' plans so that we can
        # add the other agents as obstacles when replanning
        self.plans = {id:None for id in self.antenna.peers}

        # initial state and environment
        self.tolerance = tolerance
        self.goal_dist = goal_dist
        self.bounds = bounds
        self.environment = environment

        self.pose = (x_start, y_start)
        self.goal = (x_goal, y_goal)

        self.rrt = RRTstar(self.pose, self.goal, self.environment, self.goal_dist)

        # a list of NodeStamped waypoints, starting with current pose
        self.plan = self.rrt.get_path()

        # register as listener for different kinds of messages
        self.antenna.on_message("bids", self.received_bid)
        self.antenna.on_message("waypoints", self.received_waypoints)
        self.antenna.on_message("estop", self.received_estop)

    """
    Methods for listening and broadcasting to various topics.

    The listener methods comprise the interaction component of DMA-RRT
    and Cooperative DMA-RRT from Desaraju/How 2012, as described in algorithms 5 and 8 respectively.
    """
    def broadcast_bid(self, bid):
        self.antenna.broadcast(TOPIC_BIDS, bid)

    def received_bid(self, msg):
        bidder_id, bid = msg
        self.bids[bidder_id] = bid

    def broadcast_waypoints(self, winner_id):
        self.antenna.broadcast(TOPIC_WAYPOINTS, winner_id)

    def received_waypoints(self, msg):
        other_id, other_plan, winner_id = msg
        self.plans[other_id] = other_plan
        if winner_id == self.id:
            self.token_holder = True

    def broadcast_estop(self, stop_node):
        self.antenna.broadcast(TOPIC_ESTOPS, stop_node)

    def received_estop(self, msg):
        stop_node = msg
        # terminate plan at specified node
        for i, node in enumerate(self.plan):
            if node == stop_node:
                self.plan = self.plan[:i + 1]
                break

        self.check_estops = False

    def refresh_constraints():
        self.environment = Environment(bounds = self.bounds)
        obstacles = []
        for other_id, other_plan in self.plans.items():
            # refresh agent's plan if they have already moved to their next node
            if time.time() > other_plan[0].timestamp:
                self.plans[other_id] = other_plan[1:]

            # TODO add obstacle to obstacles representing agent at first node in its updated plan
            # obstacles.append(agent at self.plans[other_id][0])

        self.environment.add_obstacles(obstacles)

    def individual(self):
        """
        Individual component of DMA-RRT as described in algorithm 4
        from Desaraju/How 2012.
        """
        # new_plan = CL_RRT(bounds, self.environment, self.pose, self.goal, radius=0.1, delta_t=0.1)

        # refresh environment to reflect other agents' current positions
        self.refresh_constraints()
        self.rrt = RRTstar(self.pose, self.goal, self.environment, self.goal_dist)
        new_plan = self.rrt.get_path()

        if self.token_holder:
            self.plan = new_plan
            winner = max(self.bids, key = lambda x: self.bids[x])
            self.broadcast_waypoints(winner)
            self.token_holder = False
        else:
            bid = self.plan.cost() - new_plan.cost()
            self.broadcast_bid(bid)

    def check_emergency_stops(self, best_new_plan, other_agent):
        """
        Helper for the individual component of Cooperative DMA-RRT
        as described in algorithm 7 from Desaraju/How 2012.
        """
        other_agent_modified = False

        if self.check_estops:
            for estop_node in other_agent.plan.emergency_stops:
                for stop_node in best_new_plan.emergency_stops:
                    pass
                    # TODO: find last safe stop_node in our plan if other agend stops at estop_node
            # TODO: see pseudocode
        else:
            # TODO: prune best new plan to satisfy all constraints
            self.check_estops = True

        return best_new_plan, other_agent_modified

    def coop_individual(self):
        """
        Individual component of Cooperative DMA-RRT as described
        in algorithm 6 from Desaraju/How 2012.
        """
        # new_plan = CL_RRT(bounds, environment, self.current_position, end_position, radius=0.1, delta_t=0.1)

        # refresh environment to reflect other agents' current positions
        self.refresh_constraints()
        self.rrt = RRTstar(self.pose, self.goal, self.environment, self.goal_dist)
        new_plan = self.rrt.get_path()

        if self.token_holder:
            #check for first conflict (j)
                #check for second conflict (j')
            #ID emergency stop nodes
            self.plan = new_plan
            #if modified angent js plan, j is winner, else:
            winner = max(self.bids, key = lambda x: self.bids[x])
            self.broadcast_waypoints(winner)
            self.token_holder = False
        else:
            bid = self.plan.cost() - new_plan.cost()
            self.broadcast_bid(bid)

    def spin(self, rate, coop = False):
        """
        Runs the agent's individual component on a timer until it is sufficiently close to the goal.

        rate - rate at which the individual algorithm is run, in Hz
        coop - indicates whether to run the Cooperative DMA-RRt extension
        """
        x, y = self.pose
        x_goal, y_goal = self.goal

        # go until agent has reached its goal state
        while ((x - x_goal)**2 + (y - y_goal)**2) ** 1/2 > self.tolerance:
            # move agent if we have reached the next node
            if time.time() > self.plan[0].timestamp:
                self.pose = plan[0]
                self.plan = plan[1:]

            # run one iteration of the individual method for DMA-RRT
            if coop:
                self.coop_individual()
            else:
                self.individual()

            time.sleep(1 / rate)
