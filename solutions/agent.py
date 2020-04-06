import time
from solutions.rrtstar import Node, RRTstar
from shapely.geometry import Point
from solutions.antenna import Antenna, TOPIC_BIDS, TOPIC_WAYPOINTS, TOPIC_ESTOPS

class Agent():
    def __init__(self, mode, x_start, y_start, goal_region, environment, bounds, goal_dist):
        """
        mode - "normal" or "cooperative" indicates whether to use DMA-RRT or Cooperative DMA-RRT
        """
        self.antenna = Antenna()
        self.antenna.find_peers()

        self.mode = mode
        if mode != "cooperative" and mode != "normal":
            raise ValueError("mode must be 'normal' or 'cooperative'")

        self.token_holder = False
        self.check_estops = (mode == 'cooperative')

        # keeps track of other agents' current bids for PPI
        # (potential path improvement) at any given time:
        self.bids = {id:None for id in self.antenna.peers}

        # keeps track of other agents' plans so that we can
        # add the other agents as obstacles when replanning
        self.plans = {id:None for id in self.antenna.peers}

        # initial state and environment
        self.goal_dist = goal_dist
        self.bounds = bounds
        self.environment = environment

        self.pose = Point(x_start, y_start)
        self.goal = goal_region

        self.rrt = RRTstar((x_start, y_start), self.goal, self.environment, self.goal_dist)

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
        msg = {"topic":TOPIC_BIDS, "bid":bid}

        self.antenna.broadcast(TOPIC_BIDS, msg)

    def received_bid(self, sender_id, msg):
        bidder = sender_id
        self.bids[bidder] = msg["bid"]

    def broadcast_waypoints(self, winner_id):
        msg = {"topic":TOPIC_WAYPOINTS, "plan":self.plan, "winner_id":winner_id}

        self.antenna.broadcast(TOPIC_WAYPOINTS, msg)

    def received_waypoints(self, sender_id, msg):
        other_plan = msg["plan"]
        winner_id = msg["winner_id"]

        self.plans[sender_id] = other_plan
        if winner_id == self.id:
            self.token_holder = True

    def broadcast_estop(self, stop_id, stop_node):
        msg = {"topic":TOPIC_ESTOPS, "stop_id":stop_id, "stop_node":stop_node}

        self.antenna.broadcast(TOPIC_ESTOPS, msg)

    def received_estop(self, sender_id, msg):
        stop_id = msg["stop_id"]
        stop_node = msg["stop_node"]

        if stop_id == self.antenna.uuid:
            # terminate plan at specified node
            for i, node in enumerate(self.plan):
                if node == stop_node:
                    self.plan = self.plan[:i + 1]
                    break

            self.check_estops = False

    def individual(self):
        """
        Individual component of DMA-RRT as described in algorithm 4
        from Desaraju/How 2012.
        """
        # refresh environment to reflect agents' current positions
        self.rrt.update_agent_plans(self.plans)
        self.rrt.update_pose(self.antenna.uuid, self.pose)

        # grow the tree to find best path given current environment
        new_plan = self.rrt.get_path()

        if self.token_holder:
            self.plan = new_plan
            winner_id = max(self.bids, key = lambda x: self.bids[x])
            self.broadcast_waypoints(winner_id)
            self.token_holder = False
        else:
            bid = self.plan.cost() - new_plan.cost()
            self.broadcast_bid(bid)

    def check_emergency_stops(self, best_new_plan, other_agent, other_plan):
        """
        Helper for the individual component of Cooperative DMA-RRT
        as described in algorithm 7 from Desaraju/How 2012.
        """
        other_agent_modified = False

        if self.check_estops:
            opt_other_stop = None
            opt_stop = None
            opt_global_cost = None

            for other_i in range(0, len(other_plan), 10):
                for i in range(0, len(best_new_plan), 10):
                    other_stop_node = other_plan[other_i]
                    stop_node = best_new_plan[i]

                    if self.rrt.plans_conflict(other_plan[:other_i + 1], best_new_plan[:stop_node + 1]):
                        # only want to consider this combination of stops if it would avoid a conflict
                        continue

                    cost = other_stop_node.cost + stop_node.cost
                    if opt_global_cost is None or cost < opt_global_cost:
                        opt_other_stop = other_stop_node
                        opt_stop = stop_node
                        opt_global_cost = cost

            if opt_other_stop != other_plan[-1]:
                self.broadcast_estop(other_agent, opt_other_stop)
                other_agent_modified = True
            if opt_stop != best_new_plan[-1]:
                for i, node in enumerate(best_new_plan):
                    if node == stop_node:
                        best_new_plan = best_new_plan[:i + 1]
                        break
        else:
            # prune our plan to avoid the conflict (don't modify other agent's plan)
            best_new_plan = rrt.prune_to_avoid_conflict(best_new_plan, other_plan)

            self.check_estops = True

        return best_new_plan, other_agent_modified

    def coop_individual(self):
        """
        Individual component of Cooperative DMA-RRT as described
        in algorithm 6 from Desaraju/How 2012.
        """
        # grow tree while ignoring other agents' paths
        self.rrt.update_agent_plans(dict())
        self.rrt.update_pose(self.pose)
        new_plan = self.rrt.get_path()

        if self.token_holder:
            conflicting_plans = RRTstar.get_conflicts(self.plan)
            conflicting_ids = list(conflicting_plans.keys())

            j = None
            if conflicting_ids:
                j = conflicting_ids[0]
                new_plan, modified_j = check_emergency_stops(new_plan, j, conflicting_plans[j])

            if len(conflicting_ids > 1):
                for j_prime in conflicting_ids[1:]:
                    j_prime_plan = conflicting_plans[j_prime]
                    if self.rrt.plans_conflict(new_plan, j_prime_plan):
                        # prune our new_plan to avoid the conflict
                        new_plan = rrt.prune_to_avoid_conflict(new_plan, j_prime_plan)

            self.plan = new_plan

            if j is not None and modified_j:
                winner = j
            else:
                winner = max(self.bids, key = lambda x: self.bids[x])

            self.broadcast_waypoints(winner)
            self.token_holder = False
        else:
            bid = self.plan.cost() - new_plan.cost()
            self.broadcast_bid(bid)

    def at_goal(self):
        return self.goal.contains(self.pose)

    def spin(self, rate):
        """
        Runs the agent's individual component on a timer until it is sufficiently close to the goal.

        rate - rate at which the individual algorithm is run, in Hz
        """
        # go until agent has reached its goal state
        while not self.at_goal():
            # move agent if we have reached the next node
            # TODO fix the use of time here, or use steer instead
            if time.time() > self.plan[0].timestamp:
                self.pose = plan[0]
                self.plan = plan[1:]

            # run one iteration of the individual method for DMA-RRT
            if self.mode == "cooperative":
                self.coop_individual()
            elif self.mode == "normal":
                self.individual()

            time.sleep(1 / rate)
