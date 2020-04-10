import time
from solutions.rrtstar import Node, RRTstar
from shapely.geometry import Point
from solutions.antenna import Antenna, TOPIC_BIDS, \
    TOPIC_WAYPOINTS, TOPIC_ESTOPS

class Agent():
    def __init__(self, mode, start_pos, goal_region, environment, goal_dist):
        """ Initializer for Agent class; represents a robot capable of planning
            and moving in an environment.

            Args:
                mode: a string: "normal" or "cooperative" indicates whether to use
                    DMA-RRT or Cooperative DMA-RRT (respectively).
                start_pos: a 2-tuple representing the starting x-y position.
                goal_region: a 2-tuple representing the goal x-y position.
                environment: an Environment object representing the map in which
                    the agent must plan.
                goal_dist: a float representing the length of a single branch in 
                    the RRT tree for the agent's planner.
        """
        self.antenna = Antenna()
        self.antenna.find_peers()
        self.curr_time = 0.0  # Simulation time. Updated externally.

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
        self.other_agent_plans = {id:None for id in self.antenna.peers}

        # initial state and environment
        self.environment = environment

        self.pos = Point(self.start_pos[0], self.start_pos[1])
        self.goal = goal_region

        self.rrt = RRTstar(self.start_pos, self.goal, self.environment, goal_dist)

        # curr_plan is the currently executing plan; best_plan is a lower-cost path
        #   than curr_plan, if one exists. The cost difference is the bid!
        self.curr_plan = Path()
        self.best_plan = None

        # register as listener for different kinds of messages
        self.antenna.on_message("bids", self.received_bid)
        self.antenna.on_message("waypoints", self.received_waypoints)
        self.antenna.on_message("estop", self.received_estop)

    """ The following methods represent the interaction component of DMA-RRT
        as described in algorithms 5 and 8.
    """
    ##########################################################################
    def broadcast_bid(self, bid):
        msg = {"topic":TOPIC_BIDS, "bid":bid}

        self.antenna.broadcast(TOPIC_BIDS, msg)

    def received_bid(self, sender_id, msg):
        bidder = sender_id
        self.bids[bidder] = msg["bid"]

    def broadcast_waypoints(self, winner_id):
        msg = {"topic":TOPIC_WAYPOINTS, "plan":self.curr_plan, "winner_id":winner_id}

        self.antenna.broadcast(TOPIC_WAYPOINTS, msg)

    def received_waypoints(self, sender_id, msg):
        other_plan = msg["plan"]
        winner_id = msg["winner_id"]

        self.other_agent_plans[sender_id] = other_plan
        if winner_id == self.id:
            self.token_holder = True

    def broadcast_estop(self, stop_id, stop_node):
        msg = {"topic":TOPIC_ESTOPS, "stop_id":stop_id, "stop_node":stop_node}

        self.antenna.broadcast(TOPIC_ESTOPS, msg)

    def received_estop(self, sender_id, msg):
        stop_id = msg["stop_id"]
        stop_node = msg["stop_node"]

        if stop_id == self.antenna.uuid:
            # Terminate plan at specified node
            # TODO(marcus): update this to Path API.
            for i, node in enumerate(self.curr_plan.nodes):
                if node == stop_node:
                    self.curr_plan = self.curr_plan[:i + 1]
                    break

            self.check_estops = False
    ##########################################################################

    def at_goal(self):
        """ Checks if the agent's current location is the goal location. """
        return self.pos == self.goal

    def update_time(self, curr_time):
        """ Update the internal time counter. """
        self.curr_time = curr_time

    def individual(self):
        """ Individual component of DMA-RRT as described in algorithm 4
            from Desaraju/How 2012.

            If this agent is the TokenHolder, it will update its internal
            plan and broadcast the next winner. Otw, it will broadcast its bid.
        """
        # Refresh environment to reflect agents' current positions
        self.rrt.update_agent_plans(self.other_agent_plans)
        self.rrt.update_pose(self.antenna.uuid, self.pos)

        # Grow the tree by one iteration.
        self.rrt.spin_once(False)

        # Find the new best path in the tree, and save it if it's better.
        new_plan = self.rrt.get_path()
        if new_plan.cost < self.best_plan.cost and new_plan.cost < self.cur_plan.cost:
            self.best_plan = new_plan

        if self.token_holder:
            # Replan to new best path if it's better.
            if self.best_plan.cost < self.curr_plan.cost:
                self.curr_plan = self.best_plan
                self.best_plan = Path()

            # Broadcast the new winner of the bidding round.
            winner_id = max(self.bids, key = lambda x: self.bids[x])
            self.broadcast_waypoints(winner_id)
            self.token_holder = False
        else:
            # Broadcast own bid.
            bid = self.curr_plan.cost - self.best_plan.cost
            self.broadcast_bid(bid)

    def check_emergency_stops(self, best_new_plan, other_agent, other_plan):
        """ Helper for the individual component of Cooperative DMA-RRT
            as described in algorithm 7 from Desaraju/How 2012.

            Checks for emergency stops between this agent and another agent
            and both current plans.
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
        """ Individual component of Cooperative DMA-RRT as described
            in algorithm 6 from Desaraju/How 2012.
        """
        # Grow tree one iteration while ignoring other agents' paths.
        self.rrt.update_agent_plans(dict())
        self.rrt.update_pose(self.pos)
        self.rrt.spin_once(False)

        # Find the new best path in the tree, and save it if it's better.
        new_plan = self.rrt.get_path()
        if new_plan.cost < self.best_plan.cost and new_plan.cost < self.cur_plan.cost:
            self.best_plan = new_plan

        if self.token_holder:
            # Replan to new best path if it's better.
            plan = self.curr_plan
            if self.best_plan.cost < self.curr_plan.cost:
                plan = self.best_plan
                self.best_plan = Path()
            
            conflicting_plans = RRTstar.get_conflicts(plan)
            conflicting_ids = list(conflicting_plans.keys())

            j = None
            if conflicting_ids:
                j = conflicting_ids[0]
                plan, modified_j = check_emergency_stops(plan, j, conflicting_plans[j])

            if len(conflicting_ids > 1):
                for j_prime in conflicting_ids[1:]:
                    j_prime_plan = conflicting_plans[j_prime]
                    if self.rrt.plans_conflict(plan, j_prime_plan):
                        # Prune our plan to avoid the conflict
                        plan = rrt.prune_to_avoid_conflict(new_plan, j_prime_plan)

            self.curr_plan = plan

            # Broadcast the new winner of the bidding round.
            if j is not None and modified_j:
                winner = j
            else:
                winner = max(self.bids, key = lambda x: self.bids[x])
            self.broadcast_waypoints(winner)
            self.token_holder = False
        else:
            # Broadcast own bid.
            bid = self.curr_plan.cost - self.best_plan.cost
            self.broadcast_bid(bid)

    def spin(self, rate):
        """ Runs the agent's individual component on a timer until it is 
            sufficiently close to the goal.

            Args:
                rate: float representing the rate for the spin, in Hz.
        """
        # go until agent has reached its goal state
        while not self.at_goal():
            self.spin_once()
            time.sleep(1.0 / rate)

    def spin_once(self):
        """ Runs the agent's individual DMA-RRT component once. 
            
            The interaction component is handled using Agent callbacks.
        """
        # Move agent if we have reached the next node
        if self.curr_time > self.curr_plan[0].timestamp:
            self.pos = plan[0]
            self.curr_plan = plan[1:]

        # run one iteration of the individual method for DMA-RRT
        if self.mode == "cooperative":
            self.coop_individual()
        elif self.mode == "normal":
            self.individual()