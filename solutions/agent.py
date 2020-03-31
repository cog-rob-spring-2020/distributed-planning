from plan import Path
from pubsub import pub
from cl_rrt import CL_RRT

class Agent():
    def __init__(self, id, other_ids):
        """
        id - unique integer id
        other_ids - list of other agents' integer ids
        """
        self.id = id
        self.token_holder = False
        self.check_estops = True

        # keeps track of other agents' current bids for PPI
        # (potential path improvement) at any given time:
        self.bids = {id:None for id in other_ids}

        # initial state (e.g. pose)
        # TODO
        self.plan = Path()

        # model of vehicle dynamics
        # TODO

        # attributes to support keeping a continually updated RRT process
        # TODO
        self.current_position = None

        # constraints (will be updated according to other agents' paths)
        # TODO

        # register as listener for different kinds of messages
        pub.subscribe(self.received_bid, "bids")
        pub.subscribe(self.received_waypoints, "waypoints")
        pub.subscribe(self.received_estop, "estop")

    """
    Methods for simulating motion using vehicle dynamics
    and calling Cam's CL-RRT code.
    """
    # TODO
    steer(self):
        # TODO:
        # update self.current_position using self.plan
        pass

    """
    Methods for listening and broadcasting to various topics.

    The listener methods comprise the interaction component of DMA-RRT
    and Cooperative DMA-RRT from Desaraju/How 2012, as described in algorithms 5 and 8 respectively.
    """
    def broadcast_bid(self, bid):
        pub.sendMessage("bids", self.id, bid)

    def received_bid(self, bidder_id, bid):
        self.bids[bidder_id] = bid

    def broadcast_waypoints(self, winner_id):
        pub.sendMessage("waypoints", self.id, self.plan, winner_id)

    def received_waypoints(self, other_id, other_plan, winner_id):
        # TODO:
        # simulate other agent's trajectory according to their plan
        # and use it to update constraints for own planning

        if winner_id == self.id:
            self.token_holder = True

    def broadcast_estop(self, stop_node):
        pub.sendMessage("estop", stop_node)

    def received_estop(self, stop_node):
        # terminate plan at specified node
        for i, node in enumerate(self.plan):
            if node == stop_node:
                self.plan = self.plan[:i + 1]
                break

        self.check_estops = False

    def individual(self, bounds, environment, start_position, end_position, radius, delta_t):
        """
        Individual component of DMA-RRT as described in algorithm 4
        from Desaraju/How 2012.
        """
        self.plan.add_node(start_position)
        self.current_position = start_position
        while True: #while active - need check for goal state
            new_plan = CL_RRT(bounds, environment, self.current_position, end_position, radius=0.1, delta_t=0.1)
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
        while True: #while active - need check for goal state
            new_plan = CL_RRT(bounds, environment, self.current_position, end_position, radius=0.1, delta_t=0.1)
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
