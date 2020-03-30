from plan import Path
from pubsub import pub

class Agent():
    def __init__(id, other_ids):
        """
        id - unique integer id
        other_ids - list of other agents' integer ids
        """
        self.id = id
        self.token_holder = False
        self.check_estops = True  # TODO - figure out when this resets

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

        # constraints (will be updated according to other agents' paths)
        # TODO

        # register as listener for different kinds of messages
        pub.subscribe(self.received_bid, "bids")
        pub.subscribe(self.received_waypoints, "waypoints")
        pub.subscribe(self.received_estop, "estop")

    """
    Methods for moving around and running RRT (call Cam's CL-RRT code)
    """
    # TODO

    """
    Methods for listening and broadcasting to various topics.

    The listener methods comprise the interaction component of DMA-RRT
    and Cooperative DMA-RRT from Desaraju/How 2012, as described in algorithms 5 and 8 respectively.
    """
    def broadcast_bid(bid):
        pub.sendMessage("bids", self.id, bid)

    def received_bid(bidder_id, bid):
        self.bids[bidder_id] = bid

    def broadcast_waypoints(winner_id):
        pub.sendMessage("waypoints", self.id, self.plan, winner_id)

    def received_waypoints(other_id, other_plan, winner_id):
        # TODO:
        # simulate other agent's trajectory according to their plan
        # and use it to update constraints for own planning

        if winner_id == self.id:
            self.token_holder = True

    def broadcast_estop(stop_node):
        pub.sendMessage("estop", stop_node)

    def received_estop(stop_node):
        # terminate plan at specified node
        for i, node in enumerate(self.plan):
            if node == stop_node:
                self.plan = self.plan[:i + 1]
                break
        
        self.check_estops = False

    def individual():
        """
        Individual component of DMA-RRT as described in algorithm 4
        from Desaraju/How 2012.
        """
        # TODO
        pass

    def check_emergency_stops():
        """
        Helper for the individual component of Cooperative DMA-RRT
        as described in algorithm 7 from Desaraju/How 2012.
        """
        # TODO
        pass

    def coop_individual():
        """
        Individual component of Cooperative DMA-RRT as described
        in algorithm 6 from Desaraju/How 2012.
        """
        # TODO
        pass
