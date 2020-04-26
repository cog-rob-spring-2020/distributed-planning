import time
import numpy as np
from shapely.geometry import Point

from rrtstar import Node, NodeStamped, RRTstar, Path
from antenna import Antenna, TOPIC_BIDS, TOPIC_WAYPOINTS, TOPIC_PEERS


class Agent:
    def __init__(self,
                 mode,
                 received_bid,
                 received_waypoints,
                 start_pos,
                 goal_pos,
                 environment,
                 goal_dist,
                 rrt_iters):
        """ Initializer for Agent class; represents a robot capable of planning
            and moving in an environment.

            Args:
                mode: a string: "normal" or "cooperative" indicates whether to use
                    DMA-RRT or Cooperative DMA-RRT (respectively).
                received_bid: a method representing the callback for bid
                    messages passed over the network.
                received_waypoints: a method representing the callback for the
                    waypoints and winner_id messages passed over the network.
                start_pos: a 2-tuple representing the starting x-y position.
                goal_pos: a 2-tuple representing the goal x-y position.
                environment: an Environment object representing the map in which
                    the agent must plan.
                goal_dist: a float representing the length of a single branch in
                    the RRT tree for the agent's planner.
                rrt_iters: the number of iterations to run for RRT at each
                    spin_once.
        """
        self.antenna = Antenna()

        # Assign interaction callbacks for messages
        Agent.received_bid = received_bid
        Agent.received_waypoints = received_waypoints

        self.curr_time = 0.0  # Simulation time. Updated externally

        self.mode = mode
        self.token_holder = False

        # Keeps track of other agents' current bids for PPI
        #     (potential path improvement) at any given time:
        self.bids = dict()

        # Keeps track of other agents' plans so that we can
        #   add the other agents as obstacles when replanning:
        self.other_agent_plans = dict()

        # Initial state and environment
        self.start = start_pos
        self.goal = goal_pos
        self.pos = self.start
        self.environment = environment

        self.rrt = RRTstar(self.start, self.goal, self.environment, goal_dist,
                           max_iter=rrt_iters)
        self.goal_dist = goal_dist
        # curr_plan is the currently executing plan; best_plan is a lower-cost path
        #     than curr_plan, if one exists. The cost difference is the bid!
        self.curr_plan = Path()
        self.best_plan = Path()

        # Register as listener for different kinds of messages
        self.antenna.on_message("peers", self.received_id)
        self.antenna.on_message("bids", self.received_bid)
        self.antenna.on_message("waypoints", self.received_waypoints)

    """ The following methods represent the interaction component of DMA-RRT
        as described in algorithms 5 and 8.
    """
    ##########################################################################

    def broadcast_id(self):
        msg = {"topic": TOPIC_PEERS}
        self.antenna.broadcast(TOPIC_PEERS, msg)

    def received_id(self, sender_id, msg):
        if sender_id != self.antenna.uuid:
            self.bids[sender_id] = 0.0
            self.other_agent_plans[sender_id] = Path()

    def broadcast_bid(self, bid):
        msg = {"topic": TOPIC_BIDS, "bid": bid}
        self.antenna.broadcast(TOPIC_BIDS, msg)

    def broadcast_waypoints(self, winner_id):
        msg = {"topic": TOPIC_WAYPOINTS,
               "plan": self.curr_plan, "winner_id": winner_id}
        self.antenna.broadcast(TOPIC_WAYPOINTS, msg)

    def received_bid(self, sender_id, msg):
        """ Callback for messages over the network. """
        raise NotImplementedError

    def received_waypoints(self, sender_id, msg):
        """ Callback for messages over the network. """
        raise NotImplementedError

    ##########################################################################

    def at_goal(self):
        """ Checks if the agent's current location is the goal location. """
        dist = np.sqrt((self.pos[0]-self.goal[0])**2 +
                       (self.pos[1]-self.goal[1])**2)
        if dist <= 0.3:
            return True
        return False

    def update_time(self, curr_time):
        """ Update the internal time counter. """
        self.curr_time = curr_time

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
        if self.curr_plan.nodes:
            if self.curr_time in self.curr_plan.ts_dict.keys():
                self.pos = (self.curr_plan.ts_dict[self.curr_time].x,
                            self.curr_plan.ts_dict[self.curr_time].y)

            # curr_time may have moved on past the plans timestamps
            elif self.curr_time > self.curr_plan.nodes[-1].stamp:
                self.pos = (self.curr_plan.nodes[-1].x,
                            self.curr_plan.nodes[-1].y)


def test_received_bid(self, sender_id, msg):
    pass


def test_received_waypoints(self, sender_id, msg):
    pass
