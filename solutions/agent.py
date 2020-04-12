import time
import numpy as np
from solutions.rrtstar import Node, NodeStamped, RRTstar, Path
from shapely.geometry import Point
from solutions.antenna import Antenna, TOPIC_BIDS, \
                              TOPIC_WAYPOINTS, TOPIC_PEERS

class Agent:
    def __init__(self,
                 mode,
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
                start_pos: a 2-tuple representing the starting x-y position.
                goal_pos: a 2-tuple representing the goal x-y position.
                environment: an Environment object representing the map in which
                    the agent must plan.
                goal_dist: a float representing the length of a single branch in
                    the RRT tree for the agent's planner.
                rrt_iters: the number of iterations to run for RRT at each
                    spin_once
        """
        self.antenna = Antenna()

        self.curr_time = 0.0  # Simulation time. Updated externally.

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
        # curr_plan is the currently executing plan; best_plan is a lower-cost path
        #     than curr_plan, if one exists. The cost difference is the bid!
        self.curr_plan = Path()
        self.best_plan = Path()

        # Register as listener for different kinds of messages
        self.antenna.on_message("bids", self.received_bid)
        self.antenna.on_message("waypoints", self.received_waypoints)
        self.antenna.on_message("peers", self.received_id)

        # if mode != "cooperative" and mode != "normal":
        #     raise ValueError("mode must be 'normal' or 'cooperative'")
        # self.check_estops = (mode == 'cooperative')
        # self.antenna.on_message("estop", self.received_estop)

    """ The following methods represent the interaction component of DMA-RRT
        as described in algorithms 5 and 8.
    """
    ##########################################################################

    def broadcast_id(self):
        msg = {"topic":TOPIC_PEERS}
        self.antenna.broadcast(TOPIC_PEERS, msg)

    def received_id(self, sender_id, msg):
        if sender_id != self.antenna.uuid:
            self.bids[sender_id] = 0.0
            self.other_agent_plans[sender_id] = Path()

    def broadcast_bid(self, bid):
        msg = {"topic":TOPIC_BIDS, "bid":bid}
        self.antenna.broadcast(TOPIC_BIDS, msg)

    def received_bid(self, sender_id, msg):
        """
        If the agent is not the one who sent the message, updates
        its `bids` dictionary to reflect the bid of the agent who did
        send the message.

        sender_id - the id of the agent who sent the message
        msg - a dictionary containing the PPI bid of the agent who
        sent the message under the key "bid"
        """
        if sender_id != self.antenna.uuid:
            bidder = sender_id
            self.bids[bidder] = msg["bid"]

    def broadcast_waypoints(self, winner_id):
        msg = {"topic":TOPIC_WAYPOINTS, "plan":self.curr_plan, "winner_id":winner_id}
        self.antenna.broadcast(TOPIC_WAYPOINTS, msg)

    def received_waypoints(self, sender_id, msg):
        """
        If the agent is the winner, updates its state to become the
        token holder.

        If the agent is not the one who sent the message, also updates its
        `other_agent_plans` dictionary to reflect the updated plan of the
        agent who did send the message.

        sender_id - the id of the agent who sent the message
        msg - a dictionary containing the id of the winning agent under
        the key "winner_id", as well as the updated plan of the agent who
        sent the message under the key "plan"
        """
        winner_id = msg["winner_id"]
        if winner_id == self.antenna.uuid:
            self.token_holder = True

        if sender_id != self.antenna.uuid:
            other_plan = msg["plan"]
            self.other_agent_plans[sender_id] = other_plan

    ##########################################################################

    def at_goal(self):
        """ Checks if the agent's current location is the goal location. """
        dist = np.sqrt((self.pos[0]-self.goal[0])**2 + (self.pos[1]-self.goal[1])**2)
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

    ##########################################################################
    ############################# E-STOP-CODE ################################

    # def broadcast_estop(self, stop_id, stop_node):
    #     msg = {"topic":TOPIC_ESTOPS, "stop_id":stop_id, "stop_node":stop_node}

    #     self.antenna.broadcast(TOPIC_ESTOPS, msg)

    # def received_estop(self, sender_id, msg):
    #     stop_id = msg["stop_id"]
    #     stop_node = msg["stop_node"]
    #     if stop_id == self.antenna.uuid:
    #         # terminate plan at specified node
    #         for i, node in enumerate(self.plan):
    #             if node == stop_node:
    #                 self.plan = self.plan[:i + 1]
    #                 break

    #         self.check_estops = False

    # def check_emergency_stops(self, best_new_plan, other_agent, other_plan):
    #     """ Helper for the individual component of Cooperative DMA-RRT
    #         as described in algorithm 7 from Desaraju/How 2012.

    #         Checks for emergency stops between this agent and another agent
    #         and both current plans.
    #     """
    #     other_agent_modified = False

    #     if self.check_estops:
    #         opt_other_stop = None
    #         opt_stop = None
    #         opt_global_cost = None

    #         for other_i in range(0, len(other_plan), 10):
    #             for i in range(0, len(best_new_plan), 10):
    #                 other_stop_node = other_plan[other_i]
    #                 stop_node = best_new_plan[i]

    #                 if self.rrt.plans_conflict(other_plan[:other_i + 1], best_new_plan[:stop_node + 1]):
    #                     # only want to consider this combination of stops if it would avoid a conflict
    #                     continue

    #                 cost = other_stop_node.cost + stop_node.cost
    #                 if opt_global_cost is None or cost < opt_global_cost:
    #                     opt_other_stop = other_stop_node
    #                     opt_stop = stop_node
    #                     opt_global_cost = cost

    #         if opt_other_stop != other_plan[-1]:
    #             self.broadcast_estop(other_agent, opt_other_stop)
    #             other_agent_modified = True
    #         if opt_stop != best_new_plan[-1]:
    #             for i, node in enumerate(best_new_plan):
    #                 if node == stop_node:
    #                     best_new_plan = best_new_plan[:i + 1]
    #                     break
    #     else:
    #         # prune our plan to avoid the conflict (don't modify other agent's plan)
    #         best_new_plan = rrt.prune_to_avoid_conflict(best_new_plan, other_plan)

    #         self.check_estops = True

    #     return best_new_plan, other_agent_modified

    ##########################################################################
    ##########################################################################
