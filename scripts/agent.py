import time
import numpy as np
import rospy
from shapely.geometry import Point

from rrtstar import Node, NodeStamped, RRTstar, Path


class Agent:
    def __init__(
        self,
        identifier,
        mode,
        start_pos,
        goal_pos,
        environment,
        goal_dist,
        rrt_iters,
        peer_pub,
        bid_pub,
        waypoints_pub,
    ):
        """ Initializer for Agent class; represents a robot capable of planning
            and moving in an environment.

            Args:
                mode: a string: "normal" or "cooperative" indicates whether to use
                    DMA-RRT or Cooperative DMA-RRT (respectively).
                identifier: string representing the unique name of this agent
                start_pos: a 2-tuple representing the starting x-y position.
                goal_pos: a 2-tuple representing the goal x-y position.
                environment: an Environment object representing the map in which
                    the agent must plan.
                goal_dist: a float representing the length of a single branch in
                    the RRT tree for the agent's planner.
                rrt_iters: the number of iterations to run for RRT at each
                    spin_once.
        """
        self.identifier = identifier
        self.peer_pub = peer_pub
        self.bid_pub = bid_pub
        self.waypoints_pub = waypoints_pub

        # TODO: how should this be updated?
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

        self.rrt = RRTstar(
            self.start, self.goal, self.environment, goal_dist, max_iter=rrt_iters
        )
        self.goal_dist = goal_dist
        # curr_plan is the currently executing plan; best_plan is a lower-cost path
        #     than curr_plan, if one exists. The cost difference is the bid!
        self.curr_plan = Path()
        self.best_plan = Path()

        # TODO: new attributes needed for new pseudocode
        self.replan_token_holder = False
        self.goal_token_holder = False
        self.replan_bids = dict()
        self.goal_bids = dict()
        self.queue = []

    ##########################################################################

    def broadcast_bid(self, bid):
        msg = {"topic": TOPIC_BIDS, "bid": bid}
        self.antenna.broadcast(TOPIC_BIDS, msg)

    def broadcast_waypoints(self, winner_id):
        msg = {"topic": TOPIC_WAYPOINTS, "plan": self.curr_plan, "winner_id": winner_id}
        self.antenna.broadcast(TOPIC_WAYPOINTS, msg)

    ##########################################################################

    def received_id(self, data):
        if data.sender_id != self.identifier:
            self.bids[data.sender_id] = 0.0
            self.other_agent_plans[data.sender_id] = Path()

    def received_bid(self, sender_id, msg):
        """
        If the agent is not the one who sent the message, updates
        its `bids` dictionary to reflect the bid of the agent who did
        send the message.
        sender_id - the id of the agent who sent the message
        msg - a dictionary containing the PPI bid of the agent who
        sent the message under the key "bid"
        """
        if sender_id != self.identifier:
            bidder = sender_id
            self.bids[bidder] = msg["bid"]

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

        # TODO: add a check for having all the bids

        winner_id = msg["winner_id"]
        if winner_id == self.identifier:
            self.token_holder = True
        if sender_id != self.identifier:
            other_plan = msg["plan"]
            self.other_agent_plans[sender_id] = other_plan

        # TODO: clear the bids too?
        # self.bids = dict()

    ### REPLACING current received_waypoints (remove `2`): ###
    def received_waypoints2(self, msg):
        '''
        Update internal state to reflect constraints based on
        other agent's new planned path.

        msg - message of type Waypoints
        msg.sender_id - unique ID of the agent who sent the message
        msg.locations - path of waypoints
        '''
        if msg.sender_id != self.identifier:
            self.other_agent_plans[msg.sender_id] = msg.locations

    ### REPLACING winner functionality from received_waypoints: ###
    def received_replan_winner(self, msg):
        '''
        If winner, update internal state to hold the token.

        msg - message of type TokenHolder (TODO: rename to ReplanWinner?)
        msg.sender_id - unique ID of the agent who sent the message
        msg.holder_id - unique ID of the agent who has won the replanning token
        '''
        self.replan_token_holder = True

    def received_goal_winner(self, msg):
        '''
        If winner, update internal state to hold the token.

        msg - message of type TODO (GoalWinner?)
        msg.sender_id - unique ID of the agent who sent the message
        msg.holder_id - unique ID of the agent who has won the goal claiming token
        '''
        self.goal_token_holder = True

    ### REPLACING received_bid: ###
    def received_replan_bid(self, msg):
        '''
        Update internal state to reflect other agent's PPI bid.

        msg - message of type PlanBid
        msg.sender_id - unique ID of the agent who sent the message
        msg.bid - agent's PPI (potential path improvement) for current
        planning iteration
        '''
        if msg.sender_id != self.identifier:
            self.replan_bids[msg.sender_id] = msg.bid

    ### REPLACING received_bid: ###
    def received_goal_bid(self, msg):
        '''
        Update internal state to reflect other agent's bid
        on the first goal in the queue.

        msg - message of type TODO (GoalBid?)
        msg.sender_id - unique ID of the agent who sent the message
        msg.bid - agent's bid (reward - cost to get to goal) for current first goal in queue
        msg.goal_id - location of the goal other agent is bidding on
        '''
        if msg.sender_id != self.identifier:
            self.goal_bids[msg.sender_id] = msg.bid

    def received_add_goal(self, msg):
        '''
        Add the new goal to internal representation of goal queue,
        updating queue to remain sorted by reward.

        msg - message of type TODO (Goal)
        msg.goal_id - location of the goal to be added
        msg.reward - reward associated with that goal (e.g. from adaptive sampling)
        '''
        self.queue_insert(msg.goal_id, msg.goal_reward)

    def received_remove_goal(self, msg):
        '''
        Remove the specified goal from internal representation of goal
        queue, updating queue to remain sorted by reward.

        msg - message of type TODO (Goal)
        msg.goal_id - location of the goal to be removed
        msg.reward - reward associated with that goal (e.g. from adaptive sampling)
        '''
        self.queue_remove(msg.goal_id)

    def queue_insert(self, goal, reward):
        '''
        Add the goal to the queue attribute, keeping the queue
        sorted by TODO (reward, priority, etc.)
        '''
        pass

    def queue_remove(self, goal):
        '''
        Remove the goal from the queue attribute.
        '''
        pass

    ##########################################################################

    def at_goal(self):
        """ Checks if the agent's current location is the goal location. """
        dist = np.sqrt(
            (self.pos[0] - self.goal[0]) ** 2 + (self.pos[1] - self.goal[1]) ** 2
        )
        if dist <= 0.3:
            return True
        return False

    def update_time(self, curr_time):
        """ Update the internal time counter. """
        self.curr_time = curr_time

    def spin_once(self):
        """ Runs the agent's individual DMA-RRT component once.

            The interaction component is handled using Agent callbacks.
        """
        # Move agent if we have reached the next node
        if self.curr_plan.nodes:
            # TODO: maybe round the time?
            if self.curr_time in self.curr_plan.ts_dict.keys():
                self.pos = (
                    self.curr_plan.ts_dict[self.curr_time].x,
                    self.curr_plan.ts_dict[self.curr_time].y,
                )

            # curr_time may have moved on past the plans timestamps
            elif self.curr_time > self.curr_plan.nodes[-1].stamp:
                self.pos = (self.curr_plan.nodes[-1].x, self.curr_plan.nodes[-1].y)

        # TODO: broadcast bid
