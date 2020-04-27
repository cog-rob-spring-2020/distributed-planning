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
        """
        Initializer for Agent class; represents a robot capable of
        planning and moving in an environment.

        mode - a string: "normal" or "cooperative" indicates whether to
            use DMA-RRT or Cooperative DMA-RRT (respectively).
        identifier - string representing the unique name of this agent
        start_pos - a 2-tuple representing the starting x-y position.
        goal_pos - a 2-tuple representing the goal x-y position.
        environment - an Environment object representing the map in
            which the agent must plan.
        goal_dist - a float representing the length of a single branch
            in the RRT tree for the agent's planner.
        rrt_iters - the number of iterations to run for RRT at each
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
        # curr_plan is the currently executing plan; best_plan is a
        #    lower-cost path than curr_plan, if one exists.
        #    The cost difference is the bid!
        self.curr_plan = Path()
        self.best_plan = Path()

        # TODO: new attributes needed for new pseudocode
        # (fix these and place them in positions that make sense)
        self.replan_token_holder = False
        self.goal_token_holder = False
        self.replan_bids = dict()
        self.goal_bids = dict()
        self.queue = []

    ####################################################################

    ### TODO: REPLACE with below ###
    def broadcast_bid(self, bid):
        msg = {"topic": TOPIC_BIDS, "bid": bid}
        self.antenna.broadcast(TOPIC_BIDS, msg)

    ### TODO: REPLACE with below ###
    def broadcast_waypoints(self, winner_id):
        msg = {"topic": TOPIC_WAYPOINTS, "plan": self.curr_plan, "winner_id": winner_id}
        self.antenna.broadcast(TOPIC_WAYPOINTS, msg)

    ### TODO: REPLACING broadcast_waypoints ###
    def broadcast_waypoints_and_replan_winner(self, winner_id):
        """
        Broadcasts the following message to the TODO topic:

        msg - message of type TODO (blend of Waypoints/TokenHolder)
        msg.sender_id - this agent's unique ID
        msg.winner_id - unique ID of the agent who has won the
         replanning token, given by `winner_id`
        msg.locations - path of waypoints representing this agent's
         current plan being executed
        """
        pass

    def broadcast_remove_goal_and_goal_winner(self, winner_id):
        """
        Broadcasts the following message to the TODO topic:

        msg - message of type TODO (GoalWinner?)
        msg.sender_id - this agent's unique ID
        msg.winner_id - unique ID of the agent who has won the goal
         claiming token, given by `winner_id`
        msg.goal_id - location of goal just claimed by this agent (so
         all others can remove from their queues)
        """
        pass

    ### TODO: REPLACING broadcast_bid ###
    def broadcast_replan_bid(self, bid):
        """
        Broadcasts the following message to the TODO topic:

        msg - message of type PlanBid
        msg.sender_id - this agent's unique ID
        msg.bid - this agent's PPI, given by `bid`
        """
        pass

    ### TODO: REPLACING broadcast_bid ###
    def broadcast_goal_bid(self, bid, bid_goal):
        """
        Broadcasts the following message to the TODO topic:

        msg - message of type TODO (GoalBid?)
        msg.sender_id - this agent's unique ID
        msg.bid - agent's bid (reward - cost to get to goal) for current favorite goal in queue, given by `bid`
        msg.goal_id - location of the goal this agent is bidding on, given by `bid_goal`
        """
        pass

    ####################################################################

    """
    TODO:
    - make sure these messages are actually getting sent at some point after the whole team has been initialized to populate all the agents' dictionaries
    - update to reflect new fields
    """
    def received_id(self, msg):
        """
        TODO
        """
        if msg.sender_id != self.identifier:
            self.bids[msg.sender_id] = 0.0
            self.other_agent_plans[msg.sender_id] = Path()

    ### TODO: REPLACE with below ###
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

    ### TODO: REPLACE with below ###
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

    ### TODO: REPLACING current received_waypoints ###
    def received_waypoints_and_replan_winner(self, msg):
        """
        Update internal state to reflect constraints based on
        other agent's new planned path.

        Also, if winner, update internal state to hold the token.

        msg - message of type TODO (blend of Waypoints/TokenHolder)
        msg.sender_id - unique ID of the agent who sent the message
        msg.winner_id - unique ID of the agent who has won the replanning token
        msg.locations - path of waypoints
        """
        if msg.sender_id != self.identifier:
            self.other_agent_plans[msg.sender_id] = msg.locations

        if msg.winner_id == self.identifier:
            self.replan_token_holder = True

    def received_remove_goal_and_goal_winner(self, msg):
        """
        If winner, update internal state to hold the token.

        Remove the goal that the previous winner claimed from queue.

        msg - message of type TODO (GoalWinner?)
        msg.sender_id - unique ID of the agent who sent the message
        msg.winner_id - unique ID of the agent who has won the goal claiming token
        msg.goal_id - location of goal claimed by previous winner
        """
        if msg.winner_id == self.identifier:
            self.goal_token_holder = True

        if msg.sender_id != self.identifier:
            self.queue_remove(msg.goal_id)

    ### TODO: REPLACING received_bid ###
    def received_replan_bid(self, msg):
        """
        Update internal state to reflect other agent's PPI bid.

        msg - message of type PlanBid
        msg.sender_id - unique ID of the agent who sent the message
        msg.bid - agent's PPI (potential path improvement) for current
        planning iteration
        """
        if msg.sender_id != self.identifier:
            self.replan_bids[msg.sender_id] = msg.bid

    ### TODO: REPLACING received_bid ###
    def received_goal_bid(self, msg):
        """
        Update internal state to reflect other agent's bid
        on the first goal in the queue.

        msg - message of type TODO (GoalBid?)
        msg.sender_id - unique ID of the agent who sent the message
        msg.bid - agent's bid (reward - cost to get to goal) for current first goal in queue
        msg.goal_id - location of the goal other agent is bidding on
        """
        if msg.sender_id != self.identifier:
            self.goal_bids[msg.sender_id] = (msg.bid, msg.goal_id)

    def received_add_goal(self, msg):
        """
        Add the new goal to internal representation of goal queue,
        updating queue to remain sorted by reward.

        This message comes from the process that is adding/removing
        goals to the queue, modeling something like adaptive
        sampling (thus there is no sender ID because a single
        agent did not send the message in our implementation).

        msg - message of type TODO (Goal)
        msg.goal_id - location of the goal to be added
        msg.reward - reward associated with that goal (e.g. from adaptive sampling)
        """
        self.queue_insert(msg.goal_id, msg.goal_reward)

    def received_remove_goal(self, msg):
        """
        Remove the specified goal from internal representation of goal
        queue, updating queue to remain sorted by reward.

        This message comes from the process that is adding/removing
        goals to the queue, modeling something like adaptive
        sampling (thus there is no sender ID because a single
        agent did not send the message in our implementation).

        msg - message of type TODO (Goal)
        msg.goal_id - location of the goal to be removed
        msg.reward - reward associated with that goal (e.g. from adaptive sampling)
        """
        self.queue_remove(msg.goal_id)

    def queue_insert(self, goal, reward):
        """
        Add the goal to the queue attribute, keeping the queue
        sorted by TODO (reward, priority, etc.)
        """
        pass

    def queue_remove(self, goal):
        """
        Remove the goal from the queue attribute.
        """
        pass

    ####################################################################

    def at_goal(self):
        """
        Checks if the agent's current location is the goal location.
        """
        dist = np.sqrt(
            (self.pos[0] - self.goal[0]) ** 2 + (self.pos[1] - self.goal[1]) ** 2
        )
        if dist <= 0.3:
            return True
        return False

    def update_time(self, curr_time):
        """
        Update the internal time counter.
        """
        self.curr_time = curr_time

    def dma_individual(self):
        """
        Individual component of DMA-RRT as described in algorithm 4
        from Desaraju/How 2012.

        If this agent holds the replanning token, it will update its
        internal plan and broadcast the next winner. Otw, it will
        broadcast its own PPI bid.

        If this agent holds the goal claiming token, it will claim
        its favorite goal and broadcast the next winner. Otw, it will
        broadcast its own new goal bid.
        """
        # Refresh environment to reflect agent's current positions
        self.rrt.update_pos(self.pos, self.curr_time, wipe_tree=True)

        # Grow the tree by one set of iterations
        self.rrt.spin(False)

        # Find the new best path in the tree
        new_plan = self.rrt.get_path()

        """
        TODO:
        rework RRT planning to sequentially plan from current position
        to first goal, then from first goal to second goal.
        curr_plan and best_plan should both go through both goals!
        """

        # Assign first "current path" found
        if not self.curr_plan.nodes:
            self.curr_plan = new_plan
        self.best_plan = new_plan

        if self.token_holder:
            # Replan to new best path
            self.curr_plan = self.best_plan

            # Solve collisions with time reallocation
            self.curr_plan = Plan.multiagent_aware_time_realloc(self.curr_plan,
                    self.other_agent_plans)

            # Broadcast the new winner of the bidding round
            if self.bids:
                self.token_holder = False
                winner_bid = max(self.bids.values())
                winner_ids = [id for id, bid in self.bids.items() \
                    if bid == winner_bid]
                winner_id = random.choice(winner_ids)
                self.broadcast_waypoints(winner_id)
        else:
            self.broadcast_bid(self.curr_plan.cost - self.best_plan.cost)

        # TODO:
        # if-else for other token (for getting goals off queue)
        # if self.goal_token_holder and you have space for a new goal:
        #   take first goal off queue
        #   compute winner of next round of goal bidding (ignoring bidders for goal we just claimed)
        #   broadcast winner (tells everyone to remove goal we claimed)
        # else:
        #   bid on our favorite goal in the queue

        # Prevent agent from getting the token if they finish.
        if self.at_goal():
            self.broadcast_bid(-1000.0)

    def spin_once(self):
        """
        Runs the agent's individual DMA-RRT component once.

        The interaction component is handled using Agent callbacks.
        """
        self.dma_individual()
