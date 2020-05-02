#!/usr/bin/env python
import random
import rospy
import yaml
from distributed_planning.msg import *
from environment import Environment
import argparse
import time
import numpy as np

from plan import Plan
from rrtstar import Node, NodeStamped, RRTstar, Path


class DMARRTAgent(object):
    """
    Represents a robot capable of
    planning and moving in an environment using DMA-RRT.
    Use as a base for decentralized planning algorithms.
    """

    def __init__(self, start_pos, goal_pos, environment, goal_dist, rrt_iters):
        """
        start_pos - a 2-tuple representing the starting x-y position.
        goal_pos - a 2-tuple representing the goal x-y position.
        environment - an Environment object representing the map in
            which the agent must plan.
        goal_dist - a float representing the length of a single branch
            in the RRT tree for the agent's planner.
        rrt_iters - the number of iterations to run for RRT at each
            spin_once.
        """

        # Initial state and environment
        self.start = start_pos
        self.pos = self.start

        self.goal = goal_pos
        self.goal_dist = goal_dist

        self.environment = environment

        self.rrt = RRTstar(self.start, self.goal, self.environment, goal_dist, max_iter=rrt_iters)

        # curr_plan is the currently executing plan; best_plan is a
        #    lower-cost path than curr_plan, if one exists.
        #    The cost difference is the bid!
        self.curr_plan = Path()
        self.best_plan = Path()

        # a unique name given to us by ROS
        self.identifier = rospy.get_name()

        # be ready for replanning bids
        rospy.Subscriber("plan_bids", PlanBid, self.received_plan_bid)
        self.plan_bid_pub = rospy.Publisher("plan_bids", PlanBid, queue_size=10)

        # be ready for updated waypoints from the winner
        rospy.Subscriber("waypoints", Waypoints, self.received_waypoints_and_replan_winner)
        self.waypoints_pub = rospy.Publisher("waypoints", Waypoints, queue_size=10)

        # let the other agents know a new agent is on the network
        rospy.Subscriber("registration", Registration, self.received_registration)
        registration_pub = rospy.Publisher("registration", Registration, queue_size=10)
        msg = Registration(sender_id=rospy.get_name())
        registration_pub.publish(msg)

        # Keeps track of other agents' current bids for PPI (potential path improvement) at any given time
        self.plan_bids = dict()
        # paths by peer ID. all peers should be represented once here, and as such calling `len(self.peer_waypoints)` should give an accurate count of peers
        self.peer_waypoints = dict()
        # whether or not this agent is holding the replan token
        self.plan_token_holder = False

        # TODO: how should this be updated?
        self.curr_time = 0.0  # Simulation time. Updated externally

    ####################################################################
    
    def broadcast_replan_bid(self, bid):
        """
        Broadcasts the following message to the plan_bids topic

        msg - message of type PlanBid
        msg.sender_id - this agent's unique ID
        msg.bid - this agent's PPI, given by `bid`
        """
        msg = PlanBid()
        msg.sender_id = self.identifier
        msg.bid = bid
        self.plan_bid_pub.publish(msg)

    ####################################################################

    def received_registration(self, msg):
        """
        We met a peer, let's note that we're tracking their path and bids
        """
        if msg.sender_id != self.identifier:
            self.peer_waypoints[msg.sender_id] = Path()

    def received_plan_bid(self, msg):
        """
        Update internal state to reflect other agent's PPI bid.

        msg - message of type PlanBid
        msg.sender_id - unique ID of the agent who sent the message
        msg.bid - agent's PPI (potential path improvement) for current
        planning iteration
        """
        if msg.sender_id != self.identifier:
            self.plan_bids[msg.sender_id] = msg.bid

    def received_waypoints_and_replan_winner(self, msg):
        """
        Interactive component of DMA-RRT as described in algorithm 5 from Desaraju/How 2012.

        Update internal state to reflect constraints based on
        other agent's new planned path.

        Also, if winner, update internal state to hold the token.

        msg - message of type Waypoints
        msg.sender_id - unique ID of the agent who sent the message
        msg.winner_id - unique ID of the agent who has won the replanning token
        msg.locations - path of waypoints
        """
        if msg.winner_id == self.identifier:
            self.plan_token_holder = True
        if msg.sender_id != self.identifier:
            self.peer_waypoints[msg.sender_id] = msg.waypoints

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

    def create_new_plan(self):
        """
        Spin RRT to create a new plan

        Returns:
            Path
        """
        # Refresh environment to reflect agent's current positions
        self.rrt.update_pos(self.pos, self.curr_time, wipe_tree=True)

        # Grow the tree by one set of iterations
        self.rrt.spin(False)

        # Find the new best path in the tree
        return self.rrt.get_path()

    def spin_once(self):
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
        # TODO: do we need a timeout here instead of rrt_iters???
        new_plan = self.create_new_plan()

        # Assign first "current path" found
        if not self.curr_plan.nodes:
            self.curr_plan = new_plan
        self.best_plan = new_plan

        if self.plan_token_holder:
            # Replan to new best path
            self.curr_plan = self.best_plan

            # Solve collisions with time reallocation
            self.curr_plan = Plan.multiagent_aware_time_realloc(
                self.curr_plan, self.peer_waypoints
            )

            # Broadcast the new winner of the bidding round
            winner_id = self.identifier #default to being the winner again
            if len(self.plan_bids) > 0:
                # select a winner based on bids
                winner_bid = max(self.plan_bids.values())
                winner_ids = [id for id, bid in self.plan_bids.items() if bid == winner_bid]
                # break bid ties with randomness
                winner_id = random.choice(winner_ids)
                self.plan_token_holder = False

            # broadcast own waypoints and new token holder
            msg = Waypoints()
            msg.sender_id = self.identifier
            msg.winner_id = winner_id
            msg.waypoints = self.best_plan
            self.waypoints_pub.publish(msg)

        else:
            self.broadcast_replan_bid(self.curr_plan.cost - self.best_plan.cost)

        if self.at_goal():
            # no more replanning necessary!
            self.broadcast_replan_bid(-1000.0)


if __name__ == "__main__":
    env_file = rospy.get_param("/env_file")
    has_token = rospy.get_param("~has_token", False)

    start_pos = (0.0, 0.0)
    goal_pos = (10.0, 10.0)

    lunar_env = Environment()
    lunar_env.parse_yaml_data(yaml.safe_load(env_file))
    lunar_env.calculate_scene_dimensions()

    goal_dist = 0.1
    rrt_iters = 10

    rospy.init_node("agent", anonymous=True, log_level=rospy.DEBUG)

    # TODO: pass a callback to get the current time?
    agent = DMARRTAgent(
        start_pos=start_pos,
        goal_pos=goal_pos,
        environment=lunar_env,
        goal_dist=goal_dist,
        rrt_iters=rrt_iters,
    )

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        agent.spin_once()
        rate.sleep()
