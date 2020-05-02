import argparse
import time
import numpy as np
import rospy

from rrtstar import Node, NodeStamped, RRTstar, Path


class Agent(object):
    def __init__(self, start_pos, goal_pos, environment, goal_dist, rrt_iters):
        """
        Initializer for Agent class; represents a robot capable of
        planning and moving in an environment. Use as a base for decentralized
        planning algorithms.

        start_pos - a 2-tuple representing the starting x-y position.
        goal_pos - a 2-tuple representing the goal x-y position.
        environment - an Environment object representing the map in
            which the agent must plan.
        goal_dist - a float representing the length of a single branch
            in the RRT tree for the agent's planner.
        rrt_iters - the number of iterations to run for RRT at each
            spin_once.
        """

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

    ####################################################################

    """
    TODO:
    - make sure these messages are actually getting sent at some point after the whole team has been initialized to populate all the agents' dictionaries
    - update to reflect new fields
    """

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
