#!/usr/bin/env python

import numpy as np

import rospy
import message_filters
from std_msgs.msg import Header
from geometry_msgs.msg import Point

from agent_dmarrt import DMARRTAgent
from distributed_planning.msg import Goal, GoalBid, WinnerID


class RewardQueueAgent(DMARRTAgent):
    """
    An Agent that picks goals from a reward queue for distributed path planning
    """

    def __init__(self, *args, **kwargs):
        # Keeps track of other agents' current goal bids at any given time
        self.goal_bids = {}
        self.goal_token_holder = rospy.get_param("~has_goal_token", False)

        # list of goals organized in an order
        self.queue = [] # ordered by priority, each element is ((x,y), reward)

        # for path from goal to second goal
        self.second_plan = None
        self.second_goal = None

        # Adding goals
        rospy.Subscriber("goals/add_goal", Goal, self.added_goal_cb)

        # Removing goals
        rospy.Subscriber("goals/remove_goal", Goal, self.removed_goal_cb)

        # goal and winner
        self.goal_ts = message_filters.TimeSynchronizer(
            [message_filters.Subscriber("goals/winner_id", WinnerID),
             message_filters.Subscriber("goals/winner_goal", Goal)], 10)
        self.goal_ts.registerCallback(self.winner_and_goal_cb)

        self.goal_winner_id_pub = rospy.Publisher("goals/winner_id", WinnerID, queue_size=10)
        self.goal_winner_point_pub = rospy.Publisher("goals/winner_goal", Goal, queue_size=10)

        # goal bids
        rospy.Subscriber("goals/goal_bids", GoalBid, self.goal_bid_cb)
        self.goal_bid_pub = rospy.Publisher("goals/goal_bids", GoalBid, queue_size=10)

        super(RewardQueueAgent, self).__init__(*args, **kwargs)

        # Wipe the RRT planner because we don't have a goal pose yet.
        self.set_goal(None)
        self.goal = None

    def spin_once(self):
        """
        Runs the agent's individual DMA-RRT component once.

        The interaction component is handled using Agent callbacks.
        """

        """
        TODO:
        rework RRT planning to sequentially plan from current position
        to first goal, then from first goal to second goal.
        curr_plan and best_plan should both go through both goals!
        This can be done inside of the DMARRTAgent class!
        """
        super(RewardQueueAgent, self).spin_once()

        # Handle goal modifications
        if self.goal_token_holder:
            removed_goal = None

            # Claim a primary goal in the queue
            if self.goal == None and len(self.queue) > 0:
                self.goal = self.queue[0][0]
                self.queue_remove(self.goal)
                removed_goal = self.goal

                # Start up RRT
                self.set_goal(self.goal)

            # Claim a secondary goal in the queue
            elif self.second_goal == None and len(self.queue) > 0:
                self.second_goal = self.queue[0][0]
                self.queue_remove(self.second_goal)
                removed_goal = self.second_goal
                
            # Send out the winner of the goal bids
            if len(self.goal_bids.keys()) > 0 and removed_goal:
                max_agent = list(self.goal_bids.keys())[0] #start with random agent
                for agent in self.goal_bids:
                    if self.goal_bids[agent][0] > self.goal_bids[max_agent][0] and self.goal_bids[agent][1] != removed_goal:
                        max_agent = agent

                stamp = rospy.Time.now()
                self.publish_goal_winner(max_agent, removed_goal, stamp)
                self.goal_token_holder = False

        # Publish a goal bid
        else:
            if len(self.queue) > 0:
                #bid = reward - distance to reward
                first_goal = self.queue[0][0]
                dist = np.abs(np.sqrt((self.pos[0] - first_goal[0])**2 + (self.pos[1] - first_goal[1])**2))
                bid = self.queue[0][1]-dist

                #no need to get the token
                if self.goal and self.second_goal:
                    self.publish_goal_bid(-1000, self.goal)

                #reduced bid for secondary goal since it's not urgent
                elif self.goal:
                    self.publish_goal_bid(bid/2, first_goal)

                #bid normally
                else:
                    self.publish_goal_bid(bid, first_goal)

    ####################################################################

    def queue_insert(self, goal, reward):
        """
        Add the goal to the queue attribute, keeping the queue
        sorted by reward
        goal - (x, y)
        reward - float
        """
        prior_reward = reward
        for i in range(len(self.queue)):
            if self.queue[i][1] <= reward and reward <= prior_reward:
                self.queue = self.queue[:i] + [(goal, reward)] + self.queue[i:]
                return
            prior_reward = self.queue[i][1] 
        self.queue.append((goal, reward))

    def queue_remove(self, goal):
        """
        Remove the goal from the queue attribute.
        goal - (x, y)
        reward - float
        """
        index = -1
        for i in range(len(self.queue)):
            if self.queue[i][0] == goal:
                index = i
                break
        if index >= 0:
            self.queue.pop(index)

    ####################################################################

    def winner_and_goal_cb(self, winner_id_msg, goal_point_msg):
        """
        If winner, update internal state to hold the token.

        Remove the goal that the previous winner claimed from queue.

        msg - message of type WinnerIDGoal
        """
        if winner_id_msg.winner_id == self.identifier:
            self.goal_token_holder = True
        goal = (goal_point_msg.goal_point.x, goal_point_msg.goal_point.y)
        self.queue_remove(goal)

    def goal_bid_cb(self, msg):
        """
        Update internal state to reflect other agent's bid
        on the first goal in the queue.
        msg - message of type GoalBid
        """
        if msg.header.frame_id != self.identifier:
            self.goal_bids[msg.header.frame_id] = (msg.bid, (msg.goal_point.x, msg.goal_point.y))

    def added_goal_cb(self, msg):
        """
        Add the new goal to internal representation of goal queue,
        updating queue to remain sorted by reward.
        This message comes from the process that is adding/removing
        goals to the queue, modeling something like adaptive
        sampling (thus there is no sender ID because a single
        agent did not send the message in our implementation).
        msg - message of type Goal
        """
        self.queue_insert((msg.goal_point.x, msg.goal_point.y), msg.reward)

    def removed_goal_cb(self, msg):
        """
        Remove the specified goal from internal representation of goal
        queue, updating queue to remain sorted by reward.
        This message comes from the process that is adding/removing
        goals to the queue, modeling something like adaptive
        sampling (thus there is no sender ID because a single
        agent did not send the message in our implementation).
        msg - message of type Goal
        """
        self.queue_remove((msg.goal_point.x, msg.goal_point.y))

    ####################################################################

    def publish_goal_winner(self, winner_id, goal_point, stamp):
        """
        Broadcasts the following message to the WinnerIDGoal topic
        goal_point - (x, y)
        other agents remove the specified goal
        """
        if not rospy.is_shutdown() and \
                self.goal_winner_point_pub.get_num_connections() > 0 and \
                self.goal_winner_id_pub.get_num_connections() > 0:
            msg = Goal()
            msg.header.stamp = stamp
            msg.header.frame_id = self.identifier
            msg.reward = 0.0  # Not used in subscriber callbacks.
            msg.goal_point = Point(goal_point[0], goal_point[1], 0.0)

            self.goal_winner_point_pub.publish(msg)

            msg = WinnerID()
            msg.header.stamp = stamp
            msg.header.frame_id = self.identifier
            msg.winner_id = winner_id

            self.goal_winner_id_pub.publish(msg)

    def publish_goal_bid(self, bid, goal_point):
        """
        Broadcasts the following message to the goal_bids topic:
        """
        if not rospy.is_shutdown() and self.goal_bid_pub.get_num_connections() > 0:
            msg = GoalBid()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.identifier
            msg.bid = bid

            goal = Point()
            goal.x = goal_point[0]
            goal.y = goal_point[1]

            msg.goal_point = goal

            self.goal_bid_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("agent", anonymous=True)
    agent = RewardQueueAgent()

    rospy.spin()
