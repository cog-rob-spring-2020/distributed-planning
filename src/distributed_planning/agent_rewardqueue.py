#!/usr/bin/env python

import numpy as np
import threading

import rospy
import message_filters
from std_msgs.msg import Header
from geometry_msgs.msg import Point

from agent_dmarrt import DMARRTAgent
from distributed_planning.msg import Goal, GoalBid, WinnerID
from rrtstar import RRTstar, Path, Node


class RewardQueueAgent(DMARRTAgent):
    """
    An Agent that picks goals from a reward queue for distributed path planning
    """
    def __init__(self, *args, **kwargs):
        super(RewardQueueAgent, self).__init__(*args, **kwargs)
        self.spin_timer.shutdown()

        # Keeps track of other agents' current goal bids at any given time
        self.goal_bids = {}
        self.goal_token_holder = rospy.get_param("~has_goal_token", False)

        # list of goals organized in an order
        self.queue = [] # ordered by priority, each element is ((x,y), reward)

        # Locks for multithreading
        self.goal_bid_lock = threading.Lock()
        self.goal_token_lock = threading.Lock()
        self.queue_lock = threading.Lock()

        self.goal_list = [self.goal]
        self.goal_index = 0 # which goal is our current primary one
        self.goal_limit = 2 # number of future goals we can claim

        # Adding goals
        rospy.Subscriber("goals/add_goal", Goal, self.added_goal_cb)

        # Removing goals
        rospy.Subscriber("goals/remove_goal", Goal, self.removed_goal_cb)

        # goal and winner
        self.goal_ts = message_filters.TimeSynchronizer(
            [message_filters.Subscriber("goals/winner_id", WinnerID),
             message_filters.Subscriber("goals/winner_goal", Goal)], 10)
        self.goal_ts.registerCallback(self.winner_and_goal_cb)

        # synchronized to avoid asynchronous queue updates
        self.goal_winner_id_pub = rospy.Publisher("goals/winner_id", WinnerID, queue_size=10)
        self.goal_winner_point_pub = rospy.Publisher("goals/winner_goal", Goal, queue_size=10)

        # goal bids
        rospy.Subscriber("goals/goal_bids", GoalBid, self.goal_bid_cb)
        self.goal_bid_pub = rospy.Publisher("goals/goal_bids", GoalBid, queue_size=10)

        self.spin_timer = rospy.Timer(rospy.Duration(1.0 / self.spin_rate), self.spin)

    def spin_once(self):
        """
        Runs the agent's individual DMA-RRT component once.

        The interaction component is handled using Agent callbacks.
        """
        num_goals = len(self.goal_list) - self.goal_index

        # Handle goal modifications
        if self.goal_token_holder:
            removed_goal = None

            if num_goals < self.goal_limit and len(self.queue) > 0:
                self.queue_lock.acquire()
                removed_goal = self.queue[0][0]

                self.goal_list.append(removed_goal)
                self.queue_remove(removed_goal)
                self.queue_lock.release()

                if num_goals == 0:
                    # taking on a new primary goal
                    self.set_goal(removed_goal)

            # Send out the winner of the goal bids
            self.goal_bid_lock.acquire()
            if len(self.goal_bids.keys()) > 0:
                winner_id = list(self.goal_bids.keys())[0]
                winner_bid, winner_goal = self.goal_bids[winner_id]

                for agent_id in self.goal_bids:
                    agent_bid, agent_goal = self.goal_bids[agent_id]

                    if winner_goal == removed_goal and agent_goal != removed_goal:
                        # top priority is making sure token gets passed to someone who isn't trying to take the goal we just claimed
                        winner_id = agent_id
                        winner_bid, winner_goal = agent_bid, agent_goal
                    elif agent_bid > winner_bid:
                        winner_id = agent_id
                        winner_bid, winner_goal = agent_bid, agent_goal

                self.goal_token_lock.acquire()
                self.goal_token_holder = False
                self.goal_token_lock.release()

                stamp = rospy.Time.now()
                if removed_goal is None:
                    self.publish_goal_winner(winner_id, (0, 0), stamp, is_claimed = False)
                else:
                    self.publish_goal_winner(winner_id, removed_goal, stamp, is_claimed = True)
            self.goal_bid_lock.release()

        # Publish a goal bid
        else:
            if len(self.queue) > 0:
                # bid = reward - distance from last goal we will visit to the target goal
                self.queue_lock.acquire()
                queue_first_goal, reward = self.queue[0]
                self.queue_lock.release()
                last_goal = self.goal_list[-1]

                dist = np.abs(np.sqrt((last_goal[0] - queue_first_goal[0])**2 + (last_goal[1] - queue_first_goal[1])**2))

                bid = reward - dist

                # all goal slots are full so no need to claim goal
                # token unless everyone in this situation
                if num_goals == len(self.goal_list):
                    self.publish_goal_bid(-1000.0, queue_first_goal)

                # bid inversely proportional to number of goals we already have claimed
                else:
                    self.publish_goal_bid(bid / (num_goals + 1), queue_first_goal)

        # update goal progress
        if self.at_goal() and self.goal_index < len(self.goal_list) - 1:
            self.goal_index += 1
            self.set_goal(self.goal_list[self.goal_index])

        # once goals are up to date, replan or bid on replanning token
        super(RewardQueueAgent, self).spin_once()

    ####################################################################

    def set_goal(self, goal):
        """
        """
        self.goal = goal
        self.start = self.pos
        if goal:
            self.rrt = RRTstar(
                start=self.pos,
                goal=goal,
                env=self.map_data,
                goal_dist=self.goal_dist,
                step_size=self.step,
                near_radius=self.ccd,
                max_iter=self.rrt_iters,
            )
            self.curr_plan_id = 0
        else:
            self.rrt.goal = None

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
        Remove the goal from the queue attribute if it is in the queue.
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
        """
        if winner_id_msg.winner_id == self.identifier:
            self.goal_token_lock.acquire()
            self.goal_token_holder = True
            self.goal_token_lock.release()

        if goal_point_msg.is_claimed:
            goal = (goal_point_msg.goal_point.x, goal_point_msg.goal_point.y)
            self.queue_lock.acquire()
            self.queue_remove(goal)
            self.queue_lock.release()

    def goal_bid_cb(self, msg):
        """
        Update internal state to reflect other agent's bid
        on the first goal in the queue.
        msg - message of type GoalBid
        msg.header - contains the sender's ID as frame_id
        """
        if msg.header.frame_id != self.identifier:
            self.goal_bid_lock.acquire()
            self.goal_bids[msg.header.frame_id] = (msg.bid, (msg.goal_point.x, msg.goal_point.y))
            self.goal_bid_lock.release()

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
        self.queue_lock.acquire()
        self.queue_insert((msg.goal_point.x, msg.goal_point.y), msg.reward)
        self.queue_lock.release()

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
        self.queue_lock.acquire()
        self.queue_remove((msg.goal_point.x, msg.goal_point.y))
        self.queue_lock.release()

    ####################################################################

    def publish_goal_winner(self, winner_id, goal_point, stamp, is_claimed):
        """
        Broadcasts the following message to the Goal Winner topics.
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
            msg.is_claimed = is_claimed

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
