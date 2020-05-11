#!/usr/bin/env python

import numpy as np

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

        self.goal_list = [self.goal, None]
        self.planner_list = [self.rrt, None]

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

    def create_new_plan(self):
        """
        Spin RRT to create a new plan.

        Use plans from multiple planners to concatenate path between goals.

        Returns:
            Path
        """
        # Refresh environment to reflect agent's current positions
        # TODO(marcus): handled by tf tree!
        self.planner_list[0].update_pos(self.pos, 0, wipe_tree=True)

        path = []
        for rrt in self.planner_list:
            if rrt is not None:
                rrt.spin(False)
                sub_path = self.allocate_time_to_path(rrt.get_path(), rospy.Time.now()).nodes

                path += list(sub_path)

            if rrt is None or not self.close_to_point((path[-1].x, path[-1].y), (rrt.goal.x, rrt.goal.y)):
                break # don't want non-contiguous paths

        return Path(nodes=path,
                    start_node=self.pos,
                    goal_node=path[-1])

    def at_goal(self):
        # no goals left to plan for
        return all(goal is None for goal in self.goal_list)

    def close_to_point(self, point, other):
        dist = np.sqrt((other[0] - point[0]) ** 2 + (other[1] - point[1]) ** 2)
        if dist <= 0.3:
            return True

        return False

    def at_first_goal(self):
        if self.goal_list[0] is not None:
            return self.close_to_point(self.goal_list[0], self.pos)
        return False

    def last_goal_index(self):
        # returns -1 if no goals, otherwise the index of the last non-None goal
        return len([goal for goal in self.goal_list if goal is not None]) - 1

    def spin_once(self):
        """
        Runs the agent's individual DMA-RRT component once.

        The interaction component is handled using Agent callbacks.
        """
        num_goals = self.last_goal_index() + 1

        # Handle goal modifications
        if self.goal_token_holder:
            removed_goal = None

            if num_goals != len(self.goal_list) and len(self.queue) > 0:
                next_goal_index = self.last_goal_index() + 1
                removed_goal = self.queue[0][0]

                self.goal_list[next_goal_index] = removed_goal
                self.queue_remove(removed_goal)

                if num_goals == 0:
                    self.set_goal(removed_goal) # obey superclass spec
                    start = self.pos
                else:
                    # second to last goal after update
                    start = self.goal_list[next_goal_index - 1]

                self.planner_list[next_goal_index] = RRTstar(start=start,
                        goal=removed_goal,
                        env=self.map_data,
                        goal_dist=self.goal_dist,
                        step_size=self.step,
                        near_radius=self.ccd,
                        max_iter=self.rrt_iters)

            # Send out the winner of the goal bids
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

                self.goal_token_holder = False

                stamp = rospy.Time.now()
                if removed_goal is None:
                    self.publish_goal_winner(winner_id, (0, 0), stamp, is_claimed = False)
                else:
                    self.publish_goal_winner(winner_id, removed_goal, stamp, is_claimed = True)

        # Publish a goal bid
        else:
            if len(self.queue) > 0:
                # bid = reward - distance from last goal we will visit to the target goal
                queue_first_goal, reward = self.queue[0]
                last_goal = self.goal_list[self.last_goal_index()]
                if last_goal is None:
                    last_goal = self.pos

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
        if self.at_first_goal():
            for i in range(len(self.goal_list) - 1):
                self.goal_list[i] = self.goal_list[i + 1]
                self.planner_list[i] = self.planner_list[i + 1]
            self.goal_list[-1] = None
            self.planner_list[-1] = None

        # once goals are up to date, replan or bid on replanning token
        super(RewardQueueAgent, self).spin_once()

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
            self.goal_token_holder = True

        if goal_point_msg.is_claimed:
            goal = (goal_point_msg.goal_point.x, goal_point_msg.goal_point.y)
            self.queue_remove(goal)

    def goal_bid_cb(self, msg):
        """
        Update internal state to reflect other agent's bid
        on the first goal in the queue.
        msg - message of type GoalBid
        msg.header - contains the sender's ID as frame_id
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
