#!/usr/bin/env python
import rospy
from math import sqrt
from agent_dmarrt import DMARRTAgent
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from distributed_planning.msg import Goal, GoalBid, WinnerIDGoal


class RewardQueueAgent(DMARRTAgent):
    """
    An Agent that picks goals from a reward queue for distributed path planning
    """

    def __init__(self, *args, **kwargs):
        # Keeps track of other agents' current goal bids at any given time
        self.goal_bids = {}
        self.goal_token_holder = False

        # list of goals organized in an order
        self.queue = [] # ordered by priority, each element is ((x,y), reward)

        # for path from goal to second goal
        self.second_plan = None
        self.second_goal = None

        # Adding goals
        rospy.Subscriber("add_goal", Goal, self.added_goal_cb)

        # Removing goals
        rospy.Subscriber("remove_goal", Goal, self.removed_goal_cb)

        # goal and winner
        rospy.Subscriber("winner_and_goal", WinnerIDGoal, self.winner_and_goal_cb)
        self.winner_and_goal_pub = rospy.Publisher("winner_and_goal", WinnerIDGoal, queue_size=10)

        # goal bids
        rospy.Subscriber("goal_bids", GoalBid, self.goal_bid_cb)
        self.goal_bid_pub = rospy.Publisher("goal_bids", GoalBid, queue_size=10)

        super(RewardQueueAgent, self).__init__(*args, **kwargs)

    ####################################################################

    def broadcast_winner_and_goal(self, winner_id, goal_point):
        """
        Broadcasts the following message to the WinnerIDGoal topic
        goal_point - (x, y)
        other agents remove the specified goal
        """
        if not rospy.is_shutdown() and self.winner_and_goal_pub.get_num_connections() > 0:
            msg = WinnerIDGoal()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.identifier
            msg.winner_id = winner_id
            
            goal = Point()
            goal.x = goal_point[0]
            goal.y = goal_point[1]

            msg.goal_point = goal

            self.winner_and_goal_pub.publish(msg)

    def broadcast_goal_bid(self, bid, goal_point):
        """
        Broadcasts the following message to the goal_bids topic:

        msg - message of type GoalBid
        msg.bid - agent's bid (reward - cost to get to goal) for current favorite goal in queue, given by `bid`
        msg.goal_point - location of the goal this agent is bidding on, given by `bid_goal`
        """
        if not rospy.is_shutdown() and self.winner_and_goal_pub.get_num_connections() > 0:
            msg = GoalBid()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.identifier
            msg.bid = bid

            goal = Point()
            goal.x = goal_point[0]
            goal.y = goal_point[1]

            msg.goal_point = goal

            self.goal_bid_pub.publish(msg)

    ####################################################################

    def winner_and_goal_cb(self, msg):
        """
        If winner, update internal state to hold the token.

        Remove the goal that the previous winner claimed from queue.

        msg - message of type WinnerIDGoal
        """
        if msg.winner_id == self.identifier:
            self.goal_token_holder = True
        goal = (msg.goal_point.x, msg.goal_point.y)
        self.queue_remove(goal)

    def goal_bid_cb(self, msg):
        """
        Update internal state to reflect other agent's bid
        on the first goal in the queue.
        msg - message of type GoalBid
        msg.sender_id - unique ID of the agent who sent the message
        """
        if msg.sender_id != self.identifier:
            self.goal_bids[msg.sender_id] = (msg.bid, (msg.goal_point.x, msg.goal_point.y))

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
        self.queue_remove((msg.x, msg.y))

    ####################################################################

    def queue_insert(self, goal, reward):
        """
        Add the goal to the queue attribute, keeping the queue
        sorted by reward
        goal - (x, y)
        reward - float
        """
        prior_reward = reward
        for i in range(self.queue):
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
        for i in range(self.queue):
            if self.queue[i][0] == goal:
                index = i
                break
        if index >= 0:
            self.queue.remove(index)

    ####################################################################

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
        """
        # TODO:
        # if-else for other token (for getting goals off queue)
        # if self.goal_token_holder and you have space for a new goal:
        #   take first goal off queue
        #   compute winner of next round of goal bidding (ignoring bidders for goal we just claimed)
        #   broadcast winner (tells everyone to remove goal we claimed)
        # else:
        #   bid on our favorite goal in the queue
                # Move the agent by one timestep.

        curr_time = rospy.Time.now()
        dt = curr_time - self.latest_movement_timestamp
        if dt.to_sec() > self.movement_dt:
            self.publish_new_tf(curr_time)
            self.latest_movement_timestamp = curr_time

        #handle goal situations
        if self.goal_token_holder:
            removed_goal = (None, None) #placeholder to avoid breaking queue_remove

            if self.goal == None and len(self.queue) > 0:
                self.goal = self.queue[0]
                self.queue_remove(self.goal)
                removed_goal = self.goal
            elif self.second_goal == None and len(self.queue) > 0:
                self.second_goal = self.queue[0]
                self.queue_remove(self.second_goal)
                removed_goal = self.second_goal
                
            max_agent = list(self.goal_bids.keys())[0]
            for agent in self.goal_bids:
                if self.goal_bids[agent][0] > self.goal_bids[max_agent][0] and self.goal_bids[agent][1] != removed_goal:
                    max_agent = agent

            self.broadcast_winner_and_goal(max_agent, removed_goal)

        else:
            bid = abs(sqrt((self.pos[0]-self.queue[0])**2 + (self.pos[1]-self.queue[1])**2))
            self.broadcast_goal_bid(bid, self.queue[0]) 

        # TODO: do we need a timeout here instead of rrt_iters???
        if not self.at_goal():
            new_plan = self.create_new_plan()
            self.publish_rrt_tree(self.rrt.node_list)

            # Assign first "current path" found
            if not self.curr_plan:
                self.curr_plan = new_plan
            self.best_plan = new_plan

            # TODO: plan from first goal to second goal


        if self.plan_token_holder:
            # Replan to new best path
            self.curr_plan = self.best_plan

            # Solve collisions with time reallocation
            # self.curr_plan = DMARRTAgent.multiagent_aware_time_reallocmultiagent_aware_time_realloc(
            #     self.curr_plan, self.other_agent_plans
            # )

            # Broadcast the new winner of the bidding round
            winner_id = self.identifier
            if len(self.plan_bids) > 0:
                # select a winner based on bids
                winner_bid = max(self.plan_bids.values())
                winner_ids = [
                    id for id, bid in self.plan_bids.items() if bid == winner_bid
                ]
                winner_id = random.choice(winner_ids)  # break bid ties with randomness

            # broadcast new tokenholder
            self.plan_token_holder = (
                False  # Set to false here in case we get the token back.
            )
            self.publish_winner_id(winner_id)

            # broadcast own waypoints
            self.publish_waypoints(self.best_plan)

            #need to change to format below
            # self.publish_winner_id_and_waypoints(winner_id, self.best_plan)

        # broadcast plan bid
        if self.at_goal():
            # no more replanning necessary!
            self.publish_plan_bid(-1000.0)
        else:
            self.publish_plan_bid(self.curr_plan.cost - self.best_plan.cost)

    def spin(self, event):
        """
        The main loop for the Agent
        """
        while not rospy.is_shutdown():
            if rospy.get_param("/run_sim"):
                self.spin_once()


if __name__ == "__main__":
    rospy.init_node("agent", anonymous=True)
    agent = RewardQueueAgent()

    rospy.spin()