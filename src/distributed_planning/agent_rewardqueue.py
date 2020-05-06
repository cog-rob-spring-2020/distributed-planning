#!/usr/bin/env python
import rospy
from agent_dmarrt import DMARRTAgent
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from distributed_planning.msg import GoalBid, WinnerIDGoal

class RewardQueueAgent(DMARRTAgent):
    """
    An Agent that picks goals from a reward queue for distributed path planning
    """

    def __init__(self, *args, **kwargs):
        # Keeps track of other agents' current goal bids at any given time
        self.goal_bids = {}
        self.goal_token_holder = False
        
        #list of goals organized in an order
        self.queue = [] #ordered by priority, each element is ((x,y), reward)

        #for path from goal to second goal
        self.second_plan = None
        self.second_goal = None

        #Adding goals
        rospy.Subscriber("add_goal", Point, self.added_goal_cb)

        #Removing goals
        rospy.Subscriber("remove_goal", Point, self.removed_goal_cb)

        #goal and winner
        rospy.Subscriber("winner_and_goal", WinnerIDGoal, self.winner_and_goal_cb)
        self.winner_and_goal_pub = rospy.Publisher("winner_and_goal", WinnerIDGoal, queue_size=10)

        #goal bids
        rospy.Subscriber("goal_bids", GoalBid, self.goal_bid_cb)
        self.goal_bid_pub = rospy.Publisher("goal_bids", GoalBid, queue_size=10)

        super(RewardQueueAgent, self).__init__(*args, **kwargs)

    ####################################################################

    def broadcast_winner_and_goal(self, winner_id, goal_id):
        """
        Broadcasts the following message to the WinnerIDGoal topic
        goal_id - (x, y)
        """
        if not rospy.is_shutdown() and self.winner_and_goal_pub.get_num_connections() > 0:
            msg = WinnerIDGoal()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.identifier
            msg.winner_id = winner_id
            
            goal = Point()
            goal.x = goal_id[0]
            goal.y = goal_id[1]

            msg.goal_id = goal

            self.winner_and_goal_pub.publish(msg)

    def broadcast_goal_bid(self, bid, goal_id):
        """
        Broadcasts the following message to the goal_bids topic:

        msg - message of type GoalBid
        msg.bid - agent's bid (reward - cost to get to goal) for current favorite goal in queue, given by `bid`
        msg.goal_id - location of the goal this agent is bidding on, given by `bid_goal`
        """
        if not rospy.is_shutdown() and self.winner_and_goal_pub.get_num_connections() > 0:
            msg = GoalBid()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.identifier
            msg.bid = bid

            goal = Point()
            goal.x = goal_id[0]
            goal.y = goal_id[1]

            msg.goal_id = goal

            self.goal_bid_pub.publish(msg)

    ####################################################################

    def winner_and_goal_cb(self, msg):
        """
        If winner, update internal state to hold the token.

        Remove the goal that the previous winner claimed from queue.

        msg - message of type WinnerIDGoal
        msg.winner_id - unique ID of the agent who has won the goal claiming token
        msg.goal_id - location of goal claimed by previous winner
        """
        if msg.winner_id == self.identifier:
            self.goal_token_holder = True
        goal = (msg.goal_id.x, msg.goal_id.y)
        self.queue_remove(goal)

    def goal_bid_cb(self, msg):
        """
        Update internal state to reflect other agent's bid
        on the first goal in the queue.
        msg - message of type GoalBid
        msg.sender_id - unique ID of the agent who sent the message
        msg.bid - agent's bid (reward - cost to get to goal) for current first goal in queue
        msg.goal_id - location of the goal other agent is bidding on
        """
        if msg.sender_id != self.identifier:
            self.goal_bids[msg.sender_id] = (msg.bid, (msg.goal_id.x, msg.goal_id.y))

    def added_goal_cb(self, msg):
        """
        Add the new goal to internal representation of goal queue,
        updating queue to remain sorted by reward.
        This message comes from the process that is adding/removing
        goals to the queue, modeling something like adaptive
        sampling (thus there is no sender ID because a single
        agent did not send the message in our implementation).
        msg - message of type Goal
        msg.goal_id - location of the goal to be added
        msg.reward - reward associated with that goal (e.g. from adaptive sampling)
        """
        self.queue_insert((msg.goal_id.x, msg.goal_id.y), msg.reward)

    def removed_goal_cb(self, msg):
        """
        Remove the specified goal from internal representation of goal
        queue, updating queue to remain sorted by reward.
        This message comes from the process that is adding/removing
        goals to the queue, modeling something like adaptive
        sampling (thus there is no sender ID because a single
        agent did not send the message in our implementation).
        msg - message of type Point
        msg.goal_id - location of the goal to be removed
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
        super(RewardQueueAgent, self).spin_once()

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