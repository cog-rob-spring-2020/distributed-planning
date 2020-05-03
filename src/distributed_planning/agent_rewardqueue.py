#!/usr/bin/env python
import random
import rospy
import yaml
from distributed_planning.msg import *
from agent_dmarrt import DMARRTAgent
from environment import Environment
from plan import Plan

class RewardQueueAgent(DMARRTAgent):
    """
    An Agent that picks goals from a reward queue for distributed path planning
    """

    def __init__(self, *args, **kwargs):
        super(RewardQueueAgent, self).__init__(args, kwargs)

        # Keeps track of other agents' current goal bids at any given time
        self.goal_bids = {}

        # whether or not this agent is holding the goal token
        self.goal_token_holder = False
        
        #list of goals organized in an order
        self.queue = []

        self.second_plan = None #for path from goal to second goal
        self.second_goal = None

        #Adding goals
        rospy.Subscriber("add_goal", Goal, self.received_add_goal)

        #Removing goals
        rospy.Subscriber("remove_goal", Goal, self.received_remove_goal)

        #goal and winner
        rospy.Subscriber("goal_and_winner", GoalWinner, self.received_remove_goal_and_goal_winner)
        self.goal_and_winner_pub = rospy.Publisher("goal_and_winner", GoalWinner, queue_size=10)

        #goal bids
        rospy.Subscriber("goal_bids", GoalBid, self.received_goal_bid)
        self.goal_bid_pub = rospy.Publisher("goal_bids", GoalBid, queue_size=10)

    ####################################################################

    def broadcast_remove_goal_and_goal_winner(self, winner_id, goal_id):
        """
        Broadcasts the following message to the TODO topic:

        msg - message of type TODO (GoalWinner?)
        msg.sender_id - this agent's unique ID
        msg.winner_id - unique ID of the agent who has won the goal
         claiming token, given by `winner_id`
        msg.goal_id - location of goal just claimed by this agent (so
         all others can remove from their queues)
        """
        msg = GoalWinner()
        msg.sender_id = self.identifier
        msg.winner_id = winner_id
        msg.goal_id = goal_id
        self.goal_and_winner_pub.publish(msg)

    def broadcast_goal_bid(self, bid, bid_goal):
        """
        Broadcasts the following message to the TODO topic:

        msg - message of type TODO (GoalBid?)
        msg.sender_id - this agent's unique ID
        msg.bid - agent's bid (reward - cost to get to goal) for current favorite goal in queue, given by `bid`
        msg.goal_id - location of the goal this agent is bidding on, given by `bid_goal`
        """
        msg = GoalBid()
        msg.sender_id = self.identifier
        msg.bid = bid
        msg.goal_id = bid_goal
        self.goal_bid_pub.publish(msg)

    ####################################################################

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

        self.queue_remove(msg.goal_id)

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

    ####################################################################

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

    def spin_once(self):
        """
        Runs the agent's individual DMA-RRT component once.

        The interaction component is handled using Agent callbacks.
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

        # Prevent agent from getting the token if they finish.
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
