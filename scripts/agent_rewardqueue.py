import rospy
from agent import Agent


class RewardQueueAgent(Agent):
    """
    An Agent that picks goals from a reward queue for distributed path planning
    """

    def __init__(self, *args, **kwargs):
        super(RewardQueueAgent, self).__init__(args, kwargs)

        self.identifier = rospy.get_name()

        self.peer_pub = rospy.Publisher("peers", PlanBid, queue_size=10)
        self.bid_pub = rospy.Publisher("plan_bids", PlanBid, queue_size=10)
        self.waypoints_pub = rospy.Publisher("waypoints", Waypoints, queue_size=10)

        # Register as listener for different kinds of messages
        rospy.Subscriber("registration", Registration, self.received_id)
        rospy.Subscriber("plan_bids", PlanBid, self.received_bid)
        rospy.Subscriber("waypoints", Waypoints, self.received_waypoints)

        # let the other agents know a new agent is on the network
        registration_pub = rospy.Publisher("registration", Registration)
        registration_pub.publish(rospy.get_name())

        # TODO: how should this be updated?
        self.curr_time = 0.0  # Simulation time. Updated externally

        # Keeps track of other agents' current bids for PPI
        #     (potential path improvement) at any given time:
        self.bids = dict()
        self.token_holder = False

        # TODO: new attributes needed for new pseudocode
        # (fix these and place them in positions that make sense)
        self.replan_token_holder = False
        self.goal_token_holder = False
        self.replan_bids = dict()
        self.goal_bids = dict()
        self.queue = []

    def received_id(self, msg):
        """
        TODO
        """
        if msg.sender_id != self.identifier:
            self.bids[msg.sender_id] = 0.0
            self.other_agent_plans[msg.sender_id] = Path()

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

    def broadcast_replan_bid(self, bid):
        """
        Broadcasts the following message to the TODO topic:

        msg - message of type PlanBid
        msg.sender_id - this agent's unique ID
        msg.bid - this agent's PPI, given by `bid`
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

    def broadcast_goal_bid(self, bid, bid_goal):
        """
        Broadcasts the following message to the TODO topic:

        msg - message of type TODO (GoalBid?)
        msg.sender_id - this agent's unique ID
        msg.bid - agent's bid (reward - cost to get to goal) for current favorite goal in queue, given by `bid`
        msg.goal_id - location of the goal this agent is bidding on, given by `bid_goal`
        """
        pass

    def spin_once(self):
        """
        Runs the agent's individual DMA-RRT component once.

        The interaction component is handled using Agent callbacks.
        """
        self.dma_individual()

        # Assign first "current path" found
        if not self.curr_plan.nodes:
            self.curr_plan = new_plan
        self.best_plan = new_plan

        if self.token_holder:
            # Replan to new best path
            self.curr_plan = self.best_plan

            # Solve collisions with time reallocation
            self.curr_plan = Plan.multiagent_aware_time_realloc(
                self.curr_plan, self.other_agent_plans
            )

            # Broadcast the new winner of the bidding round
            if self.bids:
                self.token_holder = False
                winner_bid = max(self.bids.values())
                winner_ids = [id for id, bid in self.bids.items() if bid == winner_bid]
                winner_id = random.choice(winner_ids)
                self.broadcast_waypoints(winner_id)
        else:
            self.broadcast_bid(self.curr_plan.cost - self.best_plan.cost)

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
            self.broadcast_bid(-1000.0)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        program="agent", description="Run an agent ROS node."
    )
    parser.add_argument("lunar_env", nargs=1)

    try:
        args = parser.parse_args()
        print args

        mode = "normal"
        start_pos = (0.0, 0.0)
        goal_pos = (10.0, 10.0)
        lunar_env = Environment()
        lunar_env.load_from_yaml_file(args.lunar_env)
        goal_dist = 0.1
        rrt_iters = 10

        rospy.init_node("agent", anonymous=True)
        agent = RewardQueueAgent(
            mode, start_pos, goal_pos, lunar_env, goal_dist, rrt_iters
        )

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            agent.spin_once()
            rate.sleep()
    except:
        parser.print_help()
