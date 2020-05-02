#!/usr/bin/env python
import random
import rospy
from distributed_planning.msg import *
from agent import Agent


class DMARRTAgent(Agent):
    """
    An Agent that uses DMA-RRT for distributed path planning
    """

    def __init__(self, *args, **kwargs):
        super(DMARRTAgent, self).__init__(args, kwargs)

        # a unique name given to us by ROS
        self.identifier = rospy.get_name()

        # be ready for replanning bids
        rospy.Subscriber("plan_bids", PlanBid, self.received_plan_bid)
        self.plan_bid_pub = rospy.Publisher("plan_bids", PlanBid, queue_size=10)

        # be ready for updated waypoints from the winner
        rospy.Subscriber(
            "waypoints", Waypoints, self.received_waypoints_and_replan_winner
        )
        self.waypoints_pub = rospy.Publisher("waypoints", Waypoints, queue_size=10)

        # let the other agents know a new agent is on the network
        rospy.Subscriber("registration", Registration, self.received_registration)
        registration_pub = rospy.Publisher("registration", Registration)
        registration_pub.publish(rospy.get_name())

        # Keeps track of other agents' current bids for PPI (potential path improvement) at any given time
        self.plan_bids = dict()
        # paths by peer ID. all peers should be represented once here, and as such calling `len(self.peer_waypoints)` should give an accurate count of peers
        self.peer_waypoints = dict()
        # whether or not this agent is holding the replan token
        self.plan_token_holder = False

        # TODO: how should this be updated?
        self.curr_time = 0.0  # Simulation time. Updated externally

    def received_registration(self, msg):
        """
        We met a peer, let's note that we're tracking their path
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

        self.peer_waypoints[msg.sender_id] = msg.waypoints

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
                self.curr_plan, self.other_agent_plans
            )

            # Broadcast the new winner of the bidding round
            if len(self.plan_bids) > 0:
                # select a winner based on bids
                winner_bid = max(self.bids.values())
                winner_ids = [id for id, bid in self.bids.items() if bid == winner_bid]
                # break bid ties with randomness
                winner_id = random.choice(winner_ids)

                # broadcast own waypoints and new token holder
                msg = Waypoints()
                msg.sender_id = winner_id
                msg.waypoints = self.best_plan
                self.waypoints_pub.publish(msg)

                self.plan_token_holder = False
        else:
            self.broadcast_replan_bid(self.curr_plan.cost - self.best_plan.cost)

        if self.at_goal():
            # no more replanning necessary!
            self.broadcast_bid(-1000.0)


if __name__ == "__main__":
    lunar_env = rospy.get_param("~lunar_env")
    has_token = rospy.get_param("~has_token")

    print has_token
    print lunar_env

    mode = "normal"
    start_pos = (0.0, 0.0)
    goal_pos = (10.0, 10.0)
    lunar_env = Environment()
    lunar_env.load_from_yaml_file(args.lunar_env)
    goal_dist = 0.1
    rrt_iters = 10

    rospy.init_node("agent", anonymous=True)
    # TODO: pass a callback to get the current time?
    agent = DMARRTAgent(mode, start_pos, goal_pos, lunar_env, goal_dist, rrt_iters)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        agent.spin_once()
        rate.sleep()
