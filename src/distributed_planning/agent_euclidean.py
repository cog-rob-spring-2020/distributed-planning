#!/usr/bin/env python
import numpy as np
import random
import rospy

from distributed_planning.msg import WinnerID, Queue
from agent_dmarrt import DMARRTAgent


class EuclideanAgent(DMARRTAgent):
    """
    An Agent that uses DMA-RRT for distributed path planning and continues to the next closest goal in the queue by Euclidean distance
    """

    def __init__(self, *args, **kwargs):
        # manage a FIFO goal queue
        self.queue = []

        # be ready for queue changes
        rospy.Subscriber("queue", Queue, self.queue_cb)
        self.queue_pub = rospy.Publisher("queue", Queue, queue_size=10)

        super(EuclideanAgent, self).__init__(*args, **kwargs)

    def spin(self, event):
        """
        The main loop for the Agent
        """
        while not rospy.is_shutdown():
            if rospy.get_param("/run_sim"):
                self.spin_once()

    def queue_cb(self, msg):
        """
        Update the queue

        Params:
            msg distributed_planning.msg.Queue
        """
        self.queue = msg.goals

        if len(self.queue) > 0:
                # pick the closest goal by Euclidean distance
                eucl = lambda g: np.linalg.norm(
                    np.array((g.x, g.y))
                    - np.array((self.rrt.curr_pos.x, self.rrt.curr_pos.y))
                )
                i = np.argmin(map(eucl, self.queue))
                self.goal = self.queue.pop(i)
                msg = Queue(goals=self.queue)
                self.queue_pub(msg)

    def spin_once(self):
        """
        An extended version of the individual component of DMA-RRT as described in algorithm 4
        from Desaraju/How 2012. If the agent is at a goal, it will bid high to get the replanning token to update the queue

        If this agent holds the replanning token, it will update its
        internal plan and broadcast the next winner. Otw, it will
        broadcast its own PPI bid.
        """
        # Move the agent by one timestep.
        curr_time = rospy.Time.now()
        dt = curr_time - self.latest_movement_timestamp
        if dt.to_sec() > self.movement_dt:
            self.publish_new_tf(curr_time)
            self.latest_movement_timestamp = curr_time

        # create a new plan to the goal
        new_plan = self.create_new_plan()
        self.publish_rrt_tree(self.rrt.node_list)

        # Assign first "current path" found
        if not self.curr_plan:
            self.curr_plan = new_plan
        self.best_plan = new_plan

        bid = None
        if self.at_goal() and len(self.queue) > 0:
            # we're at the goal and there are more goals on the queue
            # bid high to get the replanning token next round
            bid = max(bid, 10000.0)
        elif self.at_goal() and len(self.queue) == 0:
            # we're at the goal and there are no more goals on the queue
            # no more replanning necessary!
            bid = max(bid, -1000.0)

        if not self.plan_token_holder:
            # compute a bid based on a comparison of the current and best plans
            bid = max(bid, self.curr_plan.cost - self.best_plan.cost)
            # broadcast plan bid
            self.publish_plan_bid(bid)
            return

        # we know we have the token
        if self.at_goal():
            # we get to pick a new goal and update the queue (assuming goals are still on the queue)
            # we won't be doing any planning this round, but we will next round
            if len(self.queue) > 0:
                # pick the closest goal by Euclidean distance
                eucl = lambda g: np.linalg.norm(
                    np.array((g.x, g.y))
                    - np.array((self.rrt.curr_pos.x, self.rrt.curr_pos.y))
                )
                i = np.argmin(map(eucl, self.queue))
                self.goal = self.queue.pop(i)
                msg = Queue(goals=self.queue)
                self.queue_pub(msg)

            # hand the token over to a new agent
            if len(self.plan_bids) > 0:
                self.hand_over_token()
            self.plan_token_holder = False
            return

        # Replan to new best path
        self.curr_plan = self.best_plan

        # broadcast own waypoints
        self.publish_waypoints(self.best_plan)

        # Solve collisions with time reallocation
        # self.curr_plan = ContinuationAgent.multiagent_aware_time_reallocmultiagent_aware_time_realloc(
        #     self.curr_plan, self.other_agent_plans
        # )

        # hand the token over to a new agent
        if len(self.plan_bids) > 0:
            self.hand_over_token()
        self.plan_token_holder = False

    def hand_over_token(self):
        """
        Broadcast the token winner
        """
        if not rospy.is_shutdown() and self.winner_id_pub.get_num_connections() > 0:
            # select a winner based on bids
            winner_bid = max(self.plan_bids.values())
            winner_ids = [id for id, bid in self.plan_bids.items() if bid == winner_bid]
            winner_id = random.choice(winner_ids)  # break bid ties with randomness

            winner_id_msg = WinnerID()
            winner_id_msg.header.stamp = rospy.Time.now()
            winner_id_msg.header.frame_id = self.identifier
            winner_id_msg.winner_id = winner_id

            self.winner_id_pub.publish(winner_id_msg)


if __name__ == "__main__":
    rospy.init_node("agent", anonymous=True)
    agent = EuclideanAgent()

    rospy.spin()
