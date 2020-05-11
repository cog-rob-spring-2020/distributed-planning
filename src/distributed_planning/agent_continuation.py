#!/usr/bin/env python
import random
import numpy as np
import yaml
import threading

import rospy
import message_filters
import tf
import tf2_ros
from std_msgs.msg import Header
from geometry_msgs.msg import (
    Pose,
    PoseStamped,
    Transform,
    TransformStamped,
    Point,
)
from nav_msgs.msg import Path as PathRosMsg
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker

from distributed_planning.msg import PlanBid, WinnerID, Queue
from rrtstar import RRTstar, Path, Node
from agent_dmarrt import DMARRTAgent


class ContinuationAgent(DMARRTAgent):
    """
    An Agent that uses DMA-RRT for distributed path planning and continues to the next goal in the queue.
    """

    def __init__(self, *args, **kwargs):
        super(ContinuationAgent, self).__init__(*args, **kwargs)
        self.spin_timer.shutdown()

        self.goal_count = 0

        # manage a FIFO goal queue
        self.queue = []

        # be ready for queue changes
        rospy.Subscriber("queue", Queue, self.queue_cb)
        self.queue_pub = rospy.Publisher("queue", Queue, queue_size=10)

        self.spin_timer = rospy.Timer(rospy.Duration(1.0 / self.spin_rate), self.spin)

    def queue_cb(self, msg):
        """
        Update the queue

        Params:
            msg distributed_planning.msg.Queue
        """
        self.queue = msg.goal_points

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

        # Assign first "current path" found
        if new_plan:
            self.best_plan = new_plan
            if not self.curr_plan:
                self.curr_plan = new_plan

        if self.plan_token_holder:
            if new_plan:
                # Replan to new best path
                self.curr_plan = self.best_plan

            # hand the token over to a new agent
            self.plan_bid_lock.acquire()
            winner_id = self.identifier
            if len(self.plan_bids) > 0:
                # select a winner based on bids
                winner_bid = max(self.plan_bids.values())
                winner_ids = [
                    id for id, bid in self.plan_bids.items() if bid == winner_bid
                ]

                winner_id = random.choice(winner_ids)  # break bid ties with randomness
                self.plan_bid_lock.release()

            if self.at_goal():
                if len(self.queue) > 0:
                    # we get to pick a new goal and update the queue
                    goal = self.take_goal()
                    self.reassign_goal(goal)

            self.plan_token_lock.acquire()
            self.plan_token_holder = False
            self.plan_token_lock.release()
            self.publish_winner_id(winner_id, curr_time)
            self.publish_path(self.curr_plan, curr_time, self.waypoint_pub)

        else:
            if self.at_goal():
                bid = 10000.0 if len(self.queue) > 0 else -1000
                self.publish_plan_bid(bid, curr_time)
            else:
                # compute a bid based on a comparison of the current and best plans
                if self.curr_plan and self.best_plan:
                    # broadcast plan bid
                    bid = np.abs(self.curr_plan.cost - self.best_plan.cost)
                    self.publish_plan_bid(bid, curr_time)

        self.visualize(curr_time)

    def take_goal(self):
        return self.queue.pop(0)

    def reassign_goal(self, new_goal):
        new_goal = (new_goal.x, new_goal.y)

        new_start = self.pos
        self.goal = new_goal
        self.start = new_start

        self.rrt = RRTstar(
            start=new_start,
            goal=new_goal,
            env=self.map_data,
            goal_dist=self.goal_dist,
            step_size=self.step,
            near_radius=self.ccd,
            max_iter=self.rrt_iters,
        )

        self.curr_plan = None
        self.best_plan = None
        self.curr_plan_id = 0
        msg = Queue(goal_points=self.queue)
        self.queue_pub.publish(msg)
        self.goal_count += 1

    def at_goal(self):
        if self.goal:
            return self.close_to_point(self.goal)

    def close_to_point(self, point):
        dist = np.sqrt((self.pos[0] - point[0]) ** 2 + (self.pos[1] - point[1]) ** 2)
        if dist <= 0.3:
            return True

        return False


if __name__ == "__main__":
    rospy.init_node("agent", anonymous=True)
    agent = ContinuationAgent()

    rospy.spin()
