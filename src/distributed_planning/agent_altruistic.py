#!/usr/bin/env python
import math
import numpy as np
import random
import yaml

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
    PoseWithCovarianceStamped,
)
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path as PathRosMsg
from visualization_msgs.msg import Marker

from rrtstar import RRTstar, Path
from distributed_planning.msg import (
    Costs,
    GoalCostPairing,
    PlanBid,
    WinnerID,
    AltruisticQueue,
)
from agent_dmarrt import DMARRTAgent


class Subagent(object):
    def __init__(
        self, map_data, start, goal, goal_dist, step, ccd, rrt_iters, movement_dt
    ):
        self.start = start

        self.rrt = RRTstar(
            start=start,
            goal=goal,
            env=map_data,
            goal_dist=goal_dist,
            step_size=step,
            near_radius=ccd,
            max_iter=rrt_iters,
        )

        # Initial state
        self.pos = self.start
        self.goal = goal
        self.plan_id = 0  # ID of the node in the current plan we are at
        self.latest_movement_timestamp = rospy.Time.now()
        self.movement_dt = movement_dt

        self.plan = None
        self.curr_agent_id = ""

    def spin_once(self, curr_time):
        dt = curr_time - self.latest_movement_timestamp
        if dt.to_sec() > self.movement_dt:
            self.update_pos(curr_time)
            self.latest_movement_timestamp = curr_time
        self.plan = self.create_new_plan()

    def create_new_plan(self):
        """
        Spin RRT to create a new plan

        Returns:
            Path
        """
        # Refresh environment to reflect agent's current positions
        self.rrt.update_pos(self.pos, 0, wipe_tree=True)

        # Grow the tree by one set of iterations
        if self.rrt.goal:
            self.rrt.spin(False)

            # Find the new best path in the tree
            return self.allocate_time_to_path(self.rrt.get_path(), rospy.Time.now())

        return None

    def update_pos(self, timestamp):
        if not rospy.is_shutdown():
            if self.plan:
                curr_node = None
                for i in range(self.plan_id + 1, len(self.plan.nodes)):
                    curr_node = self.plan.nodes[i]

                    if curr_node.stamp >= timestamp:
                        self.plan_id = i
                        break

                if curr_node:
                    self.pos = (curr_node.x, curr_node.y)

    def allocate_time_to_path(self, path, start_time):
        """
        Distribute time to nodes in path

        Returns:
            Path
        """
        if path.nodes:
            if self.plan:
                start_id = 0
                for i in range(len(path.nodes)):
                    node = path.nodes[i]
                    if self.plan_id < len(self.plan.nodes):
                        if node == self.plan.nodes[self.plan_id]:
                            start_id = i
                            break

                for i in range(len(path.nodes)):
                    if i < start_id:
                        path.nodes[i].stamp = rospy.Time.from_sec(0.0)
                    else:
                        path.nodes[i].stamp = start_time + rospy.Duration(
                            i * self.movement_dt
                        )
            else:
                for i in range(len(path.nodes)):
                    path.nodes[i].stamp = start_time + rospy.Duration(
                        i * self.movement_dt
                    )

        return path


class AltruisticAgent(DMARRTAgent):
    """
    An Agent that uses an altruistic cooperative DMA-RRT for distributed path planning
    """

    def __init__(self):
        # Get All ROS params
        self.identifier = rospy.get_name()[1:]
        self.body_frame = self.identifier + "_body"

        # Get map data from server
        self.map_topic = rospy.get_param("/map_topic")
        self.map_data = rospy.wait_for_message(self.map_topic, OccupancyGrid)

        self.agent_params = rospy.get_param("/" + self.identifier)
        self.spin_rate = self.agent_params["spin_rate"]

        # Initialize the RRT planner.
        self.start = (
            self.agent_params["start_pos"]["x"],
            self.agent_params["start_pos"]["y"],
        )
        self.goal = (
            self.agent_params["goal_pos"]["x"],
            self.agent_params["goal_pos"]["y"],
        )
        self.goal_dist = self.agent_params["goal_dist"]
        self.rrt_iters = self.agent_params["rrt_iters"]
        self.step = self.agent_params["step_size"]
        self.ccd = self.agent_params["near_radius"]

        # Initial state
        self.pos = self.start
        self.curr_plan_id = 0  # ID of the node in the current plan we are at
        self.latest_movement_timestamp = rospy.Time.now()
        self.movement_dt = self.agent_params["movement_dt"]

        # Send and track robot positions
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        # self.tf_listener = tf.TransformListener()

        # curr_plan is the currently executing plan; best_plan is a
        #    lower-cost path than curr_plan, if one exists.
        self.curr_plan = None
        self.best_plan = None

        # manage DMA-RRT bid and waypoint info
        self.plan_bids = {}
        self.peer_waypoints = {}
        self.plan_token_holder = rospy.get_param("~has_token", False)

        # be ready for replanning bids
        rospy.Subscriber("plan/bids", PlanBid, self.plan_bid_cb)
        self.plan_bid_pub = rospy.Publisher("plan/bids", PlanBid, queue_size=10)

        # publishers for replanning
        self.winner_id_pub = rospy.Publisher("plan/winner_id", WinnerID, queue_size=10)
        self.waypoint_pub = rospy.Publisher("plan/waypoints", PathRosMsg, queue_size=10)

        # Time-synchronized replan subscribers
        self.planning_ts = message_filters.TimeSynchronizer(
            [
                message_filters.Subscriber("plan/winner_id", WinnerID),
                message_filters.Subscriber("plan/waypoints", PathRosMsg),
            ],
            10,
        )
        self.planning_ts.registerCallback(self.winner_waypoint_cb)

        # publish static tf for private map frame
        self.own_map_frame_id = "/" + self.identifier + "_map"
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        own_map_tf = TransformStamped()
        own_map_tf.header.stamp = rospy.Time.now()
        own_map_tf.header.frame_id = self.map_topic
        own_map_tf.child_frame_id = self.own_map_frame_id
        own_map_tf.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform([own_map_tf])

        # Visualization publishers
        self.curr_path_viz_pub = rospy.Publisher(
            self.identifier + "/viz/curr_path", PathRosMsg, queue_size=10
        )
        self.best_path_viz_pub = rospy.Publisher(
            self.identifier + "/viz/best_path", PathRosMsg, queue_size=10
        )
        self.rrt_tree_pub = rospy.Publisher(
            self.identifier + "/viz/rrt/tree", Marker, queue_size=10
        )
        self.endpoint_marker_pub = rospy.Publisher(
            self.identifier + "/viz/rrt/endpoints", Marker, queue_size=10
        )

        self.tree_marker = self.setup_tree_marker()
        self.endpoint_marker = self.setup_endpoint_marker()

        rospy.loginfo(self.identifier + " has initialized.")
        # altruistic DMA-RRT goal management
        # for now, hardcode G, the number of goals to plan against
        self.G = 5
        self.queue = []
        # plans against different goals
        subagent_to_initial_goal = Subagent(
            self.map_data,
            self.start,
            self.goal,
            self.goal_dist,
            self.step,
            self.ccd,
            int(math.ceil(self.rrt_iters / self.G)),
            self.movement_dt,
        )
        subagent_to_initial_goal.curr_agent_id = self.identifier
        self.subagents = [subagent_to_initial_goal]
        # just for visualization
        self.rrt = subagent_to_initial_goal.rrt

        # be ready for queue changes
        rospy.Subscriber("queue", AltruisticQueue, self.queue_cb)
        self.queue_pub = rospy.Publisher("queue", AltruisticQueue, queue_size=10)

        # nested dict of costs to goals for each peer
        # self.peer_costs[peer][goal] = float
        self.peer_costs = {}

        # be ready for receiving costs to other goals
        rospy.Subscriber("costs", Costs, self.costs_cb)
        self.costs_pub = rospy.Publisher("costs", Costs, queue_size=10)

        # Setup the spin
        rospy.Timer(rospy.Duration(1.0 / self.spin_rate), self.spin)

    def active_subagent(self):
        subagents = [a for a in self.subagents if a.curr_agent_id == self.identifier]
        if len(subagents) == 1:
            return subagents[0]
        return None

    def queue_cb(self, msg):
        """
        Update the queue and our goal

        Params:
            msg distributed_planning.msg.AltruisticQueue
        """

        # [{location, agent_id}]
        self.queue = [{'agent_id': g.agent_id, 'location': (g.location.x, g.location.y) } for g in msg.goals]
        # TODO: make sure we always use tuples!!!

        my_goal = [g for g in self.queue if g.agent_id == self.identifier]

        if len(my_goal) > 0:
            self.goal = my_goal[0]

    def costs_cb(self, msg):
        """
        Update peer costs to other goals
        self.peer_costs[peer][Point] = cost

        Params:
            msg distributed_planning.msg.Costs
        """
        self.costs = {}
        sender_id = msg.header.frame_id
        # start with a fresh dict to ensure we remove any stale goals/costs
        self.peer_costs[sender_id] = {}
        self.peer_costs[sender_id] = {c.location: c.cost for c in msg.costs}

    def publish_costs(self):
        """
        Publish path costs to each goal to allow for altruistic goal-cost comparison
        """
        msg = Costs()
        msg.header.frame_id = self.identifier
        msg.costs = [
            GoalCostPairing(location=Point(a.goal[0], a.goal[1], 0.0), cost=a.plan.cost)
            for a in self.subagents
        ]
        self.costs_pub.publish(msg)

        # analysis: write a node that can read the queue, look at number of planning iterations a goal is in the queue, number of planning iterations, etc

    def select_other_goals(self):
        """
        Select other goals to plan against (creates the iterator of first for-loop in the individual component of the altruistic algorithm)
        """
        eucl = lambda g: np.linalg.norm(
            np.array((g.location.x, g.location.y)) - np.array(self.pos)
        )
        return filter(lambda g: (g.location.x, g.location.y) != self.goal, sorted(self.queue, key=eucl))[
            : self.G
        ]

    def update_own_goal_ownership(self, goal_to_take):
        """
        Make a copy of the queue with new owners for each goal

        Params:
            goal_to_release msg.AltruisticGoal
            goal_to_take msg.AltruisticGoal
        Returns:
            queue msg.AltruisticGoal[]
        """

        def mapper(g):
            if g == self.goal:
                g.agent_id = ""
            if g == goal_to_take:
                g.agent_id = self.identifier
            return g

        self.queue = list(map(self.queue, mapper))
        self.rrt = self.rrts[goal_to_take.location]
        self.curr_plan = self.curr_plans[goal_to_take.location]

    def update_their_goal_ownership(self, goal_to_give, recipient):
        """
        Make a copy of the queue and give a goal to another agent

        Params:
            goal_to_give msg.AltruisticGoal
            recipient string
        Returns:
            queue msg.AltruisticGoal[]
        """

        def mapper(g):
            if g == goal_to_give:
                g.agent_id = recipient
            return g

        self.queue = list(map(self.queue, mapper))

    def reroute_check(self, new_goal, new_goal_cost):
        """
        Run the reroute check against the peer currently holding new_goal. If we take new_goal, can the peer take ours or can they take an unclaimed goal to lower the global cost?

        Params:
            new_goal msg.AltruisticGoal
            new_goal_cost float agent's cost to this new goal
        Returns:
            new_goal
        """
        found_goal = None
        peer = new_goal.agent_id
        l = self.peer_costs[peer][new_goal.location]
        # ascending list of peer costs for all unclaimed goals and our goal
        possible_goals = sorted(
            filter(
                lambda g: self.queue[g].agent_id == ""
                or self.queue[g].agent_id == self.identifier,
                self.peer_costs[peer],
            ),
        )
        closest_possible_goal = possible_goals[0]

        m = self.peer_costs[peer][closest_possible_goal]
        a = self.curr_path.cost + l
        b = new_goal_cost + m
        if a > b:
            # the new pairings reduce the overall cost
            found_goal = g

        return found_goal

    def make_timestep(self):
        curr_time = rospy.Time.now()
        dt = curr_time - self.latest_movement_timestamp
        if dt.to_sec() > self.movement_dt:
            self.publish_new_tf(curr_time)
            self.latest_movement_timestamp = curr_time
        return curr_time

    def take_goal(self, location):
        for a in self.subagents:
            if a.goal == location:
                a.curr_agent_id = self.identifier
                break

        for g in self.queue:
            print(g.location, location)
            if (g.location.x, g.location.y) == location:
                g.agent_id = self.identifier
                break

        self.goal = location
        self.curr_plan_id = 0
        # TODO: clear curr_plan, best_plan instead?

    def release_goal(self):
        for a in self.subagents:
            if a.curr_agent_id == self.identifier:
                a.curr_agent_id = ""
                break

        for g in self.queue:
            if g.agent_id == self.identifier:
                g.agent_id = ""
                break

    def give_goal(self, location, agent_id):
        for g in self.queue:
            if g.location == location:
                g.agent_id = agent_id
                break

    def goal_by_location(self, location):
        # print(location, [g for g in self.queue])
        maybe_goal = [g for g in self.queue if (g.location.x, g.location.y) == location]
        if len(maybe_goal) > 0:
            return maybe_goal[0]
        return None

    def spin_once(self):
        """
        Individual component of altruistic goal selection.

        If this agent holds the replanning token, it will update its
        internal plan and broadcast the next winner. Otw, it will
        broadcast its own PPI bid.

        This agent will also create paths to other goals and look for opportunities to lower global cost by reassigning goals.
        """
        # Move the agent by one timestep.
        curr_time = self.make_timestep()

        active_subagent = self.active_subagent()
        if active_subagent:
            active_subagent.spin_once(curr_time)
            self.rrt = active_subagent.rrt
            # self.rrt.update_pos(self.pos, 0, wipe_tree=True)

            # Assign first "current path" found
            if not self.curr_plan:
                self.curr_plan = active_subagent.plan
            self.best_plan = active_subagent.plan

        # paths to each goal, i, at time k
        for goal in self.select_other_goals():
            subagent = Subagent(
                self.map_data,
                self.pos,
                (goal.location.x, goal.location.y),
                self.goal_dist,
                self.step,
                self.ccd,
                # split RRT iterations between subagents
                int(math.ceil(self.rrt_iters / self.G)),
                self.movement_dt,
            )
            subagent.curr_agent_id = goal.agent_id
            subagent.curr_plan_id = self.curr_plan_id
            subagent.spin_once(curr_time)
            self.subagents.append(subagent)

        # subagents sorted by ascending goal cost
        subagents = sorted(self.subagents, key=lambda a: a.plan.cost)

        if self.plan_token_holder:
            # print('start', self.identifier, self.goal)
            queue_msg = None
            if self.at_goal():
                # prune the subagent associated with the goal
                subagents = [
                    a for a in self.subagents if a.curr_agent_id != self.identifier
                ]

                # prune the goal from the queue
                self.queue = [g for g in self.queue if g.location != self.goal]
                queue_msg = AltruisticQueue(goals=self.queue)

            # print(self.identifier, 'looking at', self.queue)
            for a in subagents:
                goal = self.goal_by_location(a.goal)
                if not goal:
                    break

                if goal.agent_id == self.identifier:
                    # our current goal has the lowest cost. no goal modification necessary

                    # replan to new best path for same goal
                    self.curr_plan = self.best_plan
                    self.rrt = a.rrt
                    break
                elif goal.agent_id == "":
                    # print(self.identifier, "found a new goal")
                    # the goal is unclaimed and free to be taken
                    self.take_goal(a.goal)
                    self.curr_plan = self.best_plan = a.plan
                    self.rrt = a.rrt

                    # update the queue
                    queue_msg = AltruisticQueue(goals=self.queue)
                    break
                else:
                    # print(self.identifier, "found someone else's goal")
                    # someone else has this goal
                    found_peer_goal = self.reroute_check(goal, a.plan.cost)
                    if not found_peer_goal:
                        break

                    self.give_goal(found_peer_goal.location, goal.agent_id)

                    self.take_goal(a.goal)
                    self.curr_plan = self.best_plan = a.plan
                    self.rrt = a.rrt

                    # update the queue
                    queue_msg = AltruisticQueue(goals=self.queue)
                    break

            # print('end', self.identifier, self.goal)
            if queue_msg is not None:
                self.queue_pub.publish(queue_msg)

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
            self.publish_winner_id(winner_id, curr_time)
            self.publish_path(self.curr_plan, curr_time, self.waypoint_pub)

        else:
            if active_subagent:
                self.curr_plan = active_subagent.plan

            unclaimed_goals = filter(lambda g: g.agent_id == "", self.queue)
            # TODO: check how goal is being set! check .goal vs .location everywhere!
            if self.at_goal():
                bid = 10000.0 if len(unclaimed_goals) > 0 else -1000
                self.publish_plan_bid(bid, curr_time)
            else:
                # compute a bid based on a comparison of the current and best plans
                if self.curr_plan and self.best_plan:
                    # broadcast plan bid
                    bid = self.curr_plan.cost - self.best_plan.cost
                    self.publish_plan_bid(bid, curr_time)

            if not self.plan_token_holder:
                # compute a bid based on a comparison of the current plan and best plan to any goal
                bid = max(bid, self.curr_plan.cost - self.best_plan.cost)
                # broadcast plan bid
                self.publish_plan_bid(bid, curr_time)

        # broadcast costs to all goals planned against
        self.publish_costs()

        # prune all unused subagents
        self.subagents = [
            a for a in self.subagents if a.curr_agent_id == self.identifier
        ]

        self.visualize(curr_time)


if __name__ == "__main__":
    rospy.init_node("agent", anonymous=True)
    agent = AltruisticAgent()

    rospy.spin()
