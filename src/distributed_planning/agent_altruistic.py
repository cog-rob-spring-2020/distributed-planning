#!/usr/bin/env python
import math
import numpy as np
import random
import yaml

import rospy
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
from visualization_msgs.msg import Marker

from rrtstar import RRTstar, Path
from distributed_planning.msg import PlanBid, WinnerID, Estop, EstopWaypoints


class AltruisticAgent(object):
    """
    An Agent that uses an altruistic cooperative DMA-RRT for distributed path planning
    """

    def __init__(self):
        # Get All ROS params
        self.identifier = rospy.get_name()[1:]
        self.body_frame = self.identifier + "_body"

        agent_params = rospy.get_param("/" + self.identifier)
        start_pos = (agent_params["start_pos"]["x"], agent_params["start_pos"]["y"])
        goal_pos = (agent_params["goal_pos"]["x"], agent_params["goal_pos"]["y"])
        self.goal_dist = agent_params["goal_dist"]
        self.rrt_iters = agent_params["rrt_iters"]
        self.step = agent_params["step_size"]
        self.ccd = agent_params["near_radius"]
        self.spin_rate = agent_params["spin_rate"]

        # get map data from server
        self.map_topic = rospy.get_param("/map_topic")
        self.map_data = rospy.wait_for_message(self.map_topic, OccupancyGrid)

        # Initial state
        self.start = start_pos
        self.goal = goal_pos
        self.pos = self.start
        self.curr_plan_id = 0  # ID of the node in the current plan we are at
        self.latest_movement_timestamp = rospy.Time.now()
        self.movement_dt = agent_params["movement_dt"]

        # set our RRT implementation against multiple goals
        self.rrts = {}

        # Send and track robot positions
        self.tf_broadcaster = tf.TransformBroadcaster()
        # self.tf_listener = tf.TransformListener()

        # curr_plan is the currently executing plan; best_plan is a
        #    lower-cost path than curr_plan, if one exists.
        self.curr_plan = None
        self.best_plan = None

        # manage DMA-RRT bid and waypoint info
        self.plan_bids = {}
        self.peer_waypoints = {}
        self.plan_token_holder = rospy.get_param("~has_token", False)

        # altruistic DMA-RRT goal management
        # for now, hardcode G, the number of goals to plan against
        self.G = 5
        self.queue = []

        # be ready for queue changes
        rospy.Subscriber("queue", AltruisticQueue, self.queue_cb)
        self.queue_pub = rospy.Publisher("queue", AltruisticQueue, queue_size=10)

        # nested dict of costs to goals for each peer
        # self.peer_costs[peer][goal] = float
        self.peer_costs = {}

        # be ready for receiving costs to other goals
        rospy.Subscriber("costs", Costs, self.costs_cb)
        self.costs_pub = rospy.Publisher("costs", Costs, queue_size=10)

        # be ready for replanning bids
        rospy.Subscriber("plan_bids", PlanBid, self.plan_bid_cb)
        self.plan_bid_pub = rospy.Publisher("plan_bids", PlanBid, queue_size=10)

        # be ready for updated waypoints from the winner
        self.waypoints_pub = rospy.Publisher(
            self.identifier + "/waypoints", EstopWaypoints, queue_size=10
        )

        # be ready for winner ID messages
        rospy.Subscriber("winner_id", WinnerID, self.winner_id_cb)
        self.winner_id_pub = rospy.Publisher("winner_id", WinnerID, queue_size=10)

        # be ready for emergency stops
        rospy.Subscriber("estop", Estop, self.estop_cb)
        self.estop_pub = rospy.Publisher("estop", Estop, queue_size=10)

        # publish static tf for private map frame
        self.own_map_frame_id = "/" + self.identifier + "_map"
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        own_map_tf = TransformStamped()
        own_map_tf.header.stamp = rospy.Time.now()
        own_map_tf.header.frame_id = self.map_topic
        own_map_tf.child_frame_id = self.own_map_frame_id
        own_map_tf.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform([own_map_tf])

        # optionally publish RRT tree
        self.rrt_tree_pub = rospy.Publisher(
            self.identifier + "/rrt/tree", Marker, queue_size=10
        )
        self.tree_marker = self.setup_tree_marker()

        rospy.loginfo(self.identifier + " has initialized.")

        # Setup the spin
        rospy.Timer(rospy.Duration(1.0 / self.spin_rate), self.spin)

    def spin(self, event):
        """
        The main loop for the Agent
        """
        while not rospy.is_shutdown():
            if rospy.get_param("/run_sim"):
                self.spin_once()

    def plan_bid_cb(self, msg):
        """
        Update internal state to reflect other agent's PPI bid.

        msg - message of type PlanBid
        msg.sender_id - unique ID of the agent who sent the message
        msg.bid - agent's PPI (potential path improvement) for current
        planning iteration
        """
        self.plan_bids[msg.header.frame_id] = msg.bid

    def queue_cb(self, msg):
        """
        Update the queue and our goal

        Params:
            msg distributed_planning.msg.AltruisticQueue
        """
        self.queue = msg.Altruisticgoals
        self.goal = filter(lambda g: g.agent_id == self.identifier, msg.Altruisticgoals)[0]

    def costs_cb(self, msg):
        """
        Update peer costs to other goals

        Params:
            msg distributed_planning.msg.Costs
        """
        self.costs = {}
        sender_id = msg.header.frame_id
        # start with a fresh dict to ensure we remove any stale goals/costs
        self.costs[sender_id] = {}
        self.costs[sender_id] = {c["goal"]: c["cost"] for c in msg.costs}

    def winner_id_cb(self, msg):
        """
        Update our token holder status

        Params:
            msg distributed_planning.msg.WinnerID
        """
        if msg.winner_id == self.identifier:
            self.plan_token_holder = True

    def waypoint_cb(self, msg):
        """
        Update waypoints as they're broadcasted
        """
        self.peer_waypoints[msg.header.frame_id] = msg.poses

    def select_other_goals(self):
        """
        Select other goals to plan against (creates the iterator of first for-loop in the individual component of the altruistic algorithm)
        """
        eucl = lambda g: np.linalg.norm(
            np.array((g.goal.x, g.goal.y))
            - np.array((self.rrt.curr_pos.x, self.rrt.curr_pos.y))
        )
        return filter(lambda g: g.goal != self.goal, sorted(self.queue, key=eucl))[
            : self.G
        ]

    def update_own_goal_ownership(self, goal_to_release, goal_to_take):
        """
        Make a copy of the queue with new owners for each goal

        Params:
            goal_to_release msg.AltruisticGoal
            goal_to_take msg.AltruisticGoal
        Returns:
            queue msg.AltruisticGoal[]
        """

        def mapper(g):
            if g == goal_to_release:
                g.agent_id = ""
            if g == goal_to_take:
                g.agent_id = self.identifier
            return g

        new_queue = map(self.queue, mapper)
        return list(new_queue)

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

        new_queue = map(self.queue, mapper)
        return list(new_queue)

    def reroute_check(self, new_goal, new_goal_cost):
        """
        Run the reroute check against the peer currently holding new_goal. If we take new_goal, can the peer take ours or can they take an unclaimed goal to lower the global cost?

        Params:
            new_goal msg.AltruisticGoal
            new_goal_cost float agent's cost to this new goal
        """
        found_goal = None
        peer = new_goal["agent_id"]
        l = self.costs[peer][new_goal["location"]]
        # ascending list of peer costs for all unclaimed goals and our goal
        possible_goals = sorted(
            filter(
                lambda g: self.queue[g]["agent_id"] == ""
                or self.queue[g]["agent_id"] == self.identifier,
                self.costs[peer],
            ),
        )
        closest_possible_goal = possible_goals[0]

        m = self.costs[peer][closest_possible_goal]
        a = self.curr_path.cost + l
        b = new_goal_cost + m
        if a > b:
            # the new pairings reduce the overall cost
            found_goal = g

        return found_goal

    def spin_once(self):
        """
        Individual component of altruistic goal selection.

        If this agent holds the replanning token, it will update its
        internal plan and broadcast the next winner. Otw, it will
        broadcast its own PPI bid.

        This agent will also create paths to other goals and look for opportunities to lower global cost by reassigning goals.
        """
        # Move the agent by one timestep.
        curr_time = rospy.Time.now()
        dt = curr_time - self.latest_movement_timestamp
        if dt.to_sec() > self.movement_dt:
            self.publish_new_tf(curr_time)
            self.latest_movement_timestamp = curr_time

        if not self.at_goal():
            new_plan = self.create_new_plan(self.goal)
            self.publish_rrt_tree(self.rrts[self.goal].node_list)

            # Assign first "current path" found
            if not self.curr_plan:
                self.curr_plan = new_plan
            self.best_plan = new_plan

        # paths to each goal, i, at time k
        p_iks = dict(
            [
                (goal, self.create_new_plan(goal.location))
                for goal in self.select_other_goals()
            ]
        )
        p_iks[self.goal] = self.best_plan

        # goals sorted by ascending cost
        goals = sorted([g for g in p_iks], key=lambda g: p_iks[g].cost)

        if self.plan_token_holder:
            for g in goals:
                if g.agent_id == self.identifier:
                    # our current goal has the lowest cost. no goal modification necessary

                    # replan to new best path for same goal
                    self.curr_plan = self.best_plan
                    break
                if g.agent_id == "":
                    # the goal is unclaimed and free to be taken
                    new_queue = self.update_goal_ownership(self.goal, g)

                    # update the queue
                    msg = Queue(goals=new_queue)
                    self.queue_pub(msg)

                    # take the path to our new goal
                    self.curr_plan = p_iks[g]
                    break
                else:
                    # someone else has this goal
                    found_peer_goal = self.reroute_check(g, p_iks[g].cost)
                    pass

            # Solve collisions with time reallocation
            # self.curr_plan = AltruisticAgent.multiagent_aware_time_reallocmultiagent_aware_time_realloc(
            #     self.curr_plan, self.other_agent_plans
            # )

            # Broadcast the new winner of the bidding round
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

        bid = None
        unclaimed_goals = filter(lambda g: g.agent_id == "", self.queue)
        if self.at_goal() and len(unclaimed_goals) > 0:
            # we're at the goal and there are unclaimed goals
            # bid high to get the replanning token next round
            bid = max(bid, 10000.0)
        elif self.at_goal() and len(unclaimed_goals) == 0:
            # we're at the goal and there are no unclaimed goals
            # no more replanning necessary!
            bid = max(bid, -1000.0)

        if not self.plan_token_holder:
            # compute a bid based on a comparison of the current plan and best plan to any goal
            bid = max(bid, self.curr_plan.cost - goals[0].cost)
            # broadcast plan bid
            self.publish_plan_bid(bid)
            return

        # broadcast costs to all goals planned against
        self.publish_costs(p_iks)

    def publish_winner_id(self, winner_id):
        """
        Broadcast the token winner
        """
        if not rospy.is_shutdown() and self.winner_id_pub.get_num_connections() > 0:
            winner_id_msg = WinnerID()
            winner_id_msg.header.stamp = rospy.Time.now()
            winner_id_msg.header.frame_id = self.identifier
            winner_id_msg.winner_id = winner_id

            self.winner_id_pub.publish(winner_id_msg)

    def publish_plan_bid(self, bid):
        """
        Broadcast our bid for the token
        """
        if not rospy.is_shutdown() and self.plan_bid_pub.get_num_connections() > 0:
            bid_msg = PlanBid()
            bid_msg.header.stamp = rospy.Time.now()
            bid_msg.header.frame_id = self.identifier
            bid_msg.bid = bid

            self.plan_bid_pub.publish(bid_msg)

    def publish_waypoints(self, plan):
        """
        Broadcast our waypoints
        """
        if not rospy.is_shutdown() and self.waypoints_pub.get_num_connections() > 0:
            path_msg = EstopWaypoints()
            path_msg.header.stamp = rospy.Time.now()
            path_msg.header.frame_id = self.own_map_frame_id
            path_msg.poses = [PoseStamped() for i in range(len(plan.nodes))]
            for i in range(len(plan.nodes)):
                pose = PoseStamped()
                node = plan.nodes[i]
                pose.header.frame_id = self.own_map_frame_id
                pose.header.stamp = node.stamp
                pose.pose.position.x = node.x
                pose.pose.position.y = node.y
                pose.pose.position.z = 0.0  # TODO(marcus): extend to 3D
                pose.pose.orientation.w = 1.0  # TODO(marcus): include orientation info

                path_msg.poses[i] = pose

            # mark evenly spaced poses along the path as estops
            path_msg.estops = []
            spacing = math.floor(len(plan.nodes) / (self.num_estops + 1))
            for n in range(1, self.num_estops + 1):
                estop = path_msg.poses[n * spacing]
                path_msg.estops.push(estop)

            self.waypoints_pub.publish(path_msg)

    def publish_costs(self, p_iks):
        """
        Publish path costs to each goal to allow for altruistic goal-cost comparison

        Params:
            p_iks dict paths by goal
        """
        msg = Costs()
        msg.header.frame_id = self.identifier
        msg.costs = [
            {"location": goal.location, "cost": plan.cost} for (goal, plan) in p_iks
        ]
        self.costs_pub.publish(msg)

    def publish_rrt_tree(self, nodes):
        """
        Broadcast our RRT tree
        """
        if not rospy.is_shutdown() and self.rrt_tree_pub.get_num_connections() > 0:
            self.tree_marker.points = []
            self.tree_marker.header.stamp = rospy.Time.now()

            for node in nodes:
                if node.parent:
                    self.tree_marker.points.append(Point(node.x, node.y, 0.0))
                    self.tree_marker.points.append(
                        Point(node.parent.x, node.parent.y, 0.0)
                    )

            self.rrt_tree_pub.publish(self.tree_marker)

    def publish_new_tf(self, timestamp):
        """ Publish the ground-truth transform to the TF tree.

            Args:
                timestamp: A rospy.Time instance representing the current
                    time in the simulator.
        """
        # Publish current transform to tf tree.
        if self.curr_plan:
            curr_node = None
            for i in range(self.curr_plan_id + 1, len(self.curr_plan.nodes)):
                curr_node = self.curr_plan.nodes[i]

                if curr_node.stamp >= timestamp:
                    self.curr_plan_id = i
                    break

            if curr_node:
                trans = [curr_node.x, curr_node.y, 0.0]
                quat = [0, 0, 0, 1]  # TODO(marcus): add this dimensionality
                self.tf_broadcaster.sendTransform(
                    trans, quat, timestamp, self.body_frame, self.own_map_frame_id
                )

                self.pos = (curr_node.x, curr_node.y)

    def setup_tree_marker(self):
        """
        Create a tree marker for visualization

        Returns:
            visualization_msgs.msg.Marker
        """
        tree_marker = Marker()
        tree_marker.points = []
        tree_marker.header.frame_id = self.own_map_frame_id
        tree_marker.action = Marker.ADD
        tree_marker.type = Marker.LINE_LIST
        tree_marker.pose.orientation.w = 1.0
        tree_marker.scale.x = 0.5
        tree_marker.color.r = 1.0
        tree_marker.color.a = 1.0

        return tree_marker

    def at_goal(self):
        """
        Checks if the agent's current location is the goal location.

        Returns:
            bool
        """
        dist = np.sqrt(
            (self.pos[0] - self.goal[0]) ** 2 + (self.pos[1] - self.goal[1]) ** 2
        )
        if dist <= 0.3:
            return True
        return False

    def create_new_plan(self, location):
        """
        Spin RRT to create a new plan

        Params:
            location Point

        Returns:
            Path
        """
        if goal.location not in self.rrts:
            self.rrts[goal.location] = RRTstar(
                self.pos,
                goal.location,
                self.map_data,
                self.goal_dist,
                step_size=self.step,
                near_radius=self.ccd,
                max_iter=self.rrt_iters,
            )
        # Refresh environment to reflect agent's current positions
        # TODO(marcus): handled by tf tree!
        self.rrts[goal.location].update_pos(self.pos, 0, wipe_tree=True)

        # Grow the tree by one set of iterations
        self.rrts[goal.location].spin(False)

        # Find the new best path in the tree
        return self.allocate_time_to_path(
            self.rrts[goal.location].get_path(), rospy.Time.now()
        )

    def allocate_time_to_path(self, path, start_time):
        """
        Distribute time to nodes in path

        Returns:
            Path
        """
        if path.nodes:
            if self.curr_plan:
                start_id = 0
                for i in range(len(path.nodes)):
                    node = path.nodes[i]
                    if self.curr_plan_id < len(self.curr_plan.nodes):
                        if node == self.curr_plan.nodes[self.curr_plan_id]:
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

    @staticmethod
    def multiagent_aware_time_realloc(path, other_agent_plans):
        """ Allocates more time to nodes in a path that conflict with other
            agent nodes, s.t. the conflict is gone.

            Only tokenholders should use this, to prevent duplicate conflict
            resolution.

            Args:
                path: a Path object.
                other_agent_plans: a dictionary keyed by Agent ID containing
                    the current Path objects of other agents in the scenario.
            Returns:
                a Path object similar to `path` but with collision-free
                    timestamps on each node.
        """
        for i in range(1, len(path.nodes)):
            # Check for collisions with any other node:
            for id, other_path in other_agent_plans.items():
                if other_path:
                    curr_stamp = path.nodes[i].stamp
                    if curr_stamp in other_path.ts_dict.keys():
                        if path.nodes[i] == other_path.ts_dict[curr_stamp]:
                            # Collision detected! Allocate more time to this node.

                            revised_stamp = curr_stamp
                            while path.nodes[i] == other_path.ts_dict[revised_stamp]:
                                revised_stamp += 1
                                path.nodes[i].stamp = revised_stamp

                            # Update all nodes after that one to have higher timetsamps.
                            for node in path.nodes[i + 1 :]:
                                revised_stamp += 1
                                node.stamp = revised_stamp

        path.ts_dict = {node.stamp: node for node in path.nodes}
        return path


if __name__ == "__main__":
    rospy.init_node("agent", anonymous=True)
    agent = AltruisticAgent()

    rospy.spin()
