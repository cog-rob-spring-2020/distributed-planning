#!/usr/bin/env python
import random
import numpy as np
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
)
from nav_msgs.msg import Path as PathRosMsg
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker

from distributed_planning.msg import PlanBid, WinnerID
from rrtstar import RRTstar, Path, Node


class DMARRTAgent(object):
    """`
    An Agent that uses DMA-RRT for distributed path planning
    """

    def __init__(self):
        # Get All ROS params
        self.identifier = rospy.get_name()[1:]
        self.body_frame = self.identifier + "_body"

        # Get map data from server
        self.map_topic = rospy.get_param("/map_topic")
        map_data = rospy.wait_for_message(self.map_topic, OccupancyGrid)

        self.agent_params = rospy.get_param("/" + self.identifier)
        self.spin_rate = self.agent_params["spin_rate"]
        
        # Initialize the RRT planner.
        start_pos = (self.agent_params["start_pos"]["x"], self.agent_params["start_pos"]["y"])
        goal_pos = (self.agent_params["goal_pos"]["x"], self.agent_params["goal_pos"]["y"])
        goal_dist = self.agent_params["goal_dist"]
        rrt_iters = self.agent_params["rrt_iters"]
        step = self.agent_params["step_size"]
        ccd = self.agent_params["near_radius"]

        # Setup our RRT implementation
        self.rrt = RRTstar(
            start_pos,
            goal_pos,
            map_data,
            goal_dist,
            step_size=step,
            near_radius=ccd,
            max_iter=rrt_iters,
        )

        # Initial state
        self.start = start_pos
        self.goal = goal_pos
        self.pos = self.start
        self.curr_plan_id = 0  # ID of the node in the current plan we are at
        self.latest_movement_timestamp = rospy.Time.now()
        self.movement_dt = self.agent_params["movement_dt"]

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

        # be ready for replanning bids
        rospy.Subscriber("plan/bids", PlanBid, self.plan_bid_cb)
        self.plan_bid_pub = rospy.Publisher("plan/bids", PlanBid, queue_size=10)

        # publishers for replanning
        self.winner_id_pub = rospy.Publisher("plan/winner_id", WinnerID, queue_size=10)
        self.waypoint_pub = rospy.Publisher("plan/waypoints", PathRosMsg, queue_size=10)

        # Time-synchronized replan subscribers
        self.planning_ts = message_filters.TimeSynchronizer(
            [message_filters.Subscriber("plan/winner_id", WinnerID),
             message_filters.Subscriber("plan/waypoints", PathRosMsg)], 10)
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
        self.tree_marker = self.setup_tree_marker()

        rospy.loginfo(self.identifier + " has initialized.")

        # Setup the spin
        rospy.Timer(rospy.Duration(1.0 / self.spin_rate), self.spin)

    ####################################################################

    def spin(self, event):
        """
        The main loop for the Agent
        """
        while not rospy.is_shutdown():
            if rospy.get_param("/run_sim"):
                self.spin_once()

    def spin_once(self):
        """
        Individual component of DMA-RRT as described in algorithm 4
        from Desaraju/How 2012.

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

        # TODO: do we need a timeout here instead of rrt_iters???
        new_plan = None
        if not self.at_goal():
            new_plan = self.create_new_plan()

            # Assign first "current path" found
            if new_plan:
                self.best_plan = new_plan
                if not self.curr_plan:
                    self.curr_plan = new_plan

        if self.plan_token_holder and new_plan:
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
            
            # These topics are time-synchronized for all agents
            self.publish_winner_id(winner_id, curr_time)
            self.publish_path(self.curr_plan, curr_time, self.waypoint_pub)

        if self.at_goal():
            # no more replanning necessary!
            self.publish_plan_bid(-1000.0, curr_time)
        else:
            if self.curr_plan and self.best_plan:
                # broadcast plan bid
                self.publish_plan_bid(self.curr_plan.cost - self.best_plan.cost, curr_time)

        # Visualize relevant data
        self.publish_rrt_tree(self.rrt.node_list)
        self.publish_path(self.curr_plan, curr_time, self.curr_path_viz_pub)
        self.publish_path(self.best_plan, curr_time, self.best_path_viz_pub)

    ####################################################################

    def plan_bid_cb(self, msg):
        """
        Update internal state to reflect other agent's PPI bid.

        msg - message of type PlanBid
        msg.sender_id - unique ID of the agent who sent the message
        msg.bid - agent's PPI (potential path improvement) for current
        planning iteration
        """
        self.plan_bids[msg.header.frame_id] = msg.bid

    def winner_waypoint_cb(self, winner_id_msg, waypoint_msg):
        """
        """
        self.winner_id_cb(winner_id_msg)
        self.waypoint_cb(waypoint_msg)

    def winner_id_cb(self, msg):
        """
        Update our token holder status
        """
        if msg.winner_id == self.identifier:
            self.plan_token_holder = True

    def waypoint_cb(self, msg):
        """
        Update waypoints as they're broadcasted
        """
        self.peer_waypoints[msg.header.frame_id] = msg.poses

    ####################################################################

    def publish_plan_bid(self, bid, stamp):
        """
        Broadcast our bid for the token
        """
        if not rospy.is_shutdown() and self.plan_bid_pub.get_num_connections() > 0:
            bid_msg = PlanBid()
            bid_msg.header.stamp = stamp
            bid_msg.header.frame_id = self.identifier
            bid_msg.bid = bid

            self.plan_bid_pub.publish(bid_msg)
            
    def publish_winner_id(self, winner_id, stamp):
        """
        Broadcast the token winner
        """
        if not rospy.is_shutdown() and self.winner_id_pub.get_num_connections() > 0:
            winner_id_msg = WinnerID()
            winner_id_msg.header.stamp = stamp
            winner_id_msg.header.frame_id = self.identifier
            winner_id_msg.winner_id = winner_id

            self.winner_id_pub.publish(winner_id_msg)

    def publish_path(self, plan, stamp, pub):
        """
        Broadcast our waypoints
        """
        if not rospy.is_shutdown() and pub.get_num_connections() > 0:
            path_msg = PathRosMsg()
            path_msg.header.stamp = stamp
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

            pub.publish(path_msg)

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
                self.pos = (curr_node.x, curr_node.y)

        trans = [self.pos[0], self.pos[1], 0.0]
        quat = [0, 0, 0, 1]  # TODO(marcus): add this dimensionality
        self.tf_broadcaster.sendTransform(
            trans, quat, timestamp, self.body_frame, self.own_map_frame_id
        )

    ####################################################################

    def set_goal(self, goal):
        """
        """
        self.goal = goal
        if goal:
            self.rrt.goal = Node(goal[0], goal[1])
        else:
            self.rrt.goal = None

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
        if self.goal:
            dist = np.sqrt(
                (self.pos[0] - self.goal[0]) ** 2 + (self.pos[1] - self.goal[1]) ** 2
            )
            if dist <= 0.3:
                return True
        return False

    def create_new_plan(self):
        """
        Spin RRT to create a new plan

        Returns:
            Path
        """
        # Refresh environment to reflect agent's current positions
        # TODO(marcus): handled by tf tree!
        self.rrt.update_pos(self.pos, 0, wipe_tree=True)

        # Grow the tree by one set of iterations
        if self.rrt.goal:
            self.rrt.spin(False)

            # Find the new best path in the tree
            return self.allocate_time_to_path(self.rrt.get_path(), rospy.Time.now())
        
        return None

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
    agent = DMARRTAgent()

    rospy.spin()
