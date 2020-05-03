#!/usr/bin/env python
import random
import numpy as np
import yaml

from distributed_planning.msg import GoalBid, PlanBid, WinnerID
from agent import Agent
from rrtstar import RRTstar, Path

import rospy
import tf
import tf2_ros
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped, Transform, \
        TransformStamped, Point, PoseWithCovarianceStamped
from nav_msgs.msg import Path as PathRosMsg
from nav_msgs.msg import OccupancyGrid, MapMetaData
from visualization_msgs.msg import Marker


class DMARRTAgent(Agent):
    """
    An Agent that uses DMA-RRT for distributed path planning
    """

    def __init__(self):
        # Get All ROS params
        self.identifier = rospy.get_name()[1:]

        agent_params = rospy.get_param("/" + self.identifier)
        start_pos = (agent_params['start_pos']['x'], agent_params['start_pos']['y'])
        goal_pos = (agent_params['goal_pos']['x'], agent_params['goal_pos']['y'])
        goal_dist = agent_params['goal_dist']
        rrt_iters = agent_params['rrt_iters']
        step = agent_params['step_size']
        ccd = agent_params['near_radius']
        self.spin_rate = agent_params['spin_rate']

        # get map data from server
        map = rospy.wait_for_message("/map", OccupancyGrid)
        

        # Keeps track of other agents' plans so that we can
        #   add the other agents as obstacles when replanning:
        self.other_agent_plans = {}

        # Initial state and RRT planner
        self.start = start_pos
        self.goal = goal_pos
        self.pos = self.start

        self.rrt = RRTstar(
            start_pos, goal_pos, map, goal_dist,
            step_size=step, near_radius=ccd, max_iter=rrt_iters
        )
        
        # curr_plan is the currently executing plan; best_plan is a
        #    lower-cost path than curr_plan, if one exists.
        self.curr_plan = Path()
        self.best_plan = Path()

        self.plan_bids = {}
        self.peer_waypoints = {}
        self.plan_token_holder = rospy.get_param("~has_token", False)

        # TODO: how should this be updated?
        self.curr_time = 0.0  # Simulation time. Updated externally

        # be ready for replanning bids
        rospy.Subscriber("plan_bids", PlanBid, self.plan_bid_cb)
        self.plan_bid_pub = rospy.Publisher("plan_bids", PlanBid, queue_size=10)

        # be ready for updated waypoints from the winner
        self.waypoints_pub = rospy.Publisher(self.identifier + "/waypoints", PathRosMsg, queue_size=10)

        # be ready for winner ID messages
        rospy.Subscriber("winner_id", WinnerID, self.winner_id_cb)
        self.winner_id_pub = rospy.Publisher("winner_id", WinnerID, queue_size=10)

        # publish static tf for private map frame
        self.own_map_frame_id = "/" + self.identifier + "_map"
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        own_map_tf = TransformStamped()
        own_map_tf.header.stamp = rospy.Time.now()
        own_map_tf.header.frame_id = "/map"
        own_map_tf.child_frame_id = self.own_map_frame_id
        own_map_tf.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform([own_map_tf])

        # optionally publish RRT tree
        self.rrt_tree_pub = rospy.Publisher(self.identifier + "/rrt/tree", Marker, queue_size=10)
        self.tree_marker = self.setup_tree_marker()

        rospy.loginfo(self.identifier + " has initialized.")
        
        # Setup the spin
        rospy.Timer(rospy.Duration(1.0 / self.spin_rate), self.spin)

    def plan_bid_cb(self, msg):
        """
        Update internal state to reflect other agent's PPI bid.

        msg - message of type PlanBid
        msg.sender_id - unique ID of the agent who sent the message
        msg.bid - agent's PPI (potential path improvement) for current
        planning iteration
        """
        self.plan_bids[msg.header.frame_id] = msg.bid

    def winner_id_cb(self, msg):
        """
        """
        if msg.winner_id == self.identifier:
            self.plan_token_holder = True

    def waypoint_cb(self, msg):
        """
        """
        self.peer_waypoints[msg.header.frame_id] = msg.poses

    def spin(self, event):
        """
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

        If this agent holds the goal claiming token, it will claim
        its favorite goal and broadcast the next winner. Otw, it will
        broadcast its own new goal bid.
        """
        # TODO: do we need a timeout here instead of rrt_iters???
        new_plan = self.create_new_plan()
        self.publish_rrt_tree(self.rrt.node_list)

        # Assign first "current path" found
        if not self.curr_plan.nodes:
            self.curr_plan = new_plan
        self.best_plan = new_plan

        if self.plan_token_holder:
            # Replan to new best path
            self.curr_plan = self.best_plan

            # Solve collisions with time reallocation
            self.curr_plan = DMARRTAgent.multiagent_aware_time_realloc(
                self.curr_plan, self.other_agent_plans
            )

            # Broadcast the new winner of the bidding round
            if len(self.plan_bids) > 0:
                # select a winner based on bids
                winner_bid = max(self.plan_bids.values())
                winner_ids = [id for id, bid in self.plan_bids.items() if bid == winner_bid]
                winner_id = random.choice(winner_ids)  # break bid ties with randomness

                # broadcast new tokenholder
                self.plan_token_holder = False # Set to false here in case we get the token back.
                self.publish_winner_id(winner_id)

                # broadcast own waypoints
                self.publish_waypoints(self.best_plan)

        else:
            # broadcast plan bid
            self.publish_plan_bid(self.curr_plan.cost - self.best_plan.cost)

        if self.at_goal():
            # no more replanning necessary!
            self.publish_plan_bid(-1000.0)

    def publish_winner_id(self, winner_id):
        """
        """
        if not rospy.is_shutdown() and self.winner_id_pub.get_num_connections() > 0:
            winner_id_msg = WinnerID()
            winner_id_msg.header.stamp = rospy.Time.now()
            winner_id_msg.header.frame_id = self.identifier
            winner_id_msg.winner_id = winner_id
            
            self.winner_id_pub.publish(winner_id_msg)

    def publish_plan_bid(self, bid):
        """
        """
        if not rospy.is_shutdown() and self.plan_bid_pub.get_num_connections() > 0:
            bid_msg = PlanBid()
            bid_msg.header.stamp = rospy.Time.now()
            bid_msg.header.frame_id = self.identifier
            bid_msg.bid = bid

            self.plan_bid_pub.publish(bid_msg)

    def publish_waypoints(self, plan):
        """
        """
        if not rospy.is_shutdown() and self.waypoints_pub.get_num_connections() > 0:
            path_msg = PathRosMsg()
            path_msg.header.stamp = rospy.Time.now()
            path_msg.header.frame_id = self.own_map_frame_id
            path_msg.poses = [PoseStamped() for i in range(len(plan.nodes))]
            for i in range(len(plan.nodes)):
                pose = PoseStamped()
                node = plan.nodes[i]
                pose.header.frame_id = self.own_map_frame_id
                pose.header.stamp = rospy.Time(node.stamp)  # TODO(marcus): this needs to be a real timestamp
                pose.pose.position.x = node.x
                pose.pose.position.y = node.y
                pose.pose.position.z = 0.0  # TODO(marcus): extend to 3D
                pose.pose.orientation.w = 1.0  # TODO(marcus): include orientation info
                
                path_msg.poses[i] = pose

            self.waypoints_pub.publish(path_msg)

    def publish_rrt_tree(self, nodes):
        """
        """
        if not rospy.is_shutdown() and self.rrt_tree_pub.get_num_connections() > 0:
            self.tree_marker.points = []
            self.tree_marker.header.stamp = rospy.Time.now()

            for node in nodes:
                if node.parent:
                    self.tree_marker.points.append(Point(node.x, node.y, 0.0))
                    self.tree_marker.points.append(Point(node.parent.x, node.parent.y, 0.0))

            self.rrt_tree_pub.publish(self.tree_marker)

    def setup_tree_marker(self):
        """
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
        """
        dist = np.sqrt(
            (self.pos[0] - self.goal[0]) ** 2 + (self.pos[1] - self.goal[1]) ** 2
        )
        if dist <= 0.3:
            return True
        return False

    def update_time(self, curr_time):
        """
        Update the internal time counter.
        """
        self.curr_time = curr_time

    def create_new_plan(self):
        """
        Spin RRT to create a new plan

        Returns:
            Path
        """
        # Refresh environment to reflect agent's current positions
        # TODO(marcus): handled by tf tree!
        # self.rrt.update_pos(self.pos, self.curr_time, wipe_tree=True)

        # Grow the tree by one set of iterations
        self.rrt.spin(False)

        # Find the new best path in the tree
        return self.rrt.get_path()

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
        for i in range(1,len(path.nodes)):
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
                            for node in path.nodes[i+1:]:
                                revised_stamp += 1
                                node.stamp = revised_stamp

        path.ts_dict = {node.stamp : node for node in path.nodes}
        return path


if __name__ == "__main__":
    rospy.init_node("agent", anonymous=True)
    agent = DMARRTAgent()

    rospy.spin()