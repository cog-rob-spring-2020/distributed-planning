#!/usr/bin/env python
import rospy
import random
from time import sleep
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from distributed_planning.msg import AltruisticGoal, WinnerIDGoal


class GoalUpdater(object):
    def __init__(self):
        # init variables
        self.goals = []  # elts are (x,y)

        # get map data from server
        self.map_topic = rospy.get_param("/map_topic")
        map_data = rospy.wait_for_message(self.map_topic, OccupancyGrid)
        self.map_bounds = None
        self.setup_map(map_data)

        # Adding goals
        rospy.Subscriber("goals/add_goal", AltruisticGoal, self.receive_add_goal)
        self.add_goal_pub = rospy.Publisher(
            "goals/add_goal", AltruisticGoal, queue_size=10
        )

        # Removing goals
        rospy.Subscriber("goals/remove_goal", Goal, self.receive_remove_goal)
        self.remove_goal_pub = rospy.Publisher(
            "goals/remove_goal", AltruisticGoal, queue_size=10
        )

        # goal and winner
        rospy.Subscriber("goals/winner_goal", Goal, self.receive_winner_goal)

        # start publishing goals
        self.maintain_queue()

    ####################################################################

    def receive_add_goal(self, msg):
        self.goals.append((msg.goal_point.x, msg.goal_point.y))

    def receive_remove_goal(self, msg):
        if (msg.goal_point.x, msg.goal_point.y) in self.goals:
            self.goals.remove((msg.goal_point.x, msg.goal_point.y))

    def receive_winner_goal(self, msg):
        if (msg.goal_point.x, msg.goal_point.y) in self.goals:
            self.goals.remove((msg.goal_point.x, msg.goal_point.y))

    ####################################################################

    def publish_add_goal(self, goal, reward):
        msg = Goal()
        msg.reward = reward

        goal_point = Point()
        goal_point.x = goal[0]
        goal_point.y = goal[1]

        msg.goal_point = goal_point
        if not rospy.is_shutdown():
            self.add_goal_pub.publish(msg)
            self.goals.append(goal)

    def publish_remove_goal(self, goal):
        msg = Point()
        msg.x = goal[0]
        msg.y = goal[1]

        if goal in self.goals and not rospy.is_shutdown():
            self.remove_goal_pub.publish(msg)
            self.goals.remove(goal)

    ####################################################################

    def setup_map(self, map_data):
        """
        """
        map_metadata = map_data.info
        res = map_metadata.resolution

        maxx = (map_metadata.origin.position.x) + (map_metadata.width * res)
        maxy = (map_metadata.origin.position.y) + (map_metadata.height * res)
        minx = maxx - (map_metadata.width * res)
        miny = maxy - (map_metadata.height * res)
        self.map_bounds = ((minx, maxx), (miny, maxy))

    ####################################################################

    def maintain_queue(self):
        # Randomly generates goals with as many goals as agents initially
        # and new goals added periodically
        #
        # Options:
        #   Init with number of agents
        #   Remove goals / only add goals
        #   Constant reward / variable reward

        # periodically add goals
        while not rospy.is_shutdown():
            sleep(1.0)
            x = random.uniform(self.map_bounds[0][0], self.map_bounds[0][1])
            y = random.uniform(self.map_bounds[1][0], self.map_bounds[1][1])
            reward = random.randint(0, 100)
            self.publish_add_goal((x, y), reward)

            # if random.uniform(0, 1) > .8:
            #     goal = random.choice(self.goals)
            #     self.publish_remove_goal(goal)


if __name__ == "__main__":
    rospy.init_node("goal_updater", anonymous=True)
    node = GoalUpdater()

    rospy.spin()
