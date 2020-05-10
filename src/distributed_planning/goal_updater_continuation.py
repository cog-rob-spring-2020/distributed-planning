#!/usr/bin/env python
import rospy
import random
from time import sleep
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from distributed_planning.msg import Goal, Queue


class GoalUpdaterContinuation(object):
    def __init__(self):
        # init variables
        self.queue = []

        # get map data from server
        self.map_topic = rospy.get_param("/map_topic")
        map_data = rospy.wait_for_message(self.map_topic, OccupancyGrid)
        self.map_bounds = None
        self.setup_map(map_data)

        rospy.Subscriber("queue", Queue, self.queue_cb)
        self.queue_pub = rospy.Publisher("queue", Queue, queue_size=10)

        # start publishing goals
        self.maintain_queue()

    def queue_cb(self, msg):
        """
        Update the queue

        Params:
            msg distributed_planning.msg.Queue
        """
        self.queue = msg.goal_points

    def publish_add_goal(self, goal):
        goal_point = Point()
        goal_point.x = goal[0]
        goal_point.y = goal[1]

        if not rospy.is_shutdown():
            self.queue.append(goal_point)
            msg = Queue(goal_points=self.queue)
            self.queue_pub.publish(msg)

    # def publish_remove_goal(self, goal):
    #     g = Point()
    #     g.x = goal[0]
    #     g.y = goal[1]

    #     if g in self.queue.goal_points and not rospy.is_shutdown():
    #         self.queue.remove(goal)
    #         msg = Queue(goal_points=self.queue)
    #         self.queue_pub.publish(msg)

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
            self.publish_add_goal((x, y))

            # if random.uniform(0, 1) > .8:
            #     goal = random.choice(self.queue)
            #     self.publish_remove_goal(goal)


if __name__ == "__main__":
    rospy.init_node("goal_updater", anonymous=True)
    node = GoalUpdaterContinuation()

    rospy.spin()
