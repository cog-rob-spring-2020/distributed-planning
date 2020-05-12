#!/usr/bin/env python
import rospy
import random
from time import sleep
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from distributed_planning.msg import AltruisticGoal, AltruisticQueue


class GoalUpdaterAltruistic(object):
    def __init__(self):
        # init variables
        self.queue = []  # elts are (x,y)

        # get map data from server
        self.map_topic = rospy.get_param("/map_topic")
        map_data = rospy.wait_for_message(self.map_topic, OccupancyGrid)
        self.map_bounds = None
        self.setup_map(map_data)

        rospy.Subscriber("queue", AltruisticQueue, self.queue_cb)
        self.queue_pub = rospy.Publisher("queue", AltruisticQueue, queue_size=10)

        # start publishing goals
        self.maintain_queue()

    def queue_cb(self, msg):
        """
        Update the queue

        Params:
            msg distributed_planning.msg.Queue
        """
        self.queue = msg.goals

    def publish_add_goal(self, goal):
        point = Point()
        point.x = goal[0]
        point.y = goal[1]

        goal = AltruisticGoal(location=point, agent_id="")

        if not rospy.is_shutdown():
            self.queue.append(goal)
            msg = AltruisticQueue(goals=self.queue)
            self.queue_pub.publish(msg)

    # def publish_remove_goal(self, goal):
    #     msg = Point()
    #     msg.x = goal[0]
    #     msg.y = goal[1]

    #     if goal in self.queue and not rospy.is_shutdown():
    #         self.remove_goal_pub.publish(msg)
    #         self.queue.remove(goal)

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
    node = GoalUpdaterAltruistic()

    rospy.spin()
