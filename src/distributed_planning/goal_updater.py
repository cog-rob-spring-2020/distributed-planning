#!/usr/bin/env python
import rospy
import random
from time import sleep
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from distributed_planning.msg import Goal, WinnerIDGoal

class GoalUpdater(object):
    def __init__(self, num_agents, env):
        #init variables
        self.num_agents = num_agents
        self.goals = [] #elts are (x,y)

        #init map
        self.map_metadata = None
        self.map_bounds = None
        self.setup_map(env)

        #Adding goals
        rospy.Subscriber("add_goal", Goal, self.receive_add_goal)
        self.add_goal_pub = rospy.Publisher("add_goal", Goal, queue_size=10)

        #Removing goals
        rospy.Subscriber("remove_goal", Point, self.receive_remove_goal)
        self.remove_goal_pub = rospy.Publisher("remove_goal", Point, queue_size=10)

        #goal and winner
        rospy.Subscriber("winner_and_goal", WinnerIDGoal, self.receive_winner_and_goal)

        #start publishing goals
        self.maintain_queue()

    ####################################################################

    def receive_add_goal(self, msg):
        self.goals.append((msg.goal_id.x, msg.goal_id.y))

    def receive_remove_goal(self, msg):
        if (msg.x, msg.y) in self.goals:
            self.goals.remove((msg.x, msg.y))

    def receive_winner_and_goal(self, msg):
        if (msg.x, msg.y) in self.goals:
            self.goals.remove((msg.goal_id.x, msg.goal_id.y))

    ####################################################################

    def publish_add_goal(self, goal, reward):
        msg = Goal()
        msg.reward = reward

        goal_id = Point()
        goal_id.x = goal[0]
        goal_id.y = goal[1]

        msg.goal_id = goal_id
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

    def setup_map(self, env):
        """
        """
        self.map_metadata = env.info
        res = self.map_metadata.resolution

        maxx = (self.map_metadata.origin.position.x + float(self.map_metadata.width) / 2) * res
        maxy = (self.map_metadata.origin.position.y + float(self.map_metadata.height) / 2) * res
        minx = maxx - (self.map_metadata.width * res)
        miny = maxy - (self.map_metadata.height * res)
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

        #start with as many goals as agents, potentially need to wait for map
        for i in range(self.num_agents):
            x = random.randrange(self.map_bounds[0][0], self.map_bounds[0][1])
            y = random.randrange(self.map_bounds[1][0], self.map_bounds[1][1])
            reward = random.randint(0, 100)
            self.publish_add_goal((x,y), reward)

        #periodically add goals
        while not rospy.is_shutdown():
            sleep(1.0)
            x = random.randrange(self.map_bounds[0][0], self.map_bounds[0][1])
            y = random.randrange(self.map_bounds[1][0], self.map_bounds[1][1])
            reward = random.randint(0, 100)
            self.publish_add_goal((x,y), reward)

            # if random.randrange(0, 1) > .8:
            #     goal = random.choice(self.goals)
            #     self.publish_remove_goal(goal)


