#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from distributed_planning.msg import *
from agent import Agent
from environment import Environment


def viper(mode, start_pos, goal_pos, environment, goal_dist, rate):
    """
    Params:
        rate int frequency to run RRT in Hz
    """
    rospy.init_node('agent', anonymous=True)

    # handle registering peers
    registration_pub = rospy.Publisher("registration", Registration)

    # create publishers for data the agent will want to broadcast
    peer_pub = rospy.Publisher('peers', PlanBid, queue_size=10)
    bid_pub = rospy.Publisher('plan_bids', PlanBid, queue_size=10)
    waypoints_pub = rospy.Publisher('waypoints', Waypoints, queue_size=10)

    # create the agent and give it the necessary publishers
    agent = Agent(rospy.get_name(), mode, start_pos, goal_pos, environment,
                  goal_dist, peer_pub, bid_pub, waypoints_pub)

    # Register as listener for different kinds of messages
    rospy.Subscriber("registration", Registration, agent.received_id)
    rospy.Subscriber("plan_bids", PlanBid, agent.received_bid)
    rospy.Subscriber("waypoints", Waypoints, agent.received_waypoints)

    # let the other agents know a new agent is on the network
    registration_pub.publish(rospy.get_name())

    # main loop for this agent
    rate = rospy.Rate(rate)
    while not rospy.is_shutdown() and not agent.at_goal():
        agent.spin_once()
        rate.sleep()

    rospy.spin()


if __name__ == '__main__':
    try:
        # TODO: get argv instead
        mode = "normal"
        start_pos = (0., 0.)
        goal_pos = (10., 10.)
        environment = Environment()
        goal_dist = 0.1
        rrt_iters = 10
        viper(mode, start_pos, goal_pos, environment, goal_dist, rrt_iters)
    except rospy.ROSInterruptException:
        pass
