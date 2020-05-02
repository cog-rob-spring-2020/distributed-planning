#!/usr/bin/env python

# https://answers.ros.org/question/264767/plotting-real-time-data/

import numpy as np
from matplotlib import pyplot as plt
import rospy
from qualisys.msg import Subject


def plot_x(msg):
    global counter
    if counter % 10 == 0:
        stamp = msg.header.stamp
        time = stamp.secs + stamp.nsecs * 1e-9
        plt.plot(msg.position.y, msg.position.x, '*')
        plt.axis("equal")
        plt.draw()
        plt.pause(0.00000000001)

    counter += 1


if __name__ == '__main__':
    counter = 0

    rospy.init_node("plotter")
    rospy.Subscriber("position_measurements", Subject, plot_x)
    plt.ion()
    plt.show()
    rospy.spin()
