#!/usr/bin/env python
import numpy as np
import rospy

from agent_continuation import ContinuationAgent


class EuclideanAgent(ContinuationAgent):
    """
    An Agent that uses DMA-RRT for distributed path planning and continues to the next closest goal in the queue by Euclidean distance
    """

    def __init__(self, *args, **kwargs):
        super(EuclideanAgent, self).__init__(*args, **kwargs)

    def take_goal(self):
        eucl = lambda g: np.linalg.norm(
            np.array((g.x, g.y)) - np.array((self.rrt.curr_pos.x, self.rrt.curr_pos.y))
        )
        i = np.argmin(map(eucl, self.queue))
        return self.queue.pop(i)


if __name__ == "__main__":
    rospy.init_node("agent", anonymous=True)
    agent = EuclideanAgent()

    rospy.spin()
