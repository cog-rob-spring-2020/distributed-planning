#!/usr/bin/env python
import rospy
from distributed_planning.msg import WinnerID, Queue, Stats

class AnalysisContinuation(object):
    def __init__(self):
        rospy.Subscriber("queue", Queue, self.queue_cb)
        rospy.Subscriber("plan/winner_id", WinnerID, self.winner_cb)
        self.stats_pub = rospy.Publisher("stats", Stats, queue_size=10)

        self.planning_rounds = 0
        self.queue = []

        # iters by goal
        self.goals = {}
        # rounds to complete by goal
        self.completed = {}

    def queue_cb(self, msg):
        # got a new copy of the queue!
        self.queue = [(goal_point.x, goal_point.y) for goal_point in msg.goal_points]

        # make sure all the goals in it have been initialized
        goal_locations = [(goal.x, goal.y) for goal in msg.goal_points]

        for location in goal_locations:
            if location not in self.goals:
                self.goals[location] = 0

        for location in self.goals:
            if location not in goal_locations and location not in self.completed:
                self.completed[location] = self.goals[location]

    def winner_cb(self, msg):
        # increment round
        self.planning_rounds += 1

        for location in self.queue:
            self.goals[location] += 1

        avg_age = sum(self.goals.values()) / len(self.goals) if len(self.goals) > 0 else 0.0

        avg_age_completed = sum(self.completed.values()) / len(self.completed) if len(self.completed) > 0 else 0.0

        msg = Stats(
            round=self.planning_rounds,
            avg_age=avg_age,
            avg_age_completed=avg_age_completed,
            completed=len(self.completed),
            total=len(self.goals)
        )
        self.stats_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("analysis", anonymous=True)
    node = AnalysisContinuation()

    rospy.spin()
