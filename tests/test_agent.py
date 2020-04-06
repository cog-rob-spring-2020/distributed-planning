from solutions.agent import Agent
from shapely.geometry import Point
from shapely.geometry import Polygon
from utils.environment import Environment

def test_at_goal():
    print("testing test_at_goal...")

    # set up a known feasible environment
    environment = Environment(yaml_file="utils/simple.yaml")

    radius = 0.3
    bounds = (-2, -3, 12, 8)
    start = (0, 0)
    goal_region = Polygon([(10, 5), (10, 6), (11, 6), (11, 5)])

    agent = Agent("normal", 0, 0, goal_region, environment, bounds, radius)

    assert (not agent.at_goal())

    agent.pose = Point(10.5, 5.5)

    assert agent.at_goal()


def test_antenna_callbacks():
    print("testing test_antenna_callback...")

    # set up a known feasible environment
    environment = Environment(yaml_file="utils/simple.yaml")

    radius = 0.3
    bounds = (-2, -3, 12, 8)
    start = (0, 0)
    goal_region = Polygon([(10, 5), (10, 6), (11, 6), (11, 5)])

    agent = Agent('normal', 0, 0, goal_region, environment, bounds, radius)

    agent.antenna.broadcast("bids", {"topic":"bids", "bid":10})
    assert agent.bids[agent.antenna.uuid] == 10
