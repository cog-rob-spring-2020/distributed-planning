from solutions.agent import Agent
from shapely.geometry import Point
from shapely.geometry import Polygon
from utils.environment import Environment

def test_at_goal():
    print("testing test_at_goal...")

    # goal = Polygon([(10, 5), (10, 6), (11, 6), (11, 5)])
    goal = Point(10.5, 5.5)

    agent = Agent(mode = "normal",
                  x_start = 0,
                  y_start = 0,
                  goal_region = goal,
                  environment = Environment(yaml_file="utils/simple.yaml"),
                  bounds = (-2, -3, 12, 8),
                  goal_dist = 0.3)

    assert not agent.at_goal()

    agent.pose = Point(10.5, 5.5)

    assert agent.at_goal()


def test_antenna_callback_bids():
    print("testing test_antenna_callback_bids...")

    environment = Environment(yaml_file="utils/simple.yaml")

    # goal = Polygon([(10, 5), (10, 6), (11, 6), (11, 5)])
    goal = Point(10.5, 5.5)

    agent1 = Agent(mode = "normal",
                   x_start = 0,
                   y_start = 0,
                   goal_region = goal,
                   environment = environment,
                   bounds = (-2, -3, 12, 8),
                   goal_dist = 0.3)
    agent2 = Agent(mode = "normal",
                   x_start = 1,
                   y_start = 1,
                   goal_region = goal,
                   environment = environment,
                   bounds = (-2, -3, 12, 8),
                   goal_dist = 0.3)

    # TODO: run these asserts once we've implemented find_peers
    # assert agent1.bids[agent1.antenna.uuid] is None
    # assert agent2.bids[agent1.antenna.uuid] is None
    # assert agent1.bids[agent2.antenna.uuid] is None
    # assert agent2.bids[agent2.antenna.uuid] is None

    agent1.antenna.broadcast("bids", {"topic":"bids", "bid":10})
    agent2.antenna.broadcast("bids", {"topic":"bids", "bid":20})

    assert agent1.bids[agent1.antenna.uuid] == 10
    assert agent2.bids[agent1.antenna.uuid] == 10
    assert agent1.bids[agent2.antenna.uuid] == 20
    assert agent2.bids[agent2.antenna.uuid] == 20


def test_antenna_callback_waypoints():
    print("testing test_antenna_callback_waypoints")

    environment = Environment(yaml_file="utils/simple.yaml")

    # goal = Polygon([(10, 5), (10, 6), (11, 6), (11, 5)])
    goal = Point(10.5, 5.5)

    agent1 = Agent(mode = "normal",
                   x_start = 0,
                   y_start = 0,
                   goal_region = goal,
                   environment = environment,
                   bounds = (-2, -3, 12, 8),
                   goal_dist = 0.3)
    agent2 = Agent(mode = "normal",
                   x_start = 1,
                   y_start = 1,
                   goal_region = goal,
                   environment = environment,
                   bounds = (-2, -3, 12, 8),
                   goal_dist = 0.3)

    # TODO: run these asserts once we've implemented find_peers
    # assert agent1.plans[agent1.antenna.uuid] is None
    # assert agent2.plans[agent1.antenna.uuid] is None
    # assert not agent2.token_holder

    agent1.antenna.broadcast("waypoints", {"topic":"waypoints", "plan":[], "winner_id":agent2.antenna.uuid})

    assert agent1.plans[agent1.antenna.uuid] == []
    assert agent2.plans[agent1.antenna.uuid] == []
    assert agent2.token_holder
