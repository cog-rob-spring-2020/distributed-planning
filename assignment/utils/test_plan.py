from shapely.geometry import Point
from shapely.geometry import Polygon

from agent import Agent
from rrtstar import Path
from plan import Plan
from environment import Environment

def test_compute_winner(bid, waypoints, compute_winner):
    print("testing test_compute_winner...\n")

    environment = Environment(yaml_file="utils/simple.yaml")

    agent1 = Agent(mode = "normal",
                   received_bid = bid,
                   received_waypoints = waypoints,
                   start_pos = (0, 0),
                   goal_pos = (10.5, 5.5),
                   environment = environment,
                   goal_dist = 0.3,
                   rrt_iters = 200)
    agent2 = Agent(mode = "normal",
                   received_bid = bid,
                   received_waypoints = waypoints,
                   start_pos = (2, 3),
                   goal_pos = (0, 4),
                   environment = environment,
                   goal_dist = 0.3,
                   rrt_iters = 200)
    agent3 = Agent(mode = "normal",
                   received_bid = bid,
                   received_waypoints = waypoints,
                   start_pos = (8, 1),
                   goal_pos = (10, -1),
                   environment = environment,
                   goal_dist = 0.3,
                   rrt_iters = 200)

    agent1.broadcast_id()
    agent2.broadcast_id()
    agent3.broadcast_id()

    agent1.broadcast_bid(-1000)
    agent2.broadcast_bid(0)
    agent3.broadcast_bid(100)

    assert compute_winner(agent1) == agent3.antenna.uuid
    assert compute_winner(agent2) == agent3.antenna.uuid

    # since agent 3 wouldn't keep track of its own bid
    assert compute_winner(agent3) == agent2.antenna.uuid

def test_bid(bid, waypoints, send_bid):
    print("testing test_bid...\n")

    environment = Environment(yaml_file="utils/simple.yaml")

    agent1 = Agent(mode = "normal",
                   received_bid = bid,
                   received_waypoints = waypoints,
                   start_pos = (0, 0),
                   goal_pos = (10.5, 5.5),
                   environment = environment,
                   goal_dist = 0.3,
                   rrt_iters = 200)
    agent2 = Agent(mode = "normal",
                   received_bid = bid,
                   received_waypoints = waypoints,
                   start_pos = (2, 3),
                   goal_pos = (0, 4),
                   environment = environment,
                   goal_dist = 0.3,
                   rrt_iters = 200)

    agent1.broadcast_id()
    agent2.broadcast_id()

    agent1.curr_plan = Path()
    agent1.best_plan = Path()

    agent1.curr_plan.cost = 100
    agent1.best_plan.cost = 10
    send_bid(agent1)
    assert agent2.bids[agent1.antenna.uuid] == 90

    agent1.curr_plan.cost = 50
    agent1.best_plan.cost = 0
    send_bid(agent1)
    assert agent2.bids[agent1.antenna.uuid] == 50
