from solutions.agent import Agent
from shapely.geometry import Point
from shapely.geometry import Polygon
from utils.environment import Environment

def test_at_goal():
    print("testing test_at_goal...\n")

    agent = Agent(mode = "normal",
                  start_pos = (0, 0),
                  goal_pos = (10.5, 5.5),
                  environment = Environment(yaml_file="utils/simple.yaml"),
                  goal_dist = 0.3)

    assert not agent.at_goal()

    agent.pos = (10.5, 5.5)

    assert agent.at_goal()

def test_antenna_callback_bids():
    print("testing test_antenna_callback_bids...\n")

    environment = Environment(yaml_file="utils/simple.yaml")

    agent1 = Agent(mode = "normal",
                   start_pos = (0, 0),
                   goal_pos = (10.5, 5.5),
                   environment = environment,
                   goal_dist = 0.3)
    agent2 = Agent(mode = "normal",
                   start_pos = (1, 1),
                   goal_pos = (10.5, 5.5),
                   environment = environment,
                   goal_dist = 0.3)

    # agent1.antenna.broadcast("bids", {"topic":"bids", "bid":10})
    # agent2.antenna.broadcast("bids", {"topic":"bids", "bid":20})
    agent1.broadcast_bid(10)
    agent2.broadcast_bid(20)

    assert agent1.bids[agent1.antenna.uuid] == 10
    assert agent2.bids[agent1.antenna.uuid] == 10
    assert agent1.bids[agent2.antenna.uuid] == 20
    assert agent2.bids[agent2.antenna.uuid] == 20

def test_antenna_callback_waypoints():
    print("testing test_antenna_callback_waypoints...\n")

    environment = Environment(yaml_file="utils/simple.yaml")

    agent1 = Agent(mode = "normal",
                   start_pos = (0, 0),
                   goal_pos = (10.5, 5.5),
                   environment = environment,
                   goal_dist = 0.3)
    agent2 = Agent(mode = "normal",
                   start_pos = (1, 1),
                   goal_pos = (10.5, 5.5),
                   environment = environment,
                   goal_dist = 0.3)

    # agent1.antenna.broadcast("waypoints", {"topic":"waypoints", "plan":[], "winner_id":agent2.antenna.uuid})
    agent1.curr_plan = []
    agent1.broadcast_waypoints(agent2.antenna.uuid)

    assert agent1.other_agent_plans[agent1.antenna.uuid] == []
    assert agent2.other_agent_plans[agent1.antenna.uuid] == []
    assert agent2.token_holder

def test_antenna_callback_peers():
    print("testing test_antenna_callback_peers...\n")

    environment = Environment(yaml_file="utils/simple.yaml")

    agent1 = Agent(mode = "normal",
                   start_pos = (0, 0),
                   goal_pos = (10.5, 5.5),
                   environment = environment,
                   goal_dist = 0.3)
    agent2 = Agent(mode = "normal",
                   start_pos = (1, 1),
                   goal_pos = (10.5, 5.5),
                   environment = environment,
                   goal_dist = 0.3)

    assert agent1.antenna.uuid not in agent1.bids
    assert agent1.antenna.uuid not in agent1.other_agent_plans
    assert agent2.antenna.uuid not in agent1.bids
    assert agent2.antenna.uuid not in agent1.other_agent_plans
    assert agent1.antenna.uuid not in agent2.bids
    assert agent1.antenna.uuid not in agent2.other_agent_plans
    assert agent2.antenna.uuid not in agent2.bids
    assert agent2.antenna.uuid not in agent2.other_agent_plans

    # agent1.antenna.broadcast("peers", {"topic":"peers"})
    agent1.broadcast_id()

    assert agent1.bids[agent1.antenna.uuid] is None
    assert agent1.other_agent_plans[agent1.antenna.uuid] is None
    assert agent2.bids[agent1.antenna.uuid] is None
    assert agent2.other_agent_plans[agent1.antenna.uuid] is None

    # agent2.antenna.broadcast("peers", {"topic":"peers"})
    agent2.broadcast_id()

    assert agent1.bids[agent2.antenna.uuid] is None
    assert agent1.other_agent_plans[agent2.antenna.uuid] is None
    assert agent2.bids[agent2.antenna.uuid] is None
    assert agent2.other_agent_plans[agent2.antenna.uuid] is None
