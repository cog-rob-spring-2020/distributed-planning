from agent import Agent
from rrtstar import Path
from environment import Environment

def test_antenna_callback_bids(received_bid, received_waypoints):
    print("testing test_antenna_callback_bids...\n")

    environment = Environment(yaml_file="utils/simple.yaml")

    agent1 = Agent(mode = "normal",
                   received_bid = received_bid,
                   received_waypoints = received_waypoints,
                   start_pos = (0, 0),
                   goal_pos = (10.5, 5.5),
                   environment = environment,
                   goal_dist = 0.3,
                   rrt_iters = 200)
    agent2 = Agent(mode = "normal",
                   received_bid = received_bid,
                   received_waypoints = received_waypoints,
                   start_pos = (2, 3),
                   goal_pos = (0, 4),
                   environment = environment,
                   goal_dist = 0.3,
                   rrt_iters = 200)

    agent1.broadcast_id()
    agent2.broadcast_id()

    agent1.broadcast_bid(10)
    agent2.broadcast_bid(20)

    assert agent2.bids[agent1.antenna.uuid] == 10
    assert agent1.bids[agent2.antenna.uuid] == 20

def test_antenna_callback_waypoints(received_bid, received_waypoints):
    print("testing test_antenna_callback_waypoints...\n")

    environment = Environment(yaml_file="utils/simple.yaml")

    agent1 = Agent(mode = "normal",
                   received_bid = received_bid,
                   received_waypoints = received_waypoints,
                   start_pos = (0, 0),
                   goal_pos = (10.5, 5.5),
                   environment = environment,
                   goal_dist = 0.3,
                   rrt_iters = 200)
    agent2 = Agent(mode = "normal",
                   received_bid = received_bid,
                   received_waypoints = received_waypoints,
                   start_pos = (2, 3),
                   goal_pos = (0, 4),
                   environment = environment,
                   goal_dist = 0.3,
                   rrt_iters = 200)

    agent1.broadcast_id()
    agent2.broadcast_id()

    assert not agent1.token_holder
    assert not agent2.token_holder

    agent1.curr_plan = Path()
    agent1.broadcast_waypoints(agent2.antenna.uuid)

    assert not agent2.other_agent_plans[agent1.antenna.uuid].nodes
    assert agent2.token_holder
    assert not agent1.token_holder