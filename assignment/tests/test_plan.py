from shapely.geometry import Point
from shapely.geometry import Polygon

from agent import Agent
from rrtstar import Path
from plan import Plan
from environment import Environment

def test_random_token_holder():
    print("testing test_random_token_holder...\n")

    environment = Environment(yaml_file="utils/simple.yaml")

    agent1 = Agent(mode = "normal",
                   start_pos = (0, 0),
                   goal_pos = (10.5, 5.5),
                   environment = environment,
                   goal_dist = 0.3,
                   rrt_iters = 200)
    agent2 = Agent(mode = "normal",
                   start_pos = (2, 3),
                   goal_pos = (0, 4),
                   environment = environment,
                   goal_dist = 0.3,
                   rrt_iters = 200)

    assert not agent1.token_holder
    assert not agent2.token_holder

    plan = Plan(agents = [agent1, agent2],
                env = environment,
                dma_indiv = sol_dma_individual,
                dma_coop = None,
                spin_rate = 10  # Hz
                )

    assert agent1.token_holder or agent2.token_holder
    assert not (agent1.token_holder and agent2.token_holder)

def test_agents_aware_of_peers():
    print("testing test_agents_aware_of_peers...\n")

    environment = Environment(yaml_file="utils/simple.yaml")

    agent1 = Agent(mode = "normal",
                   start_pos = (0, 0),
                   goal_pos = (10.5, 5.5),
                   environment = environment,
                   goal_dist = 0.3,
                   rrt_iters = 200)
    agent2 = Agent(mode = "normal",
                   start_pos = (2, 3),
                   goal_pos = (0, 4),
                   environment = environment,
                   goal_dist = 0.3,
                   rrt_iters = 200)

    assert not agent1.bids.keys()
    assert not agent2.bids.keys()
    assert not agent1.other_agent_plans.keys()
    assert not agent2.other_agent_plans.keys()

    plan = Plan(agents = [agent1, agent2],
                env = environment,
                dma_indiv = sol_dma_individual,
                dma_coop = None,
                spin_rate = 10  # Hz
                )

    assert agent1.antenna.uuid not in agent1.bids
    assert agent1.antenna.uuid not in agent1.other_agent_plans
    assert agent2.antenna.uuid in agent1.bids
    assert agent2.antenna.uuid in agent1.other_agent_plans
    assert agent1.antenna.uuid in agent2.bids
    assert agent1.antenna.uuid in agent2.other_agent_plans
    assert agent2.antenna.uuid not in agent2.bids
    assert agent2.antenna.uuid not in agent2.other_agent_plans

def test_compute_winner():
    print("testing test_compute_winner...\n")

    environment = Environment(yaml_file="utils/simple.yaml")

    agent1 = Agent(mode = "normal",
                   start_pos = (0, 0),
                   goal_pos = (10.5, 5.5),
                   environment = environment,
                   goal_dist = 0.3,
                   rrt_iters = 200)
    agent2 = Agent(mode = "normal",
                   start_pos = (2, 3),
                   goal_pos = (0, 4),
                   environment = environment,
                   goal_dist = 0.3,
                   rrt_iters = 200)
    agent3 = Agent(mode = "normal",
                   start_pos = (8, 1),
                   goal_pos = (10, -1),
                   environment = environment,
                   goal_dist = 0.3,
                   rrt_iters = 200)

    agent1.broadcast_bid(-1000)
    agent2.broadcast_bid(0)
    agent3.broadcast_bid(100)

    assert sol_compute_winner(agent1) == agent3.antenna.uuid
    assert sol_compute_winner(agent2) == agent3.antenna.uuid

    # since agent 3 wouldn't keep track of its own bid
    assert sol_compute_winner(agent3) == agent2.antenna.uuid

def test_bid():
    print("testing test_bid...\n")

    environment = Environment(yaml_file="utils/simple.yaml")

    agent1 = Agent(mode = "normal",
                   start_pos = (0, 0),
                   goal_pos = (10.5, 5.5),
                   environment = environment,
                   goal_dist = 0.3,
                   rrt_iters = 200)
    agent2 = Agent(mode = "normal",
                   start_pos = (2, 3),
                   goal_pos = (0, 4),
                   environment = environment,
                   goal_dist = 0.3,
                   rrt_iters = 200)

    agent1.curr_plan = Path()
    agent1.best_plan = Path()

    agent1.curr_plan.cost = 10
    agent1.best_plan.cost = 100
    sol_bid(agent1)
    assert agent2.bids[agent1.antenna.uuid] == 90

    agent1.curr_plan.cost = 50
    agent1.best_plan.cost = 0
    sol_bid(agent1)
    assert agent2.bids[agent1.antenna.uuid] == 50
