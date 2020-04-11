from solutions.agent import Agent
from shapely.geometry import Point
from shapely.geometry import Polygon
from utils.environment import Environment
from solutions.plan import Plan, sol_individual

def test_random_token_holder():
    print("testing test_random_token_holder...\n")

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

    assert not agent1.token_holder
    assert not agent2.token_holder

    plan = Plan(agents = [agent1, agent2],
                env = environment,
                dma_indiv = sol_individual,
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
                   goal_dist = 0.3)
    agent2 = Agent(mode = "normal",
                   start_pos = (1, 1),
                   goal_pos = (10.5, 5.5),
                   environment = environment,
                   goal_dist = 0.3)

    assert not agent1.bids.keys()
    assert not agent2.bids.keys()
    assert not agent1.other_agent_plans.keys()
    assert not agent2.other_agent_plans.keys()

    plan = Plan(agents = [agent1, agent2],
                env = environment,
                dma_indiv = sol_individual,
                dma_coop = None,
                spin_rate = 10  # Hz
                )

    assert agent1.antenna.uuid in agent1.bids
    assert agent1.antenna.uuid in agent1.other_agent_plans
    assert agent2.antenna.uuid in agent1.bids
    assert agent2.antenna.uuid in agent1.other_agent_plans
    assert agent1.antenna.uuid in agent2.bids
    assert agent1.antenna.uuid in agent2.other_agent_plans
    assert agent2.antenna.uuid in agent2.bids
    assert agent2.antenna.uuid in agent2.other_agent_plans
