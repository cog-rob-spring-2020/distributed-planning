from utils.environment import Environment

class Plan():
    def __init__(self, agents):
        self.agents = agents

    def visualize():
        pass

    def spin():
        # TODO
        # spin up a thread for each agent, call agent.spin in parallel for each agent
        pass

def main():
    # set up environment with static obstacles shared among agents
    environment = Environment(yaml_file="utils/simple.yaml")

    # initialize all of the agents that will live in environment
    agent = Agent(mode = "normal",
                  x_start = 0,
                  y_start = 0,
                  goal_region = Polygon([(10, 5), (10, 6), (11, 6), (11, 5)]),
                  environment = environment,
                  bounds = (-2, -3, 12, 8),
                  goal_dist = 0.3)
    agents = [agent]

    plan = Plan(agents)
    plan.spin()
