from solutions.plan import Plan, sol_individual, sol_coop_individual
from solutions.agent import Agent
from utils.environment import Environment

env = Environment("../utils/simple.yaml")

agents = [Agent("normal", [0, 0], [10, 6], env, 0.75, 200),
          Agent("normal", [0, 5], [4, 6], env, 0.75, 200),
          Agent("normal", [4, -2], [-1.7, 0], env, 0.75, 200)]

plan = Plan(agents, env, sol_individual, sol_coop_individual, 100, viz_trees=False, headless=False)
plan.spin(75)

agents = [Agent("normal", [4, -2], [-1.7, 0], env, 0.3, 200)]

plan = Plan(agents, env, sol_individual, sol_coop_individual, 100, viz_trees=True, headless=False)
plan.spin(50)
# plan.visualizer.spin_once(plan.agents, True)  # If running headless, uncomment to visualize at end.
