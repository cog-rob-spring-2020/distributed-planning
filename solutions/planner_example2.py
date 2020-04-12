from solutions.plan import Plan, sol_individual, sol_coop_individual
from solutions.agent import Agent
from utils.environment import Environment

env = Environment("../utils/simple.yaml")

agents = [Agent("normal", [0, 0], [10, 6], env, 0.5, 200),
          Agent("normal", [0, 5], [4, 6], env, 0.5, 200)]

plan = Plan(agents, env, sol_individual, sol_coop_individual, 100)
plan.spin(900)
