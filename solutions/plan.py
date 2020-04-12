import numpy as np
from matplotlib import pyplot as plt
import time
import random

from solutions.agent import Agent
from solutions.rrtstar import RRTstar, Path
from solutions.visualizer import Visualizer

class Plan:
    def __init__(self, agents, env, dma_indiv, dma_coop, spin_rate,
                 viz_trees=False, headless=False):
        """ Plan object runs multiagent experiments with DMA-RRT.

            Args:
                agents: a list containing Agent objects
                env: an Environment object
                dma_indiv: a method representing the individual component
                    of the DMA-RRT algorithm.
                dma_coop: a method representing the cooperative extension
                    of the DMA-RRT algorithm.
                spin_rate: the rate (in Hz) at which the planner should run
                viz_trees: if True, visualizer will show RRT trees for all
                    agents in orange.
                headless: if True, nothing will be visualized during spin.
        """
        self.agents = agents

        # randomly assign one agent to hold the token for replanning
        random.choice(self.agents).token_holder = True

        # make all the agents aware of the antenna IDs of their peers
        for agent in self.agents:
            agent.broadcast_id()

        self.env = env
        self.spin_rate = spin_rate
        self.curr_time = 0
        self.viz_trees = viz_trees
        self.headless = headless

        Plan.dma_individual_normal = dma_indiv
        Plan.dma_individual_coop = dma_coop

        self.visualizer = Visualizer(self.env)

        # Do multithreading setup here.

    def dma_individual_normal(self, agent):
        raise NotImplementedError

    def dma_individual_coop(self, agent):
        raise NotImplementedError

    def spin(self, run_time):
        """ Run DMA-RRT experiments for all agents.

            Args:
                run_time: a float (atm) representing the length of the
                    experiment.
        """
        plt.ion()
        plt.show()

        agent_goal_stats = [agent.at_goal() for agent in self.agents]
        rate_dt = 1.0 / self.spin_rate

        while (False in agent_goal_stats) and (self.curr_time <= run_time):
            start_time = time.time()

            self.spin_once()
            if not self.headless:
                self.visualizer.spin_once(self.agents, self.viz_trees)

            agent_goal_stats = [agent.at_goal() for agent in self.agents]

            time_elapsed = time.time() - start_time
            if time_elapsed < rate_dt:
                plt.pause(rate_dt - time_elapsed)
                time.sleep(rate_dt - time_elapsed)

            plt.show()

        print("Runtime completed. Time: ", self.curr_time)

    def spin_once(self):
        """ Run one iteration of DMA-RRT experiments for all agents. """
        print("Planner spinning once. Time = ", self.curr_time)

        token_holder_id = np.where([agent.token_holder for agent in self.agents])[0][0]
        for i in range(len(self.agents)):
            # Do non-token-holders first:
            if (i != token_holder_id) and (not self.agents[i].at_goal()):
                self.agents[i].curr_time = self.curr_time
                self.agents[i].spin_once()

                # Run one iteration of the individual method for DMA-RRT
                if self.agents[i].mode == "normal":
                    self.dma_individual_normal(self.agents[i])
                elif self.agents[i].mode == "cooperative":
                    self.dma_individual_coop(self.agents[i])

        # Do token-holder step last:
        self.agents[token_holder_id].curr_time = self.curr_time
        self.agents[token_holder_id].spin_once()

        if self.agents[token_holder_id].mode == "normal":
            self.dma_individual_normal(self.agents[token_holder_id])
        elif self.agents[token_holder_id].mode == "cooperative":
            self.dma_individual_coop(self.agents[token_holder_id])

        self.curr_time += 1

    @staticmethod
    def multiagent_aware_time_realloc(path, other_agent_plans):
        """ Allocates more time to nodes in a path that conflict with other
            agent nodes, s.t. the conflict is gone.

            Only tokenholders should use this, to prevent duplicate conflict
            resolution.

            Args:
                path: a Path object.
                other_agent_plans: a dictionary keyed by Agent ID containing
                    the current Path objects of other agents in the scenario.
            Returns:
                a Path object similar to `path` but with collision-free
                    timestamps on each node.
        """
        for i in range(1,len(path.nodes)):
            # Check for collisions with any other node:
            for id, other_path in other_agent_plans.items():
                if other_path:
                    curr_stamp = path.nodes[i].stamp
                    if curr_stamp in other_path.ts_dict.keys():
                        if path.nodes[i] == other_path.ts_dict[curr_stamp]:
                            # Collision detected! Allocate more time to this node.

                            revised_stamp = curr_stamp
                            while path.nodes[i] == other_path.ts_dict[revised_stamp]:
                                revised_stamp += 1
                                path.nodes[i].stamp = revised_stamp

                            # Update all nodes after that one to have higher timetsamps.
                            for node in path.nodes[i+1:]:
                                revised_stamp += 1
                                node.stamp = revised_stamp

        path.ts_dict = {node.stamp : node for node in path.nodes}
        return path


def sol_individual(self, agent):
    """ Individual component of DMA-RRT as described in algorithm 4
        from Desaraju/How 2012.

        If this agent is the TokenHolder, it will update its internal
        plan and broadcast the next winner. Otw, it will broadcast its bid.
    """
    # Refresh environment to reflect agents' current positions
    agent.rrt.update_pos(agent.pos, wipe_tree=True)

    # Grow the tree by one set of iterations
    agent.rrt.spin(False)

    # Find the new best path in the tree
    new_plan = agent.rrt.get_path()

    # Assign first "current path" found
    if not agent.curr_plan.nodes:
        agent.curr_plan = new_plan
    agent.best_plan = new_plan

    if agent.token_holder:

        # print("i am token_holder:", agent.antenna.uuid)

        # Replan to new best path
        agent.curr_plan = agent.best_plan

        # Solve collisions with time reallocation
        agent.curr_plan = Plan.multiagent_aware_time_realloc(agent.curr_plan,
            agent.other_agent_plans)

        # Broadcast the new winner of the bidding round
        if agent.bids:
            agent.token_holder = False
            winner_bid = max(agent.bids.values())
            winner_ids = [id for id, bid in agent.bids.items() \
                if bid == winner_bid]
            winner_id = random.choice(winner_ids)

            agent.broadcast_waypoints(winner_id)
    else:
        # Broadcast own bid
        # We use abs in the event that we are comparing an incomplete path
        #     to a complete path. Incomplete paths will usually have smaller
        #     cost; see Path definition for more info.
        bid = np.abs(agent.curr_plan.cost - agent.best_plan.cost)
        agent.broadcast_bid(bid)

    # Prevent agent from getting the token if they finish.
    if agent.at_goal():
        agent.broadcast_bid(-1000.0)

def sol_coop_individual(self, agent):
    """ Individual component of Cooperative DMA-RRT as described
        in algorithm 6 from Desaraju/How 2012.
    """
    # # Grow tree one iteration while ignoring other agents' paths
    # agent.rrt.update_agent_plans(dict())
    # agent.rrt.update_pos(agent.pos)
    # agent.rrt.spin_once(False)

    # # Find the new best path in the tree, and save it if it's better
    # new_plan = agent.rrt.get_path()
    # if new_plan.cost < agent.best_plan.cost and \
    #         new_plan.cost < agent.cur_plan.cost:
    #     agent.best_plan = new_plan

    # if agent.token_holder:
    #     # Replan to new best path if it's better
    #     plan = agent.curr_plan
    #     if agent.best_plan.cost < agent.curr_plan.cost:
    #         plan = agent.best_plan
    #         agent.best_plan = Path()

    #     conflicting_plans = RRTstar.get_conflicts(plan)
    #     conflicting_ids = list(conflicting_plans.keys())

    #     j = None
    #     if conflicting_ids:
    #         j = conflicting_ids[0]
    #         plan, modified_j = check_emergency_stops(plan, j, conflicting_plans[j])

    #     if len(conflicting_ids > 1):
    #         for j_prime in conflicting_ids[1:]:
    #             j_prime_plan = conflicting_plans[j_prime]
    #             if agent.rrt.plans_conflict(plan, j_prime_plan):
    #                 # Prune our plan to avoid the conflict
    #                 plan = rrt.prune_to_avoid_conflict(new_plan, j_prime_plan)

    #     agent.curr_plan = plan

    #     # Broadcast the new winner of the bidding round
    #     if j is not None and modified_j:
    #         winner = j
    #     else:
    #         winner = max(agent.bids, key = lambda x: agent.bids[x])
    #     agent.broadcast_waypoints(winner)
    #     agent.token_holder = False
    # else:
    #     # Broadcast own bid
    #     bid = agent.curr_plan.cost - agent.best_plan.cost
    #     agent.broadcast_bid(bid)
