import numpy as np
from matplotlib import pyplot as plt
import time
import random

from agent import Agent
from rrtstar import RRTstar, Path
from visualizer import Visualizer

class Plan:
    def __init__(self, agents, env, dma_indiv, spin_rate,
                 viz_trees=False, headless=False):
        """ Plan object runs multiagent experiments with DMA-RRT.

            Args:
                agents: a list containing Agent objects
                env: an Environment object
                dma_indiv: a method representing the individual component
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

        self.visualizer = Visualizer(self.env)

        # Do multithreading setup here.

    def dma_individual_normal(self, agent):
        raise NotImplementedError

    def dma_individual_coop(self, agent):
        raise NotImplementedError

    def spin(self, iters):
        """ Run DMA-RRT experiments for all agents.

            Args:
                iters: a float (atm) representing the length of the
                    experiment.
        """
        plt.ion()
        plt.show()

        agent_goal_stats = [agent.at_goal() for agent in self.agents]
        rate_dt = 1.0 / self.spin_rate

        while (False in agent_goal_stats) and (self.curr_time <= iters):
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
                # Run one iteration of the individual method for DMA-RRT                
                if self.agents[i].mode == "normal":
                    self.dma_individual_normal(self.agents[i])

                self.agents[i].curr_time = self.curr_time
                self.agents[i].spin_once()

        # Do token-holder step last:
        self.agents[token_holder_id].curr_time = self.curr_time
        self.agents[token_holder_id].spin_once()

        if self.agents[token_holder_id].mode == "normal":
            self.dma_individual_normal(self.agents[token_holder_id])

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


def example_dma_indiv(self, agent):
    """ Individual component of DMA-RRT as described in algorithm 4
        from Desaraju/How 2012.

        This is a non-functioning example only for visualization at the
        start of the pset.
    """
    agent.token_holder = True
    
    # Refresh environment to reflect agent's current positions
    agent.rrt.update_pos(agent.pos, self.curr_time, wipe_tree=True)
    
    # Grow the tree by one set of iterations
    agent.rrt.spin(False)

    # Find the new best path in the tree
    new_plan = agent.rrt.get_path()
    agent.best_plan = new_plan
    
    if agent.token_holder:
        agent.curr_plan = agent.best_plan