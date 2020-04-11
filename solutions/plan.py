import numpy as np
from matplotlib import pyplot as plt
import time

from agent import Agent
from rrtstar import RRTstar
from visualizer import Visualizer

class Plan:
    def __init__(self, agents, env, dma_indiv, dma_coop, spin_rate):
        """ Plan object runs multiagent experiments with DMA-RRT.

            Args:
                agents: a list containing Agent objects
                env: an Environment object
                dma_indiv: a method representing the individual component
                    of the DMA-RRT algorithm.
                dma_coop: a method representing the cooperative extension
                    of the DMA-RRT algorithm.
                spin_rate: the rate (in Hz) at which the planner should run
        """
        self.agents = agents
        self.env = env
        self.spin_rate = spin_rate
        self.curr_time = 0

        Plan.dma_individual_normal = dma_indiv
        Plan.dma_individual_coop = dma_coop

        self.path_costs = {
            agent.antenna.uuid : np.inf for agent in self.agents
        }

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

        while False in agent_goal_stats and self.curr_time <= run_time:
            # print("Spinning planner! Time = ", self.curr_time)
            start_time = time.time()

            new_path_costs = {
                agent.antenna.uuid : agent.curr_plan.cost \
                    for agent in self.agents
            }

            self.spin_once()

            run_viz = False
            for id, cost in self.path_costs.items():
                if new_path_costs[id] != cost:
                    run_viz = True
                    break
            # run_viz = True

            if run_viz:
                self.visualizer.spin_once(
                    {agent.antenna.uuid : agent.curr_plan \
                        for agent in self.agents}
                )
                # self.visualizer.spin_once_tree(self.agents[0].rrt)
            
            self.path_costs = new_path_costs

            agent_goal_stats = [agent.at_goal() for agent in self.agents]

            time_elapsed = time.time() - start_time
            if time_elapsed < rate_dt:
                plt.pause(rate_dt - time_elapsed)
                time.sleep(rate_dt - time_elapsed)

            plt.show()
        
        print("Runtime completed.")

    def spin_once(self):
        """ Run one iteration of DMA-RRT experiments for all agents. """
        # print("Planner spinning once.")
        for agent in self.agents:
            agent.curr_time = self.curr_time
            agent.spin_once()

            # Run one iteration of the individual method for DMA-RRT
            if agent.mode == "normal":
                self.dma_individual_normal(agent)
            elif agent.mode == "cooperative":
                self.dma_individual_coop(agent)
        
        self.curr_time += 1


def sol_individual(self, agent):
    """ Individual component of DMA-RRT as described in algorithm 4
        from Desaraju/How 2012.

        If this agent is the TokenHolder, it will update its internal
        plan and broadcast the next winner. Otw, it will broadcast its bid.
    """
    def multiagent_aware_time_realloc(path):
        """ Allocates more time to nodes in a path that conflict with other 
            agent nodes, s.t. the conflict is gone.

            Only tokenholders should use this, to prevent duplicate conflict
            resolution.

            Args:
                path: a Path object.
            Returns:
                a Path object similar to `path` but with collision-free
                    timestamps on each node.
        """
        for i in range(1,len(path.nodes)):
            # Check for collisions with any other node:
            for id, other_path in agent.other_agent_plans:
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

    # Refresh environment to reflect agents' current positions
    # agent.rrt.update_agent_plans(agent.other_agent_plans)
    # agent.rrt.update_pose(agent.antenna.uuid, agent.pos)

    # Grow the tree by one iteration
    agent.rrt.spin_once(False)

    # Find the new best path in the tree, and save it if it's better
    new_plan = agent.rrt.get_path()
    if new_plan:
        # Assign first "current path" found.
        if not agent.curr_plan.nodes:
            agent.curr_plan = new_plan

        if new_plan.cost < agent.best_plan.cost and \
                new_plan.cost < agent.curr_plan.cost:
            agent.best_plan = new_plan

    if agent.token_holder:
        # Replan to new best path if it's better
        if agent.best_plan.cost < agent.curr_plan.cost:
            agent.curr_plan = agent.best_plan
            agent.best_plan = Path()

        # Solve collisions with time reallocation
        agent.curr_plan = multiagent_aware_time_realloc(agent.curr_plan)

        # Broadcast the new winner of the bidding round
        winner_id = max(agent.bids, key = lambda x: agent.bids[x])
        agent.broadcast_waypoints(winner_id)
        agent.token_holder = False
    else:
        # Broadcast own bid
        bid = agent.curr_plan.cost - agent.best_plan.cost
        agent.broadcast_bid(bid)

def sol_coop_individual(self, agent):
    """ Individual component of Cooperative DMA-RRT as described
        in algorithm 6 from Desaraju/How 2012.
    """
    # # Grow tree one iteration while ignoring other agents' paths
    # agent.rrt.update_agent_plans(dict())
    # agent.rrt.update_pose(agent.pos)
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
