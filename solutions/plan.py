#!/usr/bin/env python3

# from agent import Agent

class Plan():
    def __init__(self, agents, dma_indiv, dma_coop):
        """ Plan object runs multiagent experiments with DMA-RRT.

            Args:
                agents: a list containing Agent objects
                dma_indiv: a method representing the individual component
                    of the DMA-RRT algorithm.
                dma_coop: a method representing the cooperative extension
                    of the DMA-RRT algorithm.
        """
        self.agents = agents
        Plan.dma_indiv = dma_indiv
        Plan.dma_coop = dma_coop

        # Do multithreading setup here.
    
    def dma_indiv(self):
        raise NotImplementedError

    def dma_coop(self):
        raise NotImplementedError

    def spin(self):
        """ Run DMA-RRT experiments for all agents. """
        for agent in self.agents:
            agent.spin()

if __name__ == "__main__":
    # Test of system architecture:
    def indiv(self):
        print('indiv')

    def coop(self):
        print('collab')

    plan = Plan([], indiv, coop)
    plan.dma_indiv()
    plan.dma_coop()
    print(plan.dma_indiv)
    print(plan.dma_coop)
