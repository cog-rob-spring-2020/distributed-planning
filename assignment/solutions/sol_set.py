import random
from assignment.plan import Plan

def sol_dma_individual(self, agent):
    """ Individual component of DMA-RRT as described in algorithm 4
        from Desaraju/How 2012.

        If this agent is the TokenHolder, it will update its internal
        plan and broadcast the next winner. Otw, it will broadcast its bid.
    """
    # Refresh environment to reflect agent's current positions
    agent.rrt.update_pos(agent.pos, self.curr_time, wipe_tree=True)

    # Grow the tree by one set of iterations
    agent.rrt.spin(False)

    # Find the new best path in the tree
    new_plan = agent.rrt.get_path()

    # Assign first "current path" found
    if not agent.curr_plan.nodes:
        agent.curr_plan = new_plan
    agent.best_plan = new_plan

    if agent.token_holder:
        # Replan to new best path
        agent.curr_plan = agent.best_plan

        # Solve collisions with time reallocation
        agent.curr_plan = Plan.multiagent_aware_time_realloc(agent.curr_plan,
                agent.other_agent_plans)

        # Broadcast the new winner of the bidding round
        if agent.bids:
            winner_id = compute_winner(agent)
            agent.broadcast_waypoints(winner_id)
    else:
        bid(agent)

    # Prevent agent from getting the token if they finish.
    if agent.at_goal():
        agent.broadcast_bid(-1000.0)

def sol_compute_winner(agent):
    """ Returns the id of the agent with the highest PPI (potential path
        improvement) bid, based on the bid information in the agent's
        `bids` dictionary.

        Assumes the agent's `bids` dictionary contains at least one bid.
    """
    agent.token_holder = False
    winner_bid = max(agent.bids.values())
    winner_ids = [id for id, bid in agent.bids.items() \
        if bid == winner_bid]
    winner_id = random.choice(winner_ids)

    return winner_id

def sol_bid(agent):
    """ Calculates and broadcasts the agent's bid of its 
        potential path improvement (PPI).
        
        Recall that the PPI is defined to be the difference between the 
        agent's current path and the best path returned by its RRT planner,
        representing how much better of a path the agent could have if
        given the opportunity to replan.
    """
    # Broadcast own bid
    # We use abs in the event that we are comparing an incomplete path
    #     to a complete path. Incomplete paths will usually have smaller
    #     cost; see Path definition for more info.
    agent.broadcast_bid(agent.curr_plan.cost - agent.best_plan.cost)
