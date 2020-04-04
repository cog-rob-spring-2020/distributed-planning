"""
Closed-Loop RRT (CL-RRT) from (Kuwata 2009) and (Desaraju 2012)
"""
import networkx as nx
from scipy import spatial
from shapely.geometry import Point
import time
from solutions.rrt import get_random_free_point, distance


def get_nearest_neighbor(point, tree):
    """
    Get the closest existing point in the tree

    Params:
        point tuple (float, float)
        tree networkx.DiGraph
    """
    pointsKDTree = spatial.KDTree([[p[0], p[1]] for p in tree.nodes])
    distances, elements = pointsKDTree.query([point[0], point[1]])
    return tree[elements]


def get_nearest_obstacles(point, obstacles, environment):
    """
    Get the closest obstacles to a point
    """
    p = [point[0], point[1]]
    # time saving trick to avoid checking faraway obstacles for collisions
    results = obstacles.query_ball_point(p, 25)
    obstacles = [(environment.obstacles[r - 1]) for r in results]
    return obstacles


def tree_expansion(tree, bounds, environment, obstaclesKDTree, radius, current_time):
    # make a copy?
    new_tree = tree

    # select a random point from free space
    x_sample = get_random_free_point(bounds, environment, radius)

    # find the nearest neighbor to the point
    n_near = get_nearest_neighbor(x_sample, V)

    k = 0
    last_state = x_hat(current_time + k)

    while last_state in free_space(current_time + k) and last_state not in x_sample:
        pass

    return new_tree


def lazy_check():
    pass


def best_path(tree):
    p_star = []
    return p_star


def apply(agent, p_star):
    # TODO: probably set something on the agent? maybe pass control inputs to the agent?
    pass


def CL_RRT(
    agent, bounds, environment, start_state, end_region, radius=0.1, delta_t=0.1
):
    """
    Main execution loop of the CL-RRT algorithm

    Params:
        delta_t float seconds
    """
    # use the weights of the tree as cost estimates
    tree = nx.DiGraph()
    tree.add_node(start_state)

    current_time = 0

    obstaclesKDTree = spatial.KDTree(
        [[o.centroid.x, o.centroid.y] for o in environment.obstacles]
    )

    current_state = start_state
    while not end_region.contains(Point(current_state)):
        elapsed_time = 0
        while elapsed_time <= delta_t:
            # can be slowed down with a time.sleep()
            loop_start = time.perf_counter()
            tree = tree_expansion(
                tree, bounds, environment, obstaclesKDTree, radius, current_time
            )
            loop_end = time.perf_counter()
            elapsed_time += loop_end - loop_start

        while len(tree.nodes) > 0:
            p_star = best_path(tree)

            if lazy_check(p_star):
                current_state = apply(agent, p_star)
                tree.add_node(current_state)
                break
            else:
                # remove the infeasible path
                # TODO: check format here - p_star should be a list of nodes
                tree = tree.remove_nodes_from(p_star)

        t += delta_t
