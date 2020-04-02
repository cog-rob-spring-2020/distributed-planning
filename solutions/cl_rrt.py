"""
Closed-Loop RRT (CL-RRT) from (Kuwata 2009) and (Desaraju 2012)
"""
from scipy import spatial
from shapely.geometry import Point
import time
from rrt import get_random_free_point

def get_nearest_neighbor(point, tree):
    pointsKDTree = spatial.KDTree([[p[0], p[1]] for p in tree])
    distances, elements = pointsKDTree.query([point[0], point[1]])
    return V[elements]

def tree_expansion(bounds, environment, radius, tree, current_time):
    new_tree = tree

    # select a random point from free space
    x_sample = get_random_free_point(bounds, environment, radius, V)

    # find the nearest neighbor to the point
    n_near = lambda point: min(
        V, key=functools.partial(distance, p2=point, min_distance=radius)
    )

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

def CL_RRT(agent, bounds, environment, start_state, end_region, radius=0.1, delta_t=0.1):
    """
    Main execution loop of the CL-RRT algorithm

    Params:
        delta_t float seconds
    """
    tree = [start_state]
    E = []
    current_time = 0

    obstaclesKDTree = spatial.KDTree(
        [[o.centroid.x, o.centroid.y] for o in environment.obstacles]
    )

    current_state = start_state
    while not end_region.contains(Point(current_state)):
        new_state = current_state

        elapsed_time = 0
        while elapsed_time <= delta_t:
            # can be slowed down with a time.sleep()
            loop_start = time.perf_counter()
            tree = tree_expansion(bounds, environment, radius, tree, current_time)
            loop_end = time.perf_counter()
            elapsed_time += loop_end - loop_start

        while len(tree) > 0:
            p_star = best_path(tree)

            if lazy_check(p_star):
                apply(agent, p_star)
                return
            else:
                # remove the infeasible path
                tree = tree.pop(p_star)

    t += delta_t
