"""
Closed-Loop RRT (CL-RRT) from (Kuwata 2009) and (Desaraju 2012)
"""
from scipy import spatial
from rrt import get_random_free_point

def get_nearest_neighbor(point):
    pointsKDTree = spatial.KDTree([[p[0], p[1]] for p in V])
    distances, elements = pointsKDTree.query([point[0], point[1]])
    return V[elements]

def tree_expansion(bounds, environment, radius, V, current_time, k):
    # select a random point from free space
    x_sample = get_random_free_point(bounds, environment, radius, V)

    # find the nearest neighbor to a point
    n_near = lambda point: min(
        V, key=functools.partial(distance, p2=point, min_distance=radius)
    )

    k = 0
    last_state = x_hat(current_time + k)

    while last_state in free_space(current_time + k) and last_state not in x_sample

def lazy_check():
    pass

def CL_RRT(bounds, environment, start_position, end_position, radius=0.1, delta_t=0.1):
    """
    Main execution loop of the CL-RRT algorithm
    """
    tree = [start_position]
    E = []
    current_time = 0
    pass
