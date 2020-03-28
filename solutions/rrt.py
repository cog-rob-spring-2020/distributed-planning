"""
Cameron's RRT from 16.413 F19
"""

import functools
import math
from matplotlib import pyplot as plt
import numpy as np
import random
from scipy import spatial
from shapely.geometry import Point, Polygon, LineString, box
import yaml

from utils.environment import Environment, plot_environment, plot_line, plot_poly


def get_random_free_point(bounds, environment, radius, V):
    """
    Sample the free space and return a random point

    Params:
        bounds tuple (minx int, miny int, maxx int, maxy int)
        environment Environment
        radius float
        V list of tuples (x float, y float)
    Returns:
        tuple (x float, y float)
    """
    found = False
    rand_x = None
    rand_y = None

    # sample random points, make sure they don't fall within obstacles
    while not found:
        rand_x = random.uniform(bounds[0] + radius, bounds[2] - radius)
        rand_y = random.uniform(bounds[1] + radius, bounds[3] - radius)
        point = Point(rand_x, rand_y)
        # TODO: would a generator be faster here?
        if all([not obs.contains(point) for obs in environment.obstacles]):
            found = True

    return (rand_x, rand_y)


def distance(p1, p2, min_distance=0):
    """
    Return the Euclidean distance between points. min_distance is used to ignore values that are "too close"

    Params:
        p1 tuple (x float, y float)
        p2 tuple (x float, y float)
        min_distance float
    Returns:
        float
    """
    actual = math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)
    if actual >= min_distance:
        return actual
    else:
        return np.Infinity


def steer(p1, p2, steer_distance=0.2):
    """
    Move steer_distance from p1 in the direction of p2

    Params:
        p1 tuple (x float, y float)
        p2 tuple (x float, y float)
        steer_distance float
    Returns:
        tuple (x float, y float)
    """
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    scaling_factor = steer_distance / (distance(p1, p2))
    return (p1[0] + dx * scaling_factor, p1[1] + dy * scaling_factor)


def rrt(bounds, environment, start_pose, radius, end_region):
    """
    Apply the Rapidly Exploring Random Trees algorithm to find a path between two points. Renders a plot of the path with the number of nodes in the tree, number of nodes in the solution path, and path length

    Params:
        bounds tuple (minx int, miny int, maxx int, maxy int)
        environment Environment
        start_pose tuple (x int, y int)
        radius float
        end_region shapely.Polygon
    Returns:
        list of tuples (x float, y float)
    """
    V = [start_pose]
    E = []
    # assume the start position is not in the goal
    new_point = start_pose
    goal_bias_iterations = 20
    end_minx, end_miny, end_maxx, end_maxy = end_region.bounds

    obstaclesKDTree = spatial.KDTree(
        [[o.centroid.x, o.centroid.y] for o in environment.obstacles]
    )

    # find the nearest neighbor to a point
    nearest_neighbor = lambda point: min(
        V, key=functools.partial(distance, p2=point, min_distance=radius)
    )

    def get_nearest_neighbor(point):
        pointsKDTree = spatial.KDTree([[p[0], p[1]] for p in V])
        distances, elements = pointsKDTree.query([point[0], point[1]])
        return V[elements]

    # steer_distance is a set fraction of the distance across the board (using the average of the x and y axes)
    # steer_distance = ((bounds[2] - bounds[0] + bounds[3] - bounds[1]) / 2.0) / 20.0

    steer_distance = 0.5

    def nearest_obstacles(point):
        p = [point[0], point[1]]
        results = obstaclesKDTree.query_ball_point(p, 25)
        obstacles = [(environment.obstacles[r - 1]) for r in results]
        return obstacles

    i = 0
    while not end_region.contains(Point(new_point)):
        # random point will either just be random or somewhere in the goal
        if i % goal_bias_iterations == 0:
            # Getting a random point in a shapely Polygon: https://gis.stackexchange.com/a/207740
            random_point = (
                random.uniform(end_minx + radius, end_maxx - radius),
                random.uniform(end_miny + radius, end_maxy - radius),
            )
        else:
            rand_x = random.uniform(bounds[0] + radius, bounds[2] - radius)
            rand_y = random.uniform(bounds[1] + radius, bounds[3] - radius)
            random_point = (rand_x, rand_y)

        i += 1

        # steer towards the random point from the closest point to it to get a new point
        # nearest_point = nearest_neighbor(random_point)
        nearest_point = get_nearest_neighbor(random_point)
        new_point = steer(nearest_point, random_point, steer_distance)

        # expand the path from the nearest point to the new point into a line and test for collisions
        line = LineString([nearest_point, new_point])
        expanded_line = line.buffer(radius, resolution=3)

        if all([not expanded_line.intersects(o) for o in nearest_obstacles(new_point)]):
            # the path from nearest_point to new_point is obstacle free
            V.append(new_point)
            E.append((nearest_point, new_point))

    ax = plot_environment(environment, bounds=bounds)

    # iterate through E backwards to pull out the path to the start
    next_point = E[-1][0]
    path = [next_point, new_point]
    for e in reversed(E):
        # plot the edge while we're here
        plot_line(ax, LineString(list(e)))
        # check for a matching edge pair that led to the previous point
        # comparing float equality isn't reliable: https://stackoverflow.com/a/33024979
        if e[1] == next_point:
            next_point = e[0]
            path.insert(0, next_point)

    plot_poly(ax, end_region, "magenta")

    # plot the solution path
    for p in range(len(path)):
        if p == 0:
            # don't do anything with the first point
            continue
        line = LineString([path[p], path[p - 1]])
        extended_line = line.buffer(radius, resolution=3)
        plot_poly(ax, extended_line, "yellow", alpha=0.5)

    start_pose = Point(start_pose).buffer(radius, resolution=3)
    plot_poly(ax, start_pose, "green")

    ax.set_title(
        f"Number of nodes in tree: {len(V)} \n Number of nodes in solution path: {len(path)} \n Path length: {(len(path) - 1) * steer_distance}"
    )

    return path
