from shapely.geometry import Polygon
from solutions.rrt import rrt
from utils.check_path import check_path
from utils.environment import Environment


def test_generated_paths():
    print("testing test_generated_paths...\n")

    # set up a known feasible environment
    environment = Environment(yaml_file="utils/simple.yaml")

    radius = 0.3
    bounds = (-2, -3, 12, 8)
    start = (0, 0)
    goal_region = Polygon([(10, 5), (10, 6), (11, 6), (11, 5)])

    for i in range(10):
        path = rrt(bounds, environment, start, radius, goal_region)
        check_path(path, bounds, environment, start, radius, goal_region, True)
        # FYI check_path doesn't check for collisions, just that the path is valid
