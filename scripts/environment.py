"""
Borrowed from 16.413 F19 pset 6
"""
import yaml
import math

import shapely.geometry as geom
from shapely import affinity
import itertools
from matplotlib import pyplot as plt
from descartes import PolygonPatch


def plot_environment(env, bounds=None, figsize=None):
    if bounds is None and env.bounds:
        minx, miny, maxx, maxy = env.bounds
    elif bounds:
        minx, miny, maxx, maxy = bounds
    else:
        minx, miny, maxx, maxy = (-10, -5, 10, 5)

    max_width, max_height = 12, 5.5
    if figsize is None:
        width, height = max_width, (maxy-miny)*max_width/(maxx-minx)
        if height > 5:
            width, height = (maxx-minx)*max_height/(maxy-miny), max_height
        figsize = (width, height)
    f = plt.figure(figsize=figsize)
    ax = f.add_subplot(111)
    for i, obs in enumerate(env.obstacles):
        patch = PolygonPatch(obs, fc='blue', ec='blue', alpha=0.5, zorder=20)
        ax.add_patch(patch)

    plt.xlim([minx, maxx])
    plt.ylim([miny, maxy])
    ax.set_aspect('equal', adjustable='box')
    return ax


def plot_line(ax, line, color):
    x, y = line.xy
    ax.plot(x, y, color=color, linewidth=3, solid_capstyle='round', zorder=1)


def plot_poly(ax, poly, color, alpha=1.0, zorder=1):
    patch = PolygonPatch(poly, fc=color, ec="black",
                         alpha=alpha, zorder=zorder)
    ax.add_patch(patch)


class Environment:
    def __init__(self, yaml_file=None, bounds=None):
        self.yaml_file = yaml_file
        self.environment_loaded = False
        self.obstacles = []
        self.obstacles_map = {}
        self.bounds = bounds
        if not yaml_file is None:
            if self.load_from_yaml_file(yaml_file):
                if bounds is None:
                    self.calculate_scene_dimensions()
                self.environment_loaded = True

    def add_obstacles(self, obstacles):
        self.obstacles = self.obstacles + obstacles
        self.calculate_scene_dimensions()

    def calculate_scene_dimensions(self):
        """Compute scene bounds from obstacles."""
        points = []
        for elem in self.obstacles:
            points = points + list(elem.boundary.coords)

        mp = geom.MultiPoint(points)
        self.bounds = mp.bounds

    def collision_free(self, x, y):
        """ Check if a point is obstacle-free. """
        point = geom.Point(x, y)
        return not any([obstacle.contains(point) for obstacle in self.obstacles])

    def load_from_yaml_file(self, yaml_file):
        f = open(yaml_file)
        self.data = yaml.safe_load(f)
        f.close()
        return self.parse_yaml_data(self.data)

    def parse_yaml_data(self, data):
        if 'environment' in data:
            env = data['environment']
            self.parse_yaml_obstacles(env['obstacles'])
            return True
        else:
            return False

    def parse_yaml_obstacles(self, obstacles):
        self.obstacles = []
        self.obstacles_map = {}
        for name, description in obstacles.items():
            # Double underscore not allowed in region names.
            if name.find("__") != -1:
                raise Exception("Names cannot contain double underscores.")
            if description['shape'] == 'rectangle':
                parsed = self.parse_rectangle(name, description)
            elif description['shape'] == 'polygon':
                parsed = self.parse_polygon(name, description)
            else:
                raise Exception("not a rectangle")
            if not parsed.is_valid:
                raise Exception("%s is not valid!" % name)
            self.obstacles.append(parsed)
            self.obstacles_map[name] = parsed
        self.expanded_obstacles = [obs.buffer(
            0.75/2, resolution=2) for obs in self.obstacles]

    def parse_rectangle(self, name, description):
        center = description['center']
        center = geom.Point((center[0], center[1]))
        length = description['length']
        width = description['width']
        # convert rotation to radians
        rotation = description['rotation']  # * math.pi/180
        # figure out the four corners.
        corners = [(center.x - length/2., center.y - width/2.),
                   (center.x + length/2., center.y - width/2.),
                   (center.x + length/2., center.y + width/2.),
                   (center.x - length/2., center.y + width/2.)]
        # print corners
        polygon = geom.Polygon(corners)
        out = affinity.rotate(polygon, rotation, origin=center)
        out.name = name
        out.cc_length = length
        out.cc_width = width
        out.cc_rotation = rotation
        return out

    def parse_polygon(self, name, description):
        _points = description['corners']
        for points in itertools.permutations(_points):
            polygon = geom.Polygon(points)
            polygon.name = name
            if polygon.is_valid:
                return polygon

    def save_to_yaml(self, yaml_file):
        yaml_dict = {}
        obstacles = {}
        for i, ob in enumerate(self.obstacles):
            ob_dict = {}
            ob_dict['shape'] = 'polygon'
            ob_dict['corners'] = [list(t) for t in list(ob.boundary.coords)]
            ob_name = "obstacle%.4d" % i
            obstacles[ob_name] = ob_dict
        yaml_dict['environment'] = {'obstacles': obstacles}

        f = open(yaml_file, 'w')
        f.write(yaml.dump(yaml_dict, default_flow_style=None))
        f.close()


def random_environment(bounds, start, radius, goal, n, size_limits=(0.5, 1.5)):
    """
    Create a random environment of obstacles
    """
    minx, miny, maxx, maxy = bounds
    edges = 4
    minl, maxl = size_limits
    env = Environment(None)
    obs = []
    start_pose = Point(start).buffer(radius, resolution=3)
    obi = 0
    while obi < n:
        r = np.random.uniform(low=0.0, high=1.0, size=2)
        xy = np.array([minx + (maxx - minx) * r[0],
                       miny + (maxy - miny) * r[1]])

        angles = np.random.rand(edges)
        angles = angles * 2 * np.pi / np.sum(angles)
        for i in range(1, len(angles)):
            angles[i] = angles[i - 1] + angles[i]
        angles = 2 * np.pi * angles / angles[-1]
        angles = angles + 2 * np.pi * np.random.rand()
        lengths = 0.5 * minl + (maxl - minl) * 0.5 * np.random.rand(edges)
        xx = xy[0] + np.array([l * np.cos(a) for a, l in zip(angles, lengths)])
        yy = xy[1] + np.array([l * np.sin(a) for a, l in zip(angles, lengths)])
        p = Polygon([(x, y) for x, y in zip(xx, yy)])
        if p.intersects(start_pose) or p.intersects(goal):
            continue
        else:
            obi = obi + 1
            obs.append(p)
    env.add_obstacles(obs)
    return env
