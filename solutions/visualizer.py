from matplotlib import pyplot as plt
from IPython.display import clear_output
from shapely.geometry import Point, Polygon, LineString

from rrtstar import RRTstar

from utils.environment import Environment, plot_environment, plot_line, \
    plot_poly

class Visualizer:
    def __init__(self, env):
        self.env = env
        self.finished_first_viz = False

    def plot_env(self):
        """ Plot the environment, including static obstacles but excluding 
            agents and RRT trees.

            Returns:
                ax: the plot object
        """
        ax = plot_environment(self.env)
        ax.set_title("Multiagent Planner")
        return ax

    def plot_tree(self, rrt):
        """ Plot RRT tree, final path, and start/goal nodes. 

            Args:
                ax: a Plot axis object on which to plot the tree.
                rrt: an initialized RRTstar object.
        """
        ax = plot_environment(rrt.env)
        plot_poly(ax, Point(
            (rrt.start.x, rrt.start.y)).buffer(0.3, resolution=3), "green")
        plot_poly(ax, Point(
            (rrt.goal.x, rrt.goal.y)).buffer(0.3, resolution=3), "red")

        for node in rrt.node_list:
            if node.parent is not None:
                plot_line(ax, LineString([[node.x, node.y],
                                          [node.parent.x, node.parent.y]]))

        path = rrt.get_path()
        if path is not None:
            path_xy = [[node.x, node.y] for node in path.nodes]
            line = LineString(path_xy)
            extended_line = line.buffer(0.2, resolution=3)
            plot_poly(ax, extended_line, "yellow", alpha=0.5)

            ax.set_title(f"Number of nodes in tree: {len(rrt.node_list)} \n \
                        Number of nodes in solution path: {len(path.nodes)} \
                        \n Path length: {path.cost}")

        else:
            ax.set_title(f"Number of nodes in tree: {len(rrt.node_list)}")
            # print("No path was found.")

    def plot_path(self, ax, path, id):
        """ Plot a path and label by id on a given axis object.

            Args:
                ax: a Plot axis object, previously plotted with environment.
                path: a Path object
                id: a unique identifier corresponding to the agent to which
                    the path belongs.
        """
        if path.nodes:
            plot_poly(ax, Point(
                (path.start_pose.x,
                path.start_pose.y)).buffer(0.3, resolution=3), "green")

            plot_poly(ax, Point(
                (path.goal_pose.x,
                path.goal_pose.y)).buffer(0.3, resolution=3), "red")

            if path is not None:
                path_xy = [[node.x, node.y] for node in path.nodes]
                line = LineString(path_xy)
                plot_line(ax, line, "yellow")

            # TODO(marcus): add label for path as id

    def spin(self, rate):
        """ Visualize all agents current position and best paths.

            Args:
                rate: the spin rate in Hz.
        """
        # TODO(marcus): this will be done using callbacks and on its own thread!
        raise NotImplementedError

    def spin_once(self, paths):
        """ Visualize all agents' current position and bets paths.

            Args:
                paths: a dictionary of path objects keyed by agent ID.
        """
        clear_output(wait=True)
        ax = self.plot_env()
        for id, path in paths.items():
            self.plot_path(ax, path, id)

    def spin_once_tree(self, rrt):
        """
        """
        clear_output(wait=True)
        self.plot_tree(rrt)
