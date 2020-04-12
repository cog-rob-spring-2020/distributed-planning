from matplotlib import pyplot as plt
from IPython.display import clear_output
from shapely.geometry import Point, Polygon, LineString

from solutions.rrtstar import RRTstar

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

    def plot_tree(self, ax, rrt):
        """ Plot RRT tree

            Args:
                ax: a Plot axis object on which to plot the tree.
                rrt: an initialized RRTstar object.
        """
        for node in rrt.node_list:
            if node.parent is not None:
                plot_line(ax,
                          LineString([[node.x, node.y],
                                     [node.parent.x, node.parent.y]]),
                          "orange")

    def plot_path(self, ax, path, id, color):
        """ Plot a path and label by id on a given axis object.

            Args:
                ax: a Plot axis object, previously plotted with environment.
                path: a Path object
                id: a unique identifier corresponding to the agent to which
                    the path belongs.
        """
        if path.nodes:
            plot_poly(ax, Point(
                (path.nodes[0].x,
                path.nodes[0].y)).buffer(0.3, resolution=3), "green")

            plot_poly(ax, Point(
                (path.goal_node.x,
                path.goal_node.y)).buffer(0.3, resolution=3), "red")

            if path is not None:
                path_xy = [[node.x, node.y] for node in path.nodes]
                line = LineString(path_xy)
                plot_line(ax, line, color)

            # TODO(marcus): add label for path as id

    def plot_current_pos(self, ax, pos, id):
        """ Plot the x-y position of an agent with id `id`. """
        plot_poly(ax, Point(pos[0], pos[1]).buffer(0.2, resolution=3), "blue")
        # TODO(marcus): add label for point as id

    def spin(self, rate):
        """ Visualize all agents current position and best paths.

            Args:
                rate: the spin rate in Hz.
        """
        # TODO(marcus): this will be done using callbacks and on its own thread!
        raise NotImplementedError

    def spin_once(self, agents, viz_tree=False):
        """ Visualize all agents' current position and current/best paths.

            Args:
                agents: a list of agent objects.
        """
        clear_output(wait=True)
        ax = self.plot_env()
        for agent in agents:
            self.plot_path(ax, agent.best_plan, agent.antenna.uuid, "yellow")
            self.plot_path(ax, agent.curr_plan, agent.antenna.uuid, "gray")
            self.plot_current_pos(ax, agent.pos, agent.antenna.uuid)

            if viz_tree:
                self.plot_tree(ax, agent.rrt)
