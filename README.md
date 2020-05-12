# Distributed Planning Assignment and Grand Challenge

16.412 final project

## Assignment

The student-facing Jupyter notebook for the Advanced Lecture/mini pset assignment, as well as its solutions, live in `./assignment.` See `./assignment/README.md` for more information.

## Grand Challenge

### Features and Functionality

This repository includes implementations of RRT\*, DMA-RRT, and unique extensions of the original DMA-RRT algorithm to better perform distributed goal selection. The system is comprised of discrete agents and auxiliary nodes. Agents represent robots in the simulation and are self-contained.

The agents can be found in the [source directory](/src/distributed_planning). For the basic DMA-RRT agent (which is the base class of all agent types), see [agent_dmarrt.py](/src/distributed_planning/agent_dmarrt.py). Agents perform their core functions in the `spin_once()` method, which is run on a fixed timer.

In the [agent_params](params/agent_params.yaml) file, you may specify the parameters associated with each agent in the simulation. This includes RRT\* initialization parameters, spin rates, and starting/goal positions.

There are several provided maps for visualization in the [maps](/maps) directory. You may use any of these by changing the `map` argument in the launch files.

The launch files in the [launch](/launch) directory are the main point of entry for running simulations. See more information in the Execution subsection below.

### System Architecture

Each simulation requires a map, which is a shared environment in which the agents operate, and several agents. You can have as many agents as you want, but note that if running the system on one computer there will be heavy computation loads with large numbers of agents.

Agents can be heterogeneous or homogeneous. This is reflected both in their parameters, which can vary easily, and in which classes of agents are used.

Finally, some of the simulations will require auxiliary nodes such as goal-updaters. These serve mainly to inject new goal points into the system for the agents to add to their queues and/or claim. 

### Development

See [instructions](https://docs.google.com/document/d/1oX_eJmV-vMKZSr4hDh7DyUTJEKLYcL7JspLI2c2MxHc/edit#heading=h.la9ejcobwj1r) on accessing the VM.

#### Dependencies

* [ROS Kinetic](http://wiki.ros.org/kinetic) on [Ubuntu 16.04](http://releases.ubuntu.com/16.04/)
* Python 2.7, which should be installed as `python` by default on Ubuntu 16.04
  * The class VM already has `numpy` and `matplotlib` installed

### Installation

You need the following dependencies:

```sh
sudo apt-get install python-wstool python-catkin-tools ros-kinetic-map-server
```

Make sure your [ROS environment is set up](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).

```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone git@github.mit.edu:cameronp/distributed-planning

cd ~/catkin_ws
catkin init
cd src

wstool init
wstool merge distributed-planning/install/distributed_planning.rosinstall
wstool update

cd ../
catkin build
source devel/setup.bash
```

Once your environment is set up, you should still do the following before pushing/merging anything, to make sure the build isn't broken:
```sh
catkin clean -y
catkin build
source devel/setup.bash
```

### Execution

We recommend using `roslaunch`. We have a launch file in the launch folder for each successful variation of agent, listed below. The altruistic agents were never fully debugged, so we do not recommend running that launch file.

```sh
roslaunch distributed_planning basic_agents.launch
roslaunch distributed_planning continuation_agents.launch
roslaunch distributed_planning euclidean_agents.launch
roslaunch distributed_planning reward_agents.launch
```

To visualize, use the provided RVIZ config:

```sh
roscd distributed_planning
rviz -d rviz/config.rviz
```
