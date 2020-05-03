# Distributed Planning Assignment and Grand Challenge

16.412 final project

## Assignment

The student-facing Jupyter notebook and its solutions live in `./assignment.` See `./assignment/README.md` for more information.

## Grand Challenge

### Development

See [instructions](https://docs.google.com/document/d/1oX_eJmV-vMKZSr4hDh7DyUTJEKLYcL7JspLI2c2MxHc/edit#heading=h.la9ejcobwj1r) on accessing the VM

#### Dependencies

* [ROS Kinetic](http://wiki.ros.org/kinetic) on [Ubuntu 16.04](http://releases.ubuntu.com/16.04/)
* Python 2.7, which should be installed as `python` by default on Ubuntu 16.04
  * The class VM already has `numpy` and `matplotlib` installed

### Installation

You need the following dependencies:

```sh
sudo apt-get install python-wstool python-catkin-tools ros-kinetic-map-server
```

Make sure your [ROS environment is setup](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)

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

### Running

We recommend using `roslaunch`. For example:

```sh
roslaunch distributed_planning dma-rrt.launch
```

To visualize, we currently use RVIZ:

```sh
roscd distributed_planning
rviz -d rviz/config.rviz
```
