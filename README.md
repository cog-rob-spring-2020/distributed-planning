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

0. Make sure your [ROS environment is setup](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
1. Assuming you followed the default instructions for creating a ROS environment in `~/catkin_ws`, clone this repo to `~/catkin_ws/src/distributed-planning`
  1. `mkdir -p ~/catkin_ws/src`
  2. `cd ~/catkin_ws/src`
  3. `git clone git@github.mit.edu:cameronp/distributed-planning`
2. `cd ~/catkin_ws`
3. `catkin_make`
4. `source devel/setup.bash`

### Running

```sh
rosrun distributed-planning main ./environment.yaml
```
