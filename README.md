# Distributed Planning

16.412 final project

---

## Proposed File Structure

```
/
|-- solutions/ - instructor-facing algo solutions
|-- pset/      - student-facing pset code
|-- tests/     - instructor-facing tests
|-- utils/     - shared code
```

### `utils/`

The `Environment` class defines a 2D space with obstacles. It can be used to model a simple area with binary accessible and inaccessible regions. For example usage, spin up a Jupyter notebook (see below) and check out `solutions/rrt_example.ipynb`.

## Development

* Python 3.8

For sanity's sake, let's use 4 spaces. If you use VS Code, this repo includes settings that will automatically set tabs to 4 spaces.

All Python dependencies are in `requirements.txt`. We recommend using a [virtual environment](https://docs.python.org/3/library/venv.html) (venv) to manage dependencies.

If you're on Linux/Unix, here's a quick guide. If you're on Windows, see the Python [documentation](https://docs.python.org/3/library/venv.html).
1. `python -m venv .venv` (assuming `python` is version 3.8. You may need to adjust depending on how Python is installed on your system)
2. `. .venv/bin/activate`
3. `pip install -r requirements.txt`


### API Breakdown

**TODO(marcus):** remove this subsection for final release!

To aid our devleopment, here is a breakdwon of the external-facing APIs of each component in the module and how they interact with each other.

[rrtstar.py](/solutions/rrtstar.py): Implementation of RRT* planner.
- `RRTstar.spin()` This will run the main planning loop for a fixed number of iterations (determined in the ctor of the RRTstar object). Can optionally terminate at first path found.
- `RRTstar.spin_once()` This will run one iteration of the `spin` execution loop. Useful for external objects; the agent will have its own execution loop running on some rate, and will call this method to update the tree-expansion and planning algorithm. This allows the agent to inject new information about the environment (new obstacles, new goal position) into the planner before replanning.
- `RRTstar.get_path()` External objects interfacing with `RRTstar` will call this to check for the current best path from `RRTstar.start` to `RRTstar.goal`. This can return `None` if no path is found.

[agent.py](/solutions/agent.py): Structure for a vehicle agent which plans paths in an Environment.
- `Agent.spin()` Should run the planner at a fixed rate (calling `RRTstar.spin_once()`) and optionally run the `Agent.individual()` or `Agent.coop_individual()` methods to update the internal state wrt other agents. The networked callbacks will modify each agent's internal state and this will be used to update the planner in this block, when the agent accesses these internal state variables.
- `Agent.spin_once()` should run one iteration of the spin function; basically `spin()` should only call this on a timer. This allows us to slow things down in the wrapper code.
- `Agent.broadcast_waypoints()` and `Agent.broadcast_bids()` will be called after planning to send updates on the network. The subscriber system on the client side will just update internal state variables for each agent.

[plan.py](/solutions/plan.py): Main loop for larger experiments.
- This one currently has structures to represent Path (and presumably other things). 
- I think we should change this to be a sort of `main.py`, where the script for the full experiment is. It should instantiate the different agents, which in turn should instantiate their internal planners. Maybe it reads in some representation of vehicle dynamics from a yaml file (or maybe not for this pset).
- It should have a `plan()` method that we call from the notebook to start the experiment
- The `plan()` method should call each Agent's `spin()` in parallel (spin up one thread per agent).
- There should be external callbacks for visualization or other tasks that happen on-demand or on a timer.


## Jupyter Notebook Usage

```sh
PYTHONPATH="$PWD" jupyter notebook
```

When it loads, make sure you're using the kernel from your venv.

### Testing

With your venv activated:

```sh
PYTHONPATH="$PWD" nose2 --pretty-assert
```

### Installing New Packages

Let's keep `requirements.txt` up to date with all required packages. With your venv activated:

```sh
# make sure you've installed everything first or the next step will blow away
# packages you haven't installed
pip install -r requirements.txt

# actually install the new package (`foo` in this case) and then update
# the requirements
pip install foo && pip freeze > requirements.txt
```
