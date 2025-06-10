# Tide Simulation

A Python package for adding a simulation environment to a tide robot using PyBullet. This package provides a simulation node that can be used to simulate robot behavior in a physics environment.

## Features

- PyBullet-based physics simulation
- Real-time robot joint control
- Integration with tide robot framework
- Support for custom URDF models
- Configurable simulation parameters

## Installation

### From GitHub

You can install the package directly from GitHub using pip:

```bash
pip install git+https://github.com/schreiaj/tide-sim.git
```

Or with uv:

```bash
uv pip install git+https://github.com/schreiaj/tide-sim.git
```


## Usage

The package provides a `SimulationNode` class that can be used to simulate robot behavior:

It can be used like any other node in your tide project

```
- type: tide_sim.SimulationNode
    params:
      robot_id: "simbot"
      sim_rate: 60
      update_rate: 20
```
### Configuration Options

- `robot_id`: Identifier for the robot (default: "simbot")
- `headless`: Run simulation without GUI (default: False)
- `sim_rate`: Physics simulation rate in Hz (default: 240)
- `update_rate`: Rate at which pose updates will be published
- `urdf_path`: Path to the robot's URDF file (default: "assets/simulation/robot.urdf")

### Joint Control

The simulation node subscribes to joint state commands and applies them to the simulated robot:

It uses the joint names from the URDF file. If positions are specified it will use a position controller with velocity and effort constraints. If no position but a velocity it will use a velocity controller with effort constraints. If no position or velocity it will use a torque controller. 

It is criticl that all arrays are the same length. This may be enforced in the future but for now it will just cause runtime errors. 

```python
from tide_sim import JointState

# Create a joint state command
joint_state = JointState(
    name=["joint1", "joint2"],
    position=[0.0, 1.57],  # in radians
    velocity=[],   # in rad/s
    effort=[]      # in N⋅m
)

# The simulation node will automatically apply these joint states
```

### Development 

For development, clone the repository and install in editable mode:

```bash
# Clone the repository
git clone https://github.com/schreiaj/tide-sim.git
cd tide-sim

# Create virtual environment and install dependencies
make install

# Install pre-commit hooks (optional but recommended)
pip install pre-commit
pre-commit install
```


The project uses several development tools:
- `black` for code formatting
- `isort` for import sorting
- `ruff` for linting
- `mypy` for type checking
- `pytest` for testing

To run the development tools:
```bash
make test     # Run tests
make lint     # Run linters
make format   # Format code
make clean    # Clean up build artifacts
```

## Dependencies

- `pybullet>=3.2.7`: Physics simulation engine
- `tide-sdk>=0.1.5`: Tide robot framework integration

## License

MIT License 