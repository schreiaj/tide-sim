# Tide Simulation

A Python package for adding a simulation environment to a tide robot using PyBullet. This package provides a simulation node that can be used to simulate robot behavior in a physics environment.

## Features

- PyBullet-based physics simulation
- Real-time robot joint control
- Integration with tide robot framework
- Support for custom URDF models
- Configurable simulation parameters

## Installation

This project uses `uv` for dependency management. To get started:

1. Install `uv` if you haven't already:
```bash
pip install uv
```

2. Create a virtual environment and install dependencies:
```bash
uv venv
source .venv/bin/activate  # On Unix/macOS
# or
.venv\Scripts\activate  # On Windows

uv pip install -e ".[dev]"
```

## Usage

The package provides a `SimulationNode` class that can be used to simulate robot behavior:

```python
from tide_sim import SimulationNode

# Create a simulation node with custom configuration
config = {
    "robot_id": "my_robot",
    "headless": False,  # Set to True for headless simulation
    "sim_rate": 240,    # Physics simulation rate
    "update_rate": 1,   # State update rate
    "urdf_path": "path/to/your/robot.urdf"
}

sim_node = SimulationNode(config=config)
sim_node.start()
```

### Configuration Options

- `robot_id`: Identifier for the robot (default: "simbot")
- `headless`: Run simulation without GUI (default: False)
- `sim_rate`: Physics simulation rate in Hz (default: 240)
- `update_rate`: State update rate in Hz (default: 1)
- `urdf_path`: Path to the robot's URDF file (default: "assets/simulation/robot.urdf")

### Joint Control

The simulation node subscribes to joint state commands and applies them to the simulated robot:

```python
from tide_sim import JointState

# Create a joint state command
joint_state = JointState(
    name=["joint1", "joint2"],
    position=[0.0, 1.57],  # in radians
    velocity=[0.0, 0.0],   # in rad/s
    effort=[0.0, 0.0]      # in N⋅m
)

# The simulation node will automatically apply these joint states
```

## Development

The project uses several development tools:
- `black` for code formatting
- `isort` for import sorting
- `ruff` for linting
- `mypy` for type checking
- `pytest` for testing

To run the development tools:
```bash
black .
isort .
ruff check .
mypy .
pytest
```

## Dependencies

- `pybullet>=3.2.7`: Physics simulation engine
- `tide-sdk>=0.1.5`: Tide robot framework integration

## License

MIT License 