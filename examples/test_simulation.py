#!/usr/bin/env python3
"""
Test script for the tide simulation package.
This script creates a simple simulation with a two-joint robot and moves the joints.
"""

import math
import os
import time

from tide_sim import JointState, SimulationNode


def main():
    # Get the path to the URDF file relative to this script
    current_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(current_dir, "assets", "simulation", "robot.urdf")

    # Create simulation node with custom configuration
    config = {"urdf_path": urdf_path}
    sim_node = SimulationNode(config=config)

    try:
        # Start the simulation
        sim_node.start()

        # Wait a moment for the simulation to initialize
        time.sleep(2)

        # Create a joint state command
        joint_state = JointState(
            name=["joint1", "joint2"],
            position=[0.0, 0.0],  # Start at zero position
            velocity=[0.0, 0.0],
            effort=[0.0, 0.0],
        )

        # Move joints in a simple pattern
        for i in range(10):
            # Calculate new positions
            angle = math.sin(i * 0.5) * math.pi / 2  # Oscillate between -π/2 and π/2
            joint_state.position = [
                angle,
                angle * 0.5,
            ]  # Second joint moves half as much

            # Send the command
            sim_node._on_joint_state(joint_state.model_dump())

            # Wait for the movement to complete
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    finally:
        # Clean up
        sim_node.cleanup()


if __name__ == "__main__":
    main()
