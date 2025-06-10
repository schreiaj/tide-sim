"""Tests for the simulation node."""

from tide_sim import JointState

# TODO: Add tests for the simulation node
# These are hard to test because they require a running simulation


def test_joint_state_creation():
    """Test that joint states can be created correctly."""
    joint_state = JointState(
        name=["joint1", "joint2"],
        position=[0.0, 1.57],
        velocity=[0.0, 0.0],
        effort=[0.0, 0.0],
    )
    assert len(joint_state.name) == 2
    assert len(joint_state.position) == 2
    assert len(joint_state.velocity) == 2
    assert len(joint_state.effort) == 2
    assert joint_state.position[1] == 1.57
