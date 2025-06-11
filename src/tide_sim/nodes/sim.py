import math
import os
import signal
import threading
import time

import pybullet as p
import pybullet_data
from tide.core.node import BaseNode

from tide_sim.models import JointState


class SimulationNode(BaseNode):

    ROBOT_ID = "simbot"
    GROUP = "simulation"

    _urdf_path = "assets/simulation/robot.urdf"
    _headless = False
    _update_rate = 1
    _sim_rate = 240

    _physics_client = None
    _state_client = None
    _running = False
    _threads = []
    _robot_id = None
    _joint_indices = {}

    # PD control gains
    _max_force = 2.0  # Maximum force the motor can apply

    # A queue of joint commands to execute in the next step,
    # these will be applied in the order they are received
    _joint_commands = []
    

    def __init__(self, config=None):
        super().__init__(config=config)
        print("SimulationNode initialized")

        # Override robot ID from config if provided
        if config and "robot_id" in config:
            self.ROBOT_ID = config["robot_id"]
        if config and "headless" in config:
            self._headless = config["headless"]
        if config and "sim_rate" in config:
            self._sim_rate = config["sim_rate"]
        if config and "urdf_path" in config:
            self._urdf_path = config["urdf_path"]
        if config and "update_rate" in config:
            self._update_rate = config["update_rate"]

        self.hz = 1 / self._update_rate

        # Register signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        # Now we set up subscriptions
        print("Setting up joint state subscription")
        self._joint_state_sub = self.subscribe("cmd/joints", self._on_joint_state)
        print("Joint state subscription set up")

        # Start simulation in main thread
        self._startSimulation()

    # This converts the joint state into a tuple of (control_mode, joint indices, position, velocity, effort)
    def joint_state_to_sim(self, joint_state: JointState):
        """Convert a JointState message into simulation control parameters.
        
        Args:
            joint_state: The JointState message containing joint commands
            
        Returns:
            Tuple containing:
            - control_mode: The PyBullet control mode to use
            - joint_indices: List of joint indices to control
            - positions: List of target positions (or [])
            - velocities: List of target velocities (or [])
            - efforts: List of effort limits (or [])
        """
        # Get joint indices for each joint name
        joint_indices = []
        for name in joint_state.name:
            if name not in self._joint_indices:
                print(f"Warning: Joint {name} not found in robot")
                continue
            joint_indices.append(self._joint_indices[name])
            
        if not joint_indices:
            print("No valid joints found in joint state")
            return None, [], [], [], []
            
        # Determine control mode based on provided parameters
        has_position = len(joint_state.position) > 0
        has_velocity = len(joint_state.velocity) > 0
        has_effort = len(joint_state.effort) > 0
        
        # Initialize control parameters as empty lists
        positions = []
        velocities = []
        efforts = []
        
        # Map joint state values to control parameters
        for i, idx in enumerate(joint_indices):
            if has_position and i < len(joint_state.position):
                positions.append(joint_state.position[i])
            if has_velocity and i < len(joint_state.velocity):
                velocities.append(joint_state.velocity[i])
            if has_effort and i < len(joint_state.effort):
                efforts.append(joint_state.effort[i])
                
        # Determine control mode
        if has_position:
            control_mode = p.POSITION_CONTROL
        elif has_velocity:
            control_mode = p.VELOCITY_CONTROL
        elif has_effort:
            control_mode = p.TORQUE_CONTROL
        else:
            print("No control parameters provided in joint state")
            return None, [], [], [], []
            
        return control_mode, joint_indices, positions, velocities, efforts

    def _on_joint_state(self, msg):
        try:
            joint_state = JointState.model_validate(msg)

            # Convert joint state to simulation parameters
            # Add them to the control queue
            self._joint_commands.append(self.joint_state_to_sim(joint_state))
            

        except Exception as e:
            print(f"Error parsing joint state: {e}")
            import traceback
            traceback.print_exc()

    def _signal_handler(self, signum, frame):
        print("\nReceived shutdown signal, cleaning up...")
        self.cleanup()
        os._exit(0)

    def _startSimulation(self):
        try:
            # Start physics server in shared memory mode
            print("Starting physics server...")
            self._physics_client = p.connect(p.GUI_SERVER)
            print(f"Physics server started with client ID: {self._physics_client}")

            # Configure physics
            p.setRealTimeSimulation(1)  # Enable real-time simulation
            p.setGravity(0, 0, -9.81)
            p.setTimeStep(1.0 / self._sim_rate)

            # Load environment
            print("Loading environment...")
            # Configure visualization
            p.configureDebugVisualizer(
                p.COV_ENABLE_GUI, 0
            )  # Disable the GUI overlay for better performance
            p.configureDebugVisualizer(
                p.COV_ENABLE_SHADOWS, 0
            )  # Disable shadows for better performance
            p.configureDebugVisualizer(
                p.COV_ENABLE_RENDERING, 1
            )  # Keep rendering enabled
            p.configureDebugVisualizer(
                p.COV_ENABLE_TINY_RENDERER, 0
            )  # Disable tiny renderer
            p.configureDebugVisualizer(
                p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0
            )  # Disable RGB buffer preview
            p.configureDebugVisualizer(
                p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0
            )  # Disable depth buffer preview
            p.configureDebugVisualizer(
                p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0
            )  # Disable segmentation preview
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.loadURDF("plane.urdf")

            # Load robot
            print(f"Loading robot from {self._urdf_path}")
            self._robot_id = p.loadURDF(
                self._urdf_path,
                [0, 0, 0.10],
                useFixedBase=False,
                physicsClientId=self._physics_client,
            )
            print(f"Robot loaded with ID: {self._robot_id}")
            # We need to enumerate the joints, make a mapping of joint names to indices
            self._joint_indices = {}
            for i in range(
                p.getNumJoints(self._robot_id, physicsClientId=self._physics_client)
            ):
                joint_info = p.getJointInfo(
                    self._robot_id, i, physicsClientId=self._physics_client
                )
                self._joint_indices[joint_info[1].decode("utf-8")] = i
            print(f"Joint indices: {self._joint_indices}")

            # Initialize joint state arrays
            self._joint_indices_array = list(self._joint_indices.values())
            num_joints = len(self._joint_indices_array)
            self._joint_positions = [None] * num_joints
            self._joint_velocities = [None] * num_joints
            self._joint_efforts = [None] * num_joints
            print(f"Initialized joint arrays with {num_joints} joints")

            if self._robot_id is None:
                raise Exception("Failed to load robot URDF")

            print("Environment loaded")

            # Connect state client
            print("Connecting state client...")
            self._state_client = p.connect(p.SHARED_MEMORY)
            print(f"State client connected with ID: {self._state_client}")

            # Verify robot state
            pos, orn = p.getBasePositionAndOrientation(
                self._robot_id, physicsClientId=self._state_client
            )
            print(f"Initial robot position: {pos}, orientation: {orn}")

        except Exception as e:
            print(f"Error in _startSimulation: {e}")
            import traceback

            traceback.print_exc()
            self.cleanup()
            raise

    def start(self):
        self._running = True
        thread = threading.Thread(target=self.run, daemon=True)
        thread.start()
        self._threads.append(thread)

        while self._running:
            try:
                # This is a hack, it's a noop but it stops spinning
                p.setPhysicsEngineParameter()
                time.sleep(0.1)  # Just keep the main thread alive
            except KeyboardInterrupt:
                self.cleanup()
                break

    def step(self):
        try:
            if self._state_client is None:
                print("State client not connected")
                return

            if self._robot_id is None:
                print("Robot not loaded")
                return

            # Get robot state using state client
            pos, orn = p.getBasePositionAndOrientation(
                self._robot_id, physicsClientId=self._state_client
            )

            for command in self._joint_commands:
                control_mode, joint_indices, positions, velocities, efforts = command
                for i, idx in enumerate(joint_indices):
                    # Build control parameters based on what's available
                    control_params = {
                        "bodyIndex": self._robot_id,
                        "jointIndex": idx,
                        "controlMode": control_mode,
                        "physicsClientId": self._state_client,
                    }
                    
                    # Only add parameters if they have values
                    if i < len(positions):
                        control_params["targetPosition"] = positions[i]
                    if i < len(velocities):
                        control_params["targetVelocity"] = velocities[i]
                    if i < len(efforts):
                        control_params["force"] = efforts[i]
                        
                    p.setJointMotorControl2(**control_params)
            self._joint_commands = []

        except Exception as e:
            print(f"Error in step: {e}")
            import traceback

            traceback.print_exc()

    def cleanup(self):
        print("Cleaning up simulation...")
        self._running = False

        # Wait for threads to finish
        for thread in self._threads:
            if thread.is_alive():
                thread.join(timeout=1.0)

        if self._state_client is not None:
            p.disconnect(self._state_client)
            self._state_client = None
        if self._physics_client is not None:
            p.disconnect(self._physics_client)
            self._physics_client = None
        print("Simulation cleaned up")
