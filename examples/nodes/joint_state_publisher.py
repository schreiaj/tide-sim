from tide import BaseNode
from tide_sim import JointState
import math
import random
import os

class JointStatePublisher(BaseNode):
    ROBOT_ID = "simbot"
    GROUP = "simulation"
    hz = 0.5

    angle = 0

    def __init__(self, config=None):
        super().__init__(config=config)
        print("JointStateTest initialized")
        print(os.getcwd())

    def step(self):
        azimuth_state = JointState(
            name=["fr_steering", "fl_steering", "rl_steering", "rr_steering"],
            # position=[math.radians(45.0), math.radians(135.0), math.radians(225.0), math.radians(315.0)],
            position=[self.angle,self.angle, self.angle, self.angle],
            velocity=[],
            effort=[]
        )
        

        drive_state = JointState(
            name=["fr_drive_wheel", "fl_drive_wheel", "rl_drive_wheel", "rr_drive_wheel"],
            position=[],
            velocity=[10,10,10,10],
            effort=[]
        )

        self.put("cmd/joints", azimuth_state.to_bytes())
        self.put("cmd/joints", drive_state.to_bytes())

        self.angle += math.radians(45)
        