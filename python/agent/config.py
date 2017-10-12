""" Default configuration and hyperparameters for agent objects. """
import numpy as np

# AgentMuJoCo
AGENT_MUJOCO = {
    'substeps': 1,
    'camera_pos': np.array([2., 3., 2., 0., 0., 0.]),
    'image_width': 640,
    'image_height': 480,
    'image_channels': 3,
    'meta_include': []
}

ACTION = 0
JOINT_ANGLES = 1
JOINT_VELOCITIES = 2
END_EFFECTOR_POINTS = 3
END_EFFECTOR_POINT_VELOCITIES = 4
END_EFFECTOR_POINT_JACOBIANS = 5
END_EFFECTOR_POINT_ROT_JACOBIANS = 6
END_EFFECTOR_POSITIONS = 7
END_EFFECTOR_ROTATIONS = 8
END_EFFECTOR_JACOBIANS = 9
END_EFFECTOR_HESSIANS = 10
