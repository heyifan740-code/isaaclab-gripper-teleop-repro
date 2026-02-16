"""
Gripper teleoperation environment with keyboard control.
"""

import gymnasium as gym

from . import gripper_env_cfg

gym.register(
    id="Isaac-Gripper-Teleop-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.gripper_env_cfg:GripperTeleopEnvCfg",
    },
)
