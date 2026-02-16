"""Observation definitions for the gripper teleop environment."""

from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.managers import SceneEntityCfg, ObservationGroupCfg
from isaaclab.utils import configclass

import isaaclab.envs.mdp as mdp

@configclass
class ObservationsCfg:
    """Observation specifications for the environment."""

    @configclass
    class PolicyCfg(ObservationGroupCfg):
        """Observations for policy group."""

        # Joint positions
        joint_pos = mdp.ObservationTermCfg(
            func=mdp.joint_pos_rel,
            scale=1.0,
            params={"asset_cfg": SceneEntityCfg("robot")},
        )
        
        # Joint velocities
        joint_vel = mdp.ObservationTermCfg(
            func=mdp.joint_vel_rel,
            scale=1.0,
            params={"asset_cfg": SceneEntityCfg("robot")},
        )
        
        # Object position
        object_pos = mdp.ObservationTermCfg(
            func=mdp.root_pos_w,
            params={"asset_cfg": SceneEntityCfg("cube")},
        )

    policy: PolicyCfg = PolicyCfg()
