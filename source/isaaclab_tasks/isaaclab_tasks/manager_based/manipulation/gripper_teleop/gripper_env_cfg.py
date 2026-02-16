"""
Configuration for the gripper teleoperation environment.
"""
import os
from isaaclab.envs import ManagerBasedRLEnvCfg, ViewerCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.assets import ArticulationCfg, RigidObjectCfg, AssetBaseCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass

import isaaclab.sim as sim_utils

from . import mdp as gripper_mdp

@configclass
class GripperSceneCfg(InteractiveSceneCfg):
    """Configuration for the gripper teleoperation scene."""

    # Terrain - Infinite plane
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="max",
            restitution_combine_mode="multiply",
            static_friction=1.5,
            dynamic_friction=1.2,
        ),
        debug_vis=False,
    )

    # Gripper robot
    robot: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/Gripper",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{os.getcwd()}/assets/gripper/usd/2DOF_gripper.usd",
            activate_contact_sensors=True,
            collision_props=sim_utils.CollisionPropertiesCfg(
                contact_offset=0.0065,
                rest_offset=0.00055,
            ),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=True, # Floating gripper
                retain_accelerations=False,
                linear_damping=0.0,
                angular_damping=0.0,
                max_linear_velocity=1000.0,
                max_angular_velocity=1000.0,
                max_depenetration_velocity=0.35,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=64,
                solver_velocity_iteration_count=12,
                sleep_threshold=0.005,
                stabilization_threshold=0.001,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.2), # Floating above ground
            rot=(1.0, 0.0, 0.0, 0.0), # Identity
            joint_pos={"left_joint": 0.0, "right_joint": 0.0},
        ),
        actuators={
            "fingers": ImplicitActuatorCfg(
                joint_names_expr=[".*_joint"],
                effort_limit_sim=15.0,
                velocity_limit_sim=0.5,
                stiffness=80.0,
                damping=32.0,
            ),
        },
    )

    finger_contacts = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Gripper/(left_finger|right_finger)",
        history_length=2,
        track_air_time=False,
    )

    # Cube object
    cube: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Cube",
        spawn=sim_utils.CuboidCfg(
            size=(0.035, 0.035, 0.05),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                rigid_body_enabled=True,
                max_depenetration_velocity=0.35,
                disable_gravity=False,
                sleep_threshold=0.0, # Visual fix: prevent sleeping
            ),
            collision_props=sim_utils.CollisionPropertiesCfg(
                contact_offset=0.0034,
                rest_offset=0.0003,
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.04),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 1.0, 0.0)),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=2.6,
                dynamic_friction=2.1,
                friction_combine_mode="max",
            ),
            visible=True,
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.0, 0.0, 0.1)), # Drop from higher up (0.1m) to force update
    )
    
    # Light
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DistantLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )


@configclass
class GripperTeleopEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the gripper teleoperation environment."""

    # Scene settings
    scene: GripperSceneCfg = GripperSceneCfg(num_envs=1, env_spacing=2.0)
    
    # Basic settings
    observations: gripper_mdp.ObservationsCfg = gripper_mdp.ObservationsCfg()
    actions: gripper_mdp.ActionsCfg = gripper_mdp.ActionsCfg()
    events: object = gripper_mdp.EventsCfg()
    rewards: object = gripper_mdp.RewardsCfg()
    terminations: object = gripper_mdp.TerminationsCfg()
    curriculum: object = gripper_mdp.CurriculumCfg()
    episode_length_s: float = 60.0

    viewer: ViewerCfg = ViewerCfg(
        eye=(1.5, 1.5, 1.5),
        lookat=(0.0, 0.0, 0.5),
    )

    def __post_init__(self):
        """Post initialization."""
        self.sim.dt = 0.0025
        self.sim.render_interval = 4
        self.decimation = 4 # Control frequency = 100Hz
        self.sim.physx.enable_ccd = True
        if self.scene.finger_contacts is not None:
            self.scene.finger_contacts.update_period = self.sim.dt
        self.sim.physics_material = sim_utils.RigidBodyMaterialCfg(
            static_friction=1.2,
            dynamic_friction=1.0,
            friction_combine_mode="max",
            restitution_combine_mode="multiply",
        )
