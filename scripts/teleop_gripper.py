"""Script to teleoperate the gripper with keyboard."""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Teleoperate gripper.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch

import isaaclab.sim as sim_utils
from isaaclab.devices import Se3Keyboard, Se3KeyboardCfg
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg
from isaaclab.utils.math import quat_from_euler_xyz, quat_mul
from isaaclab_tasks.manager_based.manipulation.gripper_teleop.gripper_env_cfg import GripperTeleopEnvCfg


BASE_POS_SENSITIVITY = 0.004
BASE_ROT_SENSITIVITY = 0.01

VT_CLOSE_STEP = 0.010
VT_CONTACT_STOP_FORCE = 1.0

TORQUE_VIS_SCALE = 0.014
TORQUE_VIS_MAX_RADIUS = 0.032
TORQUE_VIS_MIN_RADIUS = 0.0
TORQUE_DEADZONE = 0.20
TORQUE_SMOOTH_ALPHA = 0.15
TORQUE_RAMP_STEPS = 150
CONTACT_FORCE_DEADZONE = 0.7
CONTACT_FORCE_SPAN = 1.2


def _bind_mode_materials(mode_state, finger_low_path, finger_high_path, cube_low_path, cube_high_path):
    finger_path = finger_low_path if mode_state == "open" else finger_high_path
    cube_path = cube_low_path if mode_state == "open" else cube_high_path
    sim_utils.bind_physics_material("/World/envs/env_0/Gripper/left_finger", finger_path)
    sim_utils.bind_physics_material("/World/envs/env_0/Gripper/right_finger", finger_path)
    sim_utils.bind_physics_material("/World/envs/env_0/Cube", cube_path)


def main():
    env_cfg = GripperTeleopEnvCfg()
    env_cfg.scene.robot.init_state.pos = (0.0, 0.0, 0.5)
    env_cfg.sim.use_fabric = True
    env = ManagerBasedRLEnv(cfg=env_cfg, render_mode=None)

    teleop = Se3Keyboard(cfg=Se3KeyboardCfg(pos_sensitivity=BASE_POS_SENSITIVITY, rot_sensitivity=BASE_ROT_SENSITIVITY))
    print("Teleop Controller: ", teleop)
    print("Controls:")
    print("  Move: W/S, A/D, Q/E")
    print("  Rotate: Z/X, T/G, C/V")
    print("  Gripper Close: J")
    print("  Gripper Open: K")
    print("  Gripper Mode: vtrefine")

    gripper_mode_state = "open"
    last_gripper_mode_state = gripper_mode_state

    def close_gripper_cb():
        nonlocal gripper_mode_state
        gripper_mode_state = "close"
        print("[gripper] mode -> close (J)")

    def open_gripper_cb():
        nonlocal gripper_mode_state
        gripper_mode_state = "open"
        print("[gripper] mode -> open (K)")

    teleop.add_callback("J", close_gripper_cb)
    teleop.add_callback("K", open_gripper_cb)

    env.reset()

    finger_high_friction_cfg = sim_utils.RigidBodyMaterialCfg(
        static_friction=3.8,
        dynamic_friction=3.2,
        friction_combine_mode="max",
        restitution_combine_mode="multiply",
    )
    finger_low_friction_cfg = sim_utils.RigidBodyMaterialCfg(
        static_friction=0.02,
        dynamic_friction=0.01,
        friction_combine_mode="min",
        restitution_combine_mode="multiply",
    )

    cube_high_friction_cfg = sim_utils.RigidBodyMaterialCfg(
        static_friction=2.6,
        dynamic_friction=2.1,
        friction_combine_mode="max",
        restitution_combine_mode="multiply",
    )
    cube_low_friction_cfg = sim_utils.RigidBodyMaterialCfg(
        static_friction=0.02,
        dynamic_friction=0.01,
        friction_combine_mode="min",
        restitution_combine_mode="multiply",
    )

    finger_high_friction_path = "/World/PhysicsMaterials/FingerHighFriction"
    finger_low_friction_path = "/World/PhysicsMaterials/FingerLowFriction"
    cube_high_friction_path = "/World/PhysicsMaterials/CubeHighFriction"
    cube_low_friction_path = "/World/PhysicsMaterials/CubeLowFriction"

    sim_utils.spawn_rigid_body_material(finger_high_friction_path, finger_high_friction_cfg)
    sim_utils.spawn_rigid_body_material(finger_low_friction_path, finger_low_friction_cfg)
    sim_utils.spawn_rigid_body_material(cube_high_friction_path, cube_high_friction_cfg)
    sim_utils.spawn_rigid_body_material(cube_low_friction_path, cube_low_friction_cfg)

    _bind_mode_materials(
        gripper_mode_state,
        finger_low_friction_path,
        finger_high_friction_path,
        cube_low_friction_path,
        cube_high_friction_path,
    )

    torque_marker_left = VisualizationMarkers(
        VisualizationMarkersCfg(
            prim_path="/Visuals/TorqueLeft",
            markers={
                "torque": sim_utils.SphereCfg(
                    radius=1.0,
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0)),
                )
            },
        )
    )
    torque_marker_right = VisualizationMarkers(
        VisualizationMarkersCfg(
            prim_path="/Visuals/TorqueRight",
            markers={
                "torque": sim_utils.SphereCfg(
                    radius=1.0,
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0)),
                )
            },
        )
    )

    scene = getattr(env.unwrapped, "scene")
    robot = scene["robot"]
    finger_contacts = scene["finger_contacts"]
    contact_ids, _ = finger_contacts.find_bodies(["left_finger", "right_finger"], preserve_order=True)

    joint_indices = [0, 1]
    body_indices = [1, 2]
    joint_limits = robot.data.soft_joint_pos_limits[0, joint_indices, :]
    open_target = float(torch.max(joint_limits[:, 0]).item())
    close_target = float(torch.min(joint_limits[:, 1]).item())
    joint_lower_bound = min(open_target, close_target)
    joint_upper_bound = max(open_target, close_target)
    gripper_target = open_target

    torque_radius_left = 0.01
    torque_radius_right = 0.01
    step_count = 0

    while simulation_app.is_running():
        command = teleop.advance()
        delta_pose = command[:6].to(env.device)

        root_pos = robot.data.root_pos_w.clone()
        root_quat = robot.data.root_quat_w.clone()
        root_pos += delta_pose[0:3]

        delta_rot = quat_from_euler_xyz(delta_pose[3], delta_pose[4], delta_pose[5])
        if delta_rot.ndim == 1:
            delta_rot = delta_rot.unsqueeze(0)
        new_rot = quat_mul(root_quat, delta_rot)
        robot.write_root_pose_to_sim(torch.cat([root_pos, new_rot], dim=-1))

        _bind_mode_materials(
            gripper_mode_state,
            finger_low_friction_path,
            finger_high_friction_path,
            cube_low_friction_path,
            cube_high_friction_path,
        )
        if gripper_mode_state != last_gripper_mode_state:
            print(f"[gripper] apply material set: {gripper_mode_state}")
            last_gripper_mode_state = gripper_mode_state

        if gripper_mode_state == "open":
            gripper_target = open_target
        else:
            target_delta = close_target - gripper_target
            vt_step = max(-VT_CLOSE_STEP, min(VT_CLOSE_STEP, target_delta))
            if robot.data.applied_torque is not None:
                net_forces_hold = finger_contacts.data.net_forces_w[0]
                contact_force_mean = 0.5 * (
                    torch.norm(net_forces_hold[contact_ids[0]]).item()
                    + torch.norm(net_forces_hold[contact_ids[1]]).item()
                )
                if contact_force_mean > VT_CONTACT_STOP_FORCE:
                    vt_step = 0.0
            gripper_target += vt_step

        gripper_target = max(joint_lower_bound, min(joint_upper_bound, gripper_target))
        action = torch.tensor([[gripper_target, gripper_target]], device=env.device)
        env.step(action)

        step_count += 1

        if robot.data.applied_torque is not None:
            torques = robot.data.applied_torque[0]
            all_body_pos = robot.data.body_pos_w[0]
            ramp_gain = min(1.0, step_count / float(TORQUE_RAMP_STEPS))

            left_torque = torch.abs(torques[joint_indices[0]]).item()
            right_torque = torch.abs(torques[joint_indices[1]]).item()

            net_forces = finger_contacts.data.net_forces_w[0]
            left_contact = torch.norm(net_forces[contact_ids[0]]).item()
            right_contact = torch.norm(net_forces[contact_ids[1]]).item()
            left_gate = max(0.0, min(1.0, (left_contact - CONTACT_FORCE_DEADZONE) / CONTACT_FORCE_SPAN))
            right_gate = max(0.0, min(1.0, (right_contact - CONTACT_FORCE_DEADZONE) / CONTACT_FORCE_SPAN))

            left_torque_eff = max(0.0, left_torque - TORQUE_DEADZONE) * left_gate
            right_torque_eff = max(0.0, right_torque - TORQUE_DEADZONE) * right_gate
            left_torque_vis = left_torque_eff**0.5
            right_torque_vis = right_torque_eff**0.5

            r_left_target = min(
                TORQUE_VIS_MAX_RADIUS,
                max(TORQUE_VIS_MIN_RADIUS, left_torque_vis * TORQUE_VIS_SCALE * ramp_gain),
            )
            torque_radius_left = (1.0 - TORQUE_SMOOTH_ALPHA) * torque_radius_left + TORQUE_SMOOTH_ALPHA * r_left_target
            pos_left = all_body_pos[body_indices[0]]

            torque_marker_left.visualize(
                translations=pos_left.unsqueeze(0),
                scales=torch.tensor([[torque_radius_left] * 3], device=env.device),
            )

            r_right_target = min(
                TORQUE_VIS_MAX_RADIUS,
                max(TORQUE_VIS_MIN_RADIUS, right_torque_vis * TORQUE_VIS_SCALE * ramp_gain),
            )
            torque_radius_right = (1.0 - TORQUE_SMOOTH_ALPHA) * torque_radius_right + TORQUE_SMOOTH_ALPHA * r_right_target
            pos_right = all_body_pos[body_indices[1]]

            torque_marker_right.visualize(
                translations=pos_right.unsqueeze(0),
                scales=torch.tensor([[torque_radius_right] * 3], device=env.device),
            )

    env.close()


if __name__ == "__main__":
    main()
