# Isaac Sim Gripper Teleop Reproduction Guide

## 1) Prerequisites & Installation

To run this package, you need a working setup of **Isaac Sim 5.1** and **Isaac Lab**. Linux (Ubuntu 22.04) with a capable NVIDIA GPU is highly recommended, and this package must be cloned locally.

**Isaac Sim 5.1 Installation:**
Please download the NVIDIA Omniverse Launcher and install Isaac Sim version 5.1 from the Exchange tab.
- [Isaac Sim Official Page & Download](https://developer.nvidia.com/isaac-sim)

**Isaac Lab Installation:**
Please clone the official Isaac Lab repository and follow their setup guide to link it with your Isaac Sim installation.
- [Isaac Lab GitHub Repository](https://github.com/isaac-sim/IsaacLab)
- [Isaac Lab Installation Documentation](https://isaac-sim.github.io/IsaacLab/main/setup/installation.html)

Ensure your Python environment is activated (if your setup requires it) before proceeding.

---

## This Package Contains

This repo provides a reproducible gripper teleoperation workflow for Isaac Sim / Isaac Lab.

Main files:
- `scripts/teleop_gripper.py`
- `source/isaaclab_tasks/isaaclab_tasks/manager_based/manipulation/gripper_teleop/`
- `assets/gripper/`

---

## 2) Integrate This Package into an Existing Isaac Lab Workspace

Assume:
- Your Isaac Lab workspace is at: `~/IsaacLab`
- This repository is cloned at: `~/isaaclab-gripper-teleop-repro`

Run:

```bash
cd ~
rsync -av isaaclab-gripper-teleop-repro/scripts/ ~/IsaacLab/scripts/
rsync -av isaaclab-gripper-teleop-repro/source/ ~/IsaacLab/source/
rsync -av isaaclab-gripper-teleop-repro/assets/ ~/IsaacLab/assets/
