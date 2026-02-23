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
```

### Command explanation
- `rsync -av ...`: copies files/folders while preserving structure and showing progress.
- This overlays only the relevant teleop/task/assets into your Isaac Lab workspace.

---

## 3) Launch the Teleop Script

```bash
cd ~/IsaacLab
./isaaclab.sh -p scripts/teleop_gripper.py
```

---

## 4) Runtime Controls (Keyboard)

- Base translation: `W/S`, `A/D`, `Q/E`
- Base rotation: `Z/X`, `T/G`, `C/V`
- Gripper close: `J`
- Gripper open: `K`

---

## 5) Reproduction Test Procedure (Step-by-Step)

Follow exactly:

1. Start simulation and wait until scene is stable.
2. Move gripper above the cube.
3. Press `J` to close and grasp.
4. Move the base to another location while holding the cube.
5. Press `K` to open and release.
6. Confirm the cube drops/gets released.

---

## 6) Expected Validation Signals

### Functional validation
- `J` consistently closes gripper.
- `K` consistently opens gripper.

### Console/log validation
Look for mode-switch logs:
- `[gripper] mode -> close (J)`
- `[gripper] mode -> open (K)`
- `[gripper] apply material set: close/open`

---

## 7) Quick Troubleshooting

### Symptom: Keys do not respond
- Ensure Isaac Sim window is focused.
- Click inside viewport, then retry keys.

### Symptom: Can grasp but hard to release
- Verify `K` press logs appear in terminal.
- Verify `apply material set: open` appears after `K`.

### Symptom: Script launch issues
- Re-check file overlay paths (`scripts/`, `source/`, `assets/`).
- Re-run from Isaac Lab root using `./isaaclab.sh -p ...`.

---

## 8) Minimal Re-run Commands

```bash
cd ~/IsaacLab
./isaaclab.sh -p scripts/teleop_gripper.py
```
