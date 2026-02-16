# Isaac Sim Gripper Teleop Reproduction Guide

## Demo

<video src="demo.mp4" controls width="900"></video>

If the embedded player does not load, [https://drive.google.com/file/d/1CayQsx1-aXp-lal_XPI19Do_JIKh9EzZ/view?usp=sharing].

Recommended demo content:
1. Close gripper (`J`) and grasp the cube
2. Move base to another location : a.Base translation: `W/S`, `A/D`, `Q/E` b. Base rotation: `Z/X`, `T/G`, `C/V`
3. Open gripper (`K`) and release the cube
4. Show terminal logs:
   - `[gripper] mode -> close (J)`
   - `[gripper] mode -> open (K)`
   - `[gripper] apply material set: ...`

---

## This Package Contains

This repo provides a reproducible gripper teleoperation workflow for Isaac Sim / Isaac Lab.

Main files:
- `scripts/teleop_gripper.py`
- `source/isaaclab_tasks/isaaclab_tasks/manager_based/manipulation/gripper_teleop/`
- `assets/gripper/`

---

## 1) Prerequisites

need a working Isaac Lab + Isaac Sim setup (Linux recommended), and this package cloned locally.

- Isaac Lab installed and runnable
- Compatible Isaac Sim version (same family as your Isaac Lab setup)
- Python environment activated (if your setup requires it)

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

### Command explanation
- `./isaaclab.sh`: Isaac Lab launcher script.
- `-p scripts/teleop_gripper.py`: runs the teleoperation Python entry script with Isaac runtime.

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
7. Repeat grasp/release 3 times to confirm consistency.

---

## 6) Expected Validation Signals

### Functional validation
- `J` consistently closes gripper.
- `K` consistently opens gripper.
- Cube can be released after grasp (not only in no-load condition).

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

---

