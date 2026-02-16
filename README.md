# Gripper Teleop 提交与 Isaac Sim 复现指南

本指南用于把“夹爪 teleop 功能”作为**可复现提交包**发布到新的 GitHub 仓库，确保他人可在 Isaac Sim/IsaacLab 环境中复现并验证。

## 1. 提交内容范围（相关代码）

建议至少包含以下路径（与功能直接相关）：

- `scripts/teleop_gripper.py`
- `source/isaaclab_tasks/isaaclab_tasks/manager_based/manipulation/gripper_teleop/`
  - `__init__.py`
  - `gripper_env_cfg.py`
  - `mdp/__init__.py`
  - `mdp/actions.py`
  - `mdp/observations.py`
- 运行所需 gripper 资源（如已在本仓库维护）
  - `assets/gripper/`

## 2. 推荐发布方式（最稳）

推荐做法是创建一个**轻量 overlay 仓库**（只放相关代码），而不是上传整个 IsaacLab 大仓库。这样更清晰、可复现也更快。

---

## 3. 创建发布目录（本机）

在终端执行：

```bash
mkdir -p ~/gripper-teleop-release
cd ~/gripper-teleop-release
mkdir -p scripts
mkdir -p source/isaaclab_tasks/isaaclab_tasks/manager_based/manipulation
mkdir -p assets
```

复制相关代码：

```bash
cp /home/fan/workspace/IsaacLab/scripts/teleop_gripper.py scripts/
cp -r /home/fan/workspace/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/manipulation/gripper_teleop source/isaaclab_tasks/isaaclab_tasks/manager_based/manipulation/
cp -r /home/fan/workspace/IsaacLab/assets/gripper assets/
cp /home/fan/workspace/IsaacLab/SUBMISSION_GRIPPER_TELEOP_REPRO.md README.md
```

> 如果你的 `assets/gripper` 不在本地，请把资源路径改成实际位置。

---

## 4. 新建 GitHub 仓库并推送

当前环境未检测到 `gh` CLI，因此使用 Web + git 命令方式。

### 4.1 在 GitHub 网页创建新仓库

- 打开：`https://github.com/new`
- 仓库名建议：`isaaclab-gripper-teleop-repro`
- 选择 `Public` 或 `Private`
- **不要**勾选初始化 README（避免冲突）

### 4.2 本地初始化并推送

把 `<YOUR_GITHUB_NAME>` 替换为你的用户名：

```bash
cd ~/gripper-teleop-release
git init
git add .
git commit -m "feat: add gripper teleop reproducible package"
git branch -M main
git remote add origin https://github.com/<YOUR_GITHUB_NAME>/isaaclab-gripper-teleop-repro.git
git push -u origin main
```

---

## 5. 他人复现流程（Isaac Sim / IsaacLab）

### 5.1 准备基础 IsaacLab

先按官方方式安装 IsaacLab（与目标 Isaac Sim 版本匹配，推荐 Isaac Sim 5.1）。

### 5.2 叠加你的提交包

假设他人已经有 IsaacLab 工作目录：`~/IsaacLab`，执行：

```bash
cd ~
git clone https://github.com/<YOUR_GITHUB_NAME>/isaaclab-gripper-teleop-repro.git
rsync -av isaaclab-gripper-teleop-repro/scripts/ ~/IsaacLab/scripts/
rsync -av isaaclab-gripper-teleop-repro/source/ ~/IsaacLab/source/
rsync -av isaaclab-gripper-teleop-repro/assets/ ~/IsaacLab/assets/
```

### 5.3 运行验证

```bash
cd ~/IsaacLab
./isaaclab.sh -p scripts/teleop_gripper.py
```

---

## 6. 功能验证清单（复现验收）

运行后按以下步骤验证：

1. 使用 `J` 夹爪闭合，夹起 `cube`
2. 移动 base 到新位置
3. 使用 `K` 夹爪张开，释放 `cube`
4. 终端日志可见模式切换信息：
   - `[gripper] mode -> close (J)`
   - `[gripper] mode -> open (K)`
   - `[gripper] apply material set: ...`

判定标准：

- 空载时 `J/K` 可切换开合
- 夹住 `cube` 后仍可通过 `K` 张开并释放

---

## 7. 建议附加信息（提高可信度）

建议在仓库 `README` 补充：

- OS / GPU / 驱动版本
- Isaac Sim 版本
- IsaacLab 分支或 commit
- 你验证成功的最小命令
- 一段 10~20 秒演示视频（抓取 -> 移动 -> 释放）

---

## 8. 常见问题排查

- 按键无响应：确认窗口焦点在 Isaac Sim 主窗口
- 释放不明显：确认运行日志出现 `mode -> open (K)` 与 `apply material set: open`
- 资源缺失：确认 `assets/gripper` 已同步

---

如果你希望，我可以继续帮你生成：

1. 一个更简短的仓库 `README.md`（面向评审）
2. 一个 `reproduce.sh` 一键复现脚本（自动 rsync + 运行命令）
3. 一份提交前检查清单（环境、命令、预期输出）
