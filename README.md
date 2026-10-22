# course_project

基于 Quanser **QArm**（4 自由度机械臂）的大作业项目：仿真用 Genesis，真机通过 Quanser `pal` 用 Python 控制，并做 real2sim。

- 调研报告：[docs/research/qarm_survey_2026-10-22.md](docs/research/qarm_survey_2026-10-22.md)
- 上机清单（2026-10-23）：[docs/hardware_day_20261023.md](docs/hardware_day_20261023.md)

## 目录结构

| 目录 | 用途 |
|---|---|
| `src/qarm_lab/` | 代码包：规格、运动学、舵机模型、轨迹与检查、安全监控、执行器、后端（`pal` 真机 / `genesis` 仿真 / `fake` 假后端） |
| `scripts/` | 入口：`run.py`（真机和仿真共用）；`model/`（生成 URDF 和 mesh、渲染检查）；`traj/`（轨迹集）；`sim/`（Genesis 冒烟测试、重放对比）；`analysis/`（辨识）；`hw/`（实验室环境检查）；`research/` |
| `models/` | `qarm_official/` 是官方原件（固定 commit，**不得修改**）；`qarm/` 是派生的 URDF 和 mesh（由脚本生成） |
| `data/` | `trajectories/<集合>/`（实验轨迹）；`real/<会话>/`（真机日志，进 git） |
| `results/` | `model_check_*`、`sim_runs/*`、`replay_*`、`sysid_*` 等产物（按时间戳分子目录，不覆盖） |
| `env/` | Mac 端 requirements；`4090/`（conda 环境和 Quanser SDK 安装脚本、lock 文件） |
| `tests/` | `pytest tests` |
| `logs/` | 运行日志 |
| `docs/` | 文档 |
| `_backup/` | 不再使用但不删的大文件（不进 git） |

## 快速上手

```bash
# Mac：单元测试
.venv/bin/python -m pytest -q tests
# 4090：Genesis 仿真里干跑整套实验轨迹
ssh 4090
cd ~/course_project && ~/miniconda3/envs/qarm/bin/python scripts/run.py --backend genesis --set data/trajectories/hwday_20261023
# 4090 + 真机：先只读，再原地锁住，再跑单条轨迹（详见上机清单）
python scripts/run.py --backend pal --read-only 5
python scripts/run.py --backend pal --hold-here 5
python scripts/run.py --backend pal --traj data/trajectories/hwday_20261023/02_steps_shoulder.npz
# 把真机日志放进仿真重放并对比
python scripts/sim/replay_in_sim.py data/real/<会话>/<运行>
```
