# course_project — 项目说明

- 创建日期：2026-10-22
- GitHub：Jasonartieth/course_project（私有）
- 主题：基于 Quanser **QArm**（4 自由度，roll-pitch-pitch-roll）的大作业（具体作业要求待补）

## 目录约定
- 代码 → `scripts/`；数据 → `data/`；产物 → `results/<时间戳或序号>/`；日志 → `logs/`；文档 → `docs/`
- 不再使用但不删的大文件 → `_backup/<任务名>_<日期>/`（不进 git，仓库 archive 里留清单+路径指针）

## 进展
- 2026-10-22：建立项目骨架与私有仓
- 2026-10-22：完成 QArm 调研 → `docs/research/qarm_survey_2026-10-22.md`。结论：官方 URDF（quanser/urdf_representations，BSD-3）运动学已与官方 DH 交叉验证一致（`scripts/research/fk_crosscheck_urdf_vs_dh.py`）；电机为 Dynamixel XM540-W270-R（底座/肩×2/肘）+ XM430-W350-R（腕）+ XC430-W240-T（夹爪），瞬时/持续力矩 10.6/21.2/10.6/4.1 与 2.65/5.3/2.65/1.0 N·m，速度上限 π/2 rad/s；真实质量和惯量见官方 `7. Dynamic Parameters.pdf`。没有现成的 MuJoCo 模型。仿真路线待按作业要求选定。
- 2026-10-22：确认**真机可以纯 Python 控制，不必用 Simulink**。官方 `pal.products.qarm.QArm.read_write_std()` 在 Position 模式下下发关节角和夹爪指令，同时回读位置、速度、电流(A)、PWM、温度，默认 500 Hz；`pal` 不支持 PWM/Current 模式，Simulink 需要 QUARC 许可证。Python 依赖 Quanser SDK（含 `quanser.hardware`），**官方只提供 Windows 和 Linux 版**；macOS 只有 beta 版的 C 语言 HIL SDK（列出了 QArm，但没有 Python API）→ 真机要接在 Windows/Linux 电脑上。用户计划日后连真机做 real2sim（不完全信任网上参数）。

## 已知坑 / ★
- ★ git push 报 "Repository not found"：本机 SSH key 绑定的是 kefusong 账号，访问不到 Jasonartieth 的私有仓 → remote 必须用 HTTPS（`https://github.com/Jasonartieth/course_project.git`，走 gh 凭据）。`gh repo create --source=. --push` 默认用 SSH，会失败，需要先 `git remote set-url` 改成 HTTPS。
- ★ 官方 QARM.urdf 不能直接用于物理仿真：① 所有关节 effort/velocity=0；② 惯性参数是占位值（文件头注明）；③ mesh 路径 `package://QARM/` 和目录 `qarm`、package 名 `QArm` 大小写不一致；④ 夹爪是单独的 URDF，限位全为 0、没有连到臂上；⑤ 没有 TCP，需在 END-EFFECTOR 后加固定关节 xyz=(0,0,0.162) 才能与官方 FK/IK（λ3=0.40）对齐（已实测，误差 <1 mm）；⑥ 腕关节限位 ±172.5°，手册为 ±160°。修复清单见调研报告 §6。
- ★ `quanser/Quanser_Academic_Resources` 的默认分支是 `dev-windows`，没有 `main`，用 `gh api .../git/trees/main` 会 404。
- ★ 本机 `/usr/bin/python3` 的 numpy 装在用户 site-packages 里，`python3 -I` 会 import 失败；需要隔离时，把下载文件和脚本分目录存放，再去掉 `-I` 运行。

## 归档索引（任务 → 仓库/分支/tag/路径）
