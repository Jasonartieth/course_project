# course_project — 项目说明

- 创建日期：2026-10-22
- GitHub：Jasonartieth/course_project（私有）
- 主题：基于 Quanser **QArm**（4 自由度，roll-pitch-pitch-roll）的大作业；**没有给定作业要求，任务由用户自定**。当前阶段是搭基础环境，并做真机 real2sim。
- 仿真器：**Genesis**（用户指定）。运行机器是 **4090**（`ssh 4090`，Tailscale <tailscale-ip>，用户 jcy，Ubuntu 24.04，Alienware 笔记本）；这台电脑也会被带去接真机，从 Mac 远程控制。
- Mac 只负责写代码和离线分析（`.venv`，uv，Python 3.12；装了 MuJoCo，只用来渲染检查 URDF）。

## 目录约定
- 代码 → `src/qarm_lab/`（包）、`scripts/`（入口）、`tests/`；数据 → `data/`（`trajectories/`、`real/`）；产物 → `results/<类别>_<时间戳>/`；日志 → `logs/`；文档 → `docs/`；环境 → `env/`；模型 → `models/`（`qarm_official/` 为官方原件**不得修改**，`qarm/` 为派生版本，由脚本生成）
- 不再使用但不删的大文件 → `_backup/<任务名>_<日期>/`（不进 git，仓库 archive 里留清单+路径指针）
- 仿真日志 `results/sim_runs/**/log.npz`、`results/replay_*/**/log.npz` 不进 git（可重新生成，留在磁盘上）；**真机数据 `data/real/` 全部进 git**。

## 两台机器的工作流
- Mac 是主力编辑端：改代码 → commit → push（Mac 用 HTTPS remote）。
- 4090：`cd ~/course_project && git pull`；conda 环境 `qarm`（`~/miniconda3/envs/qarm/bin/python`，`src/` 已通过 .pth 加入 path；tmux 也装在这个环境里）。4090 上产生的数据在 4090 上 commit 并 push（4090 的 SSH key 就是 Jasonartieth）。
- Quanser `pal`/`hal`：`~/Quanser/Quanser_Academic_Resources`（稀疏检出 `0_libraries/python`，分支 dev-windows）。Quanser SDK 需要用户用 sudo 执行 `env/4090/install_quanser_sdk.sh`。

## 进展
- 2026-10-22：建立项目骨架与私有仓
- 2026-10-22：完成 QArm 调研 → `docs/research/qarm_survey_2026-10-22.md`。结论：官方 URDF（quanser/urdf_representations，BSD-3）运动学已与官方 DH 交叉验证一致（`scripts/research/fk_crosscheck_urdf_vs_dh.py`）；电机为 Dynamixel XM540-W270-R（底座/肩×2/肘）+ XM430-W350-R（腕）+ XC430-W240-T（夹爪），瞬时/持续力矩 10.6/21.2/10.6/4.1 与 2.65/5.3/2.65/1.0 N·m，速度上限 π/2 rad/s；真实质量和惯量见官方 `7. Dynamic Parameters.pdf`。没有现成的 MuJoCo 模型。
- 2026-10-22：确认**真机可以纯 Python 控制，不必用 Simulink**。官方 `pal.products.qarm.QArm.read_write_std()` 在 Position 模式下下发关节角和夹爪指令，同时回读位置、速度、电流(A)、PWM、温度，默认 500 Hz；`pal` 不支持 PWM/Current 模式，Simulink 需要 QUARC 许可证。Python 依赖 Quanser SDK（含 `quanser.hardware`），**官方只提供 Windows 和 Linux 版**；macOS 只有 beta 版的 C 语言 HIL SDK（列出了 QArm，但没有 Python API）→ 真机要接在 Windows/Linux 电脑上。
- 2026-10-22：**基础环境搭好**（为 10-23 上真机做准备）：
  - 模型：`scripts/model/simplify_meshes.py` → `build_qarm_urdf.py` → `models/qarm/urdf/{qarm,qarm_gripper}.urdf`（官方真实惯量已从 DH 坐标系换算到 URDF 连杆坐标系，换算残差 0.44 mm；手册限位；TCP；夹爪）。`render_model_check.py` 渲染检查通过：MuJoCo 里 TCP @home = (0.45, 0, 0.4898)。
  - 代码包 `src/qarm_lab`：spec / kinematics（DH + 解析 IK，2000 组随机测试误差 1e-13）/ profile（Dynamixel 梯形 profile）/ servo（PWM-PID + 反电动势阻尼的舵机模型）/ dynamics（重力回归矩阵）/ trajectory（生成器 + 离线检查）/ safety / logio / runner / backends（`pal` 真机、`genesis` 仿真、`fake` 假后端）。`pytest tests` 共 11 项全部通过。
  - 明天的轨迹集：`data/trajectories/hwday_20261023/`（15 条，12.2 min，全部通过检查）。在 4090 的 Genesis（CPU）上整套干跑**0 次中止**，最大跟踪误差 1.6°，最大电流 1.6 A。
  - 工具：`scripts/run.py`（真机/仿真共用入口）、`scripts/sim/replay_in_sim.py`（真机日志放进仿真重放并对比）、`scripts/analysis/fit_gravity.py`（重力 + 用砝码标定 Kt）、`scripts/hw/check_env.py`。
  - 上机清单：`docs/hardware_day_20261023.md`。
  - **Quanser SDK 已装到 4090**（23:01–23:08，用户输入 sudo 密码）：33 个 apt 包（quanser-sdk 1.26.133、python3-quanser-apis、Quanser 自带的 librealsense2 2.49.0），udev 规则 `77-quanser.rules`，驱动 `/usr/lib/x86_64-linux-gnu/quanser/hardware/qarm_usb.so`。`check_env.py` 全部 ALL OK；`qarm` 环境里的 `quanser` 来自 `~/.local/share/qarm_quanser_site`，numpy 仍是环境自己的 2.5.4。安装日志：`logs/install_quanser_sdk_20261022_230140.log`。
  - 用最终代码做的正式干跑：`results/sim_runs/20261022_221039_genesis_dryrun_final/`（15 条，0 次中止）。工具自检：用仿真日志重放，RMSE ≤ 0.06°（`results/replay_selftest_20261022/`）；用仿真数据做重力拟合，残差从 0.07 降到 0.02 N·m（`results/sysid_20261022_221213/`）。

## 已知坑 / ★
- ★ git push 报 "Repository not found"：本机 SSH key 绑定的是 kefusong 账号，访问不到 Jasonartieth 的私有仓 → remote 必须用 HTTPS（`https://github.com/Jasonartieth/course_project.git`，走 gh 凭据）。`gh repo create --source=. --push` 默认用 SSH，会失败，需要先 `git remote set-url` 改成 HTTPS。（**4090 例外**：它的 SSH key 就是 Jasonartieth，可以直接 `git@github.com:` 克隆和推送。）
- ★ 官方 QARM.urdf 不能直接用于物理仿真：① 所有关节 effort/velocity=0；② 惯性参数是占位值（文件头注明）；③ mesh 路径 `package://QARM/` 和目录 `qarm`、package 名 `QArm` 大小写不一致；④ 夹爪是单独的 URDF，限位全为 0、没有连到臂上；⑤ 没有 TCP，需在 END-EFFECTOR 后加固定关节 xyz=(0,0,0.162) 才能与官方 FK/IK（λ3=0.40）对齐（已实测，误差 <1 mm）；⑥ 腕关节限位 ±172.5°，手册为 ±160°。已由 `scripts/model/build_qarm_urdf.py` 全部修复。
- ★ 官方 mesh 面数太多（底座 21 万、夹爪底座 46 万）→ MuJoCo 报 "number of faces should be between 1 and 200000"。解法：`simplify_meshes.py` 生成视觉 mesh（≤5 万面，最大误差 6.8 mm）和碰撞凸包；视觉和碰撞文件用不同的后缀 `__vis`/`__col`。
- ★ 用 MuJoCo 检查 URDF 时有三个隐蔽的坑：① URDF 解析器默认 `discardvisual=true`，在解析**之前**就把视觉几何丢掉了，解析之后再改 `spec.compiler` 没用，必须把 `<mujoco><compiler discardvisual="false" fusestatic="false"/></mujoco>` 注入 URDF 文本；② `fusestatic` 会把固定关节连接的连杆（tcp、gripper_base）合并掉；③ 名字找不到时 `mj_name2id` 返回 −1，`d.xpos[-1]` 会静默取到最后一个 body → 一定要 assert id ≥ 0。
- ★ Genesis 会自动合并固定连杆（base_link→world，tcp 和 gripper_base→END-EFFECTOR）→ TCP 要用 END-EFFECTOR 的位姿加 `spec.URDF_EE_TO_TCP` 计算（`GenesisBackend.tcp_position()`）。
- ★ 4090 的 GPU 暂时不可用：`nvidia-smi` 报 "Driver/library version mismatch"，torch 报 CUDA error 804。根因：自动更新把用户态库升到 580.178，但内核模块还是 580.173（5 周没重启）。解法：重启（需要用户的 sudo）；在此之前 Genesis 用 `--device cpu`（4 自由度的臂约 3 倍实时，够用）。
- ★ Quanser `pal` 的行为：出错只 `print` 不抛异常（要检查 `status` 和读数）；`measJoint*` 是同一块内部缓冲区的引用（记日志必须复制）；`readMode=1` 是硬件时钟定时读取；打开设备时会写入 PID（P 8.89 / I 0.012 / D 10.23），但不下发位置。
- ★ 打开 `qarm_usb` 报 **HILError −108**（"kernel-level driver ... could not be found"）= 没检测到机械臂硬件（2026-10-22 没接臂时实测就是这个错误）。连上之后如果还报这个，就查 USB 线、电源、USB 口，以及 `lsusb` 有没有出现新设备。
- ★ Linux 上 Quanser 的 Python 接口是 apt 包 `python3-quanser-apis`，装在系统 Python 里 → 通过符号链接目录加 .pth 文件**只把 `quanser` 包**暴露给 conda 环境（`install_quanser_sdk.sh`），不要把整个系统 site-packages 加进去。
- ★ ProfileLimiter 不能用连续时间的制动律 √(2a|e|)：离散步进时刹车太晚，到达目标时速度会跳变（约 20 rad/s²）。改用离散制动律 v = −a·dt/2 + √((a·dt/2)² + 2a|e|) 后，只剩到达那一步 ≤ 2a·dt 的残余速度（有测试覆盖）。
- ★ SLEEP 位姿（肩 −85°、肘 +75°）正好卡在关节限位上 → 从休息位出发的接近段检查必须允许 2° 容差（`runner._approach_ok`），否则仿真的微小漂移就会被判越界。
- ★ 远程跑真机必须防 SSH 断线：在 tmux 里跑，并且 `run.py` 已把 SIGHUP/SIGTERM 转成 Ctrl+C 的中止流程（原地保持 1 s → 存日志 → 关闭连接，已用假后端测试过）。
- ★ `quanser/Quanser_Academic_Resources` 的默认分支是 `dev-windows`，没有 `main`，用 `gh api .../git/trees/main` 会 404。
- ★ 本机 `/usr/bin/python3` 的 numpy 装在用户 site-packages 里，`python3 -I` 会 import 失败；需要隔离时，把下载文件和脚本分目录存放，再去掉 `-I` 运行。

## 待确认（明天真机实测）
- 夹爪安装偏移（模型里手指平面离腕轴约 15 mm）、手指闭合角度映射（`spec.GRIPPER_FINGER_CLOSE` 是近似值）
- pal 的 PID 单位（假设是 PWM 比例/rad）、舵机的 armature 和摩擦（`servo.ServoParams` 默认值是估计）、Kt
- 程序结束（terminate）后机械臂是保持力矩还是变软

## 归档索引（任务 → 仓库/分支/tag/路径）
