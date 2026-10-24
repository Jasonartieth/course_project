# course_project — 项目说明

- 创建日期：2026-10-22
- GitHub：Jasonartieth/course_project（**2026-10-23 起公开**：https://github.com/Jasonartieth/course_project）。公开仓库里不许出现真实姓名、个人邮箱、IP 或密钥（见「已知坑」里的 git 身份条目）。
- 主题：基于 Quanser **QArm**（4 自由度，roll-pitch-pitch-roll）的大作业；**没有给定作业要求，任务由用户自定**。当前阶段是搭基础环境，并做真机 real2sim。
- 仿真器：**Genesis**（用户指定）。运行机器是 **4090**（`ssh 4090`，Tailscale <tailscale-ip>，用户 jcy，Ubuntu 24.04，Alienware 笔记本）；这台电脑也会被带去接真机，从 Mac 远程控制。
- Mac 只负责写代码和离线分析（`.venv`，uv，Python 3.12；装了 MuJoCo，只用来渲染检查 URDF）。

## 目录约定
- 代码 → `src/qarm_lab/`（包）、`scripts/`（入口）、`tests/`；数据 → `data/`（`trajectories/`、`real/`）；产物 → `results/<类别>_<时间戳>/`；日志 → `logs/`；文档 → `docs/`；环境 → `env/`；模型 → `models/`（`qarm_official/` 为官方原件**不得修改**，`qarm/` 为派生版本，由脚本生成）
- 不再使用但不删的大文件 → `_backup/<任务名>_<日期>/`（不进 git，仓库 archive 里留清单+路径指针）
- 仿真日志 `results/sim_runs/**/log.npz`、`results/replay_*/**/log.npz` 不进 git（可重新生成，留在磁盘上）；**真机数据 `data/real/` 全部进 git**。

## 两台机器的工作流
- Mac 是主力编辑端：改代码 → commit → push（Mac 用 HTTPS remote）。
- 两台机器都在仓库本地设置了 git 身份 `Jasonartieth <178580748+Jasonartieth@users.noreply.github.com>`（`git config user.*`，不是 --global）。新 clone 下来的副本要先设好这个身份再提交。
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

- 2026-10-24：**检查碰撞几何**（`scripts/model/collision_audit.py`，结果在 4090 的 `results/collision_audit_20261024_*`）。对象：队友仓库 LuoShaoyinn/mmic-project 的最新 URDF（2b8ab95，Georgiaac3 社区版改写）和我们自己的 URDF。结论：Genesis 默认把**每个连杆的碰撞 mesh 压成一个凸包**（≤500 面），夹爪底座和 END-EFFECTOR 合成一个大凸包，把两指之间的空间填满了。mmic 版：缺少材质名，直接加载失败；mesh 超过 20 万面，退回旧版解析器，加载 36 s；手指闭合时互穿；`Link1B` 质量为 0；力矩和速度上限为 0。（**更正**：起初我判断是"B 侧坐标系没有镜像"，这是**错的**。数值分析 `scripts/research/gripper_geom_mmic_2b8ab95.py`（图在 `results/research_2026-10-24/`）显示两指运动严格镜像；真正原因是闭合系数 (−0.45, −0.35, 0.45, −0.35) 太大：grip≈0.45 时指尖已在 TCP 附近（y=0.165）合拢，grip=1 时手指穿过对方。）我们这版：能加载，5 s，参数是真实值，但**夹爪手指零件是飘着的**（官方夹爪 URDF 的连杆坐标系有问题；mmic 版重新对齐后的视觉坐标系是对的）。mmic 的 10 个 mesh 和 Quanser 官方原件逐字节一致（BSD-3）。用户决定：在 Jasonartieth 的 fork 上修，然后提 PR 给 LuoShaoyinn。**碰撞体只用立方体、圆柱、球，必须完全包住视觉模型（宁大勿小），不用凸包**。
- 2026-10-24：**v2 模型完成并已提 PR**：https://github.com/LuoShaoyinn/mmic-project/pull/1（fork `Jasonartieth/mmic-project`，分支 `fix/qarm-collision-primitives`，提交 b6d3425，只改 `assets/qarm-community/`）。
  - 生成：`scripts/model/build_qarm_v2.py` → `models/qarm_v2/`（URDF、减面后的视觉 mesh、README、LICENSE、collision/gripper 报告）。碰撞体来自 `src/qarm_lab/collision_primitives.py`：先细分长边（≤5 mm），再自顶向下贪心二分，每块在立方体、圆柱、球中取体积最小的；一共 43 个原始体，**官方全精度顶点 100% 被包住**；总体积是凸包的 0.63–1.44 倍（YAW 0.63，FOREARM 0.79）。
  - 手指：沿用 mmic 的坐标系；闭合限位取两指视觉 mesh 刚好接触时的角度（近端 ±0.1616，远端 −0.1257 rad）；推荐映射 grip·(−0.1616, −0.1257, 0.1616, −0.1257)；此时指尖合拢在 y=0.168（TCP 0.162）。
  - 验证（4090，Genesis 1.4.4）：加载 5.4 s、主解析器、无质心警告、零位无自碰撞；开碰撞和重力后保持误差 0.23°；往手心、A 指、B 指上丢方块，都落在目标连杆上；mmic 的 `gripper_demo.py` 原样能跑。结果在 `results/collision_audit_*_qarm_v2/`、`results/verify_qarm_v2_20261024/`。
  - 4090 上的目录：`~/external/mmic-project`（克隆，remote `fork` 指向我们的 fork，本地 git 身份是 noreply）、`~/external/mmic_audit`（只补了材质名的副本）、`~/external/mmic_v2_test`（替换成 v2 资源的测试副本）。
  - 我们自己的 `models/qarm/`（v1）夹爪仍然是飘着的，保留但不再作为默认。
- 2026-10-24：**按用户要求把 mmic PR #1 改成只改 collision**（"不要新建 STL，只改 URDF，原始体写在 URDF 里，只改 collision 部分"）。工具 `scripts/model/replace_collisions_with_primitives.py`：在文本层面原地替换 `<collision>` 块，自检"去掉 collision 块后文本完全相同"，并把写出的 URDF 读回来做覆盖验证（视觉顶点 100% 在原始体里）。PR 现在只有 1 个文件、1 个提交 812ca51（+243/−35）：43 个原始体（35 立方体、7 圆柱、1 球），不增删 STL。效果：Genesis 主解析器只加载 collision 几何，所以碰撞改成原始体后**就不再退回旧版解析器**（加载 36 s → 7.3 s），即使视觉 mesh 还是全精度；保持误差 0.43°，丢方块测试通过。PR 描述里列出了不在本次范围内的问题（材质缺名字所以仍然加载不了、手指映射过大、占位惯量）。结果：`results/collision_primitives_mmic_2b8ab95_20261024_r2/`、`results/collision_audit_*_mmic_collision_only_primitives/`、`results/verify_mmic_collision_only_20261024/`。
- 2026-10-24：**course_project 的 Genesis 后端默认改成 v2**（`backends/sim_genesis.DEFAULT_URDF = models/qarm_v2/qarm_with_gripper.urdf`）；`spec.GRIPPER_FINGER_CLOSE = (−0.1616, −0.1257, 0.1616, −0.1257)`，由测试保证和 `models/qarm_v2/gripper_report.json` 一致；`render_model_check.py` 默认渲染 v2，并隐藏原始体碰撞几何。pytest 13 项通过（Mac 和 4090）。冒烟测试：TCP 误差 0.2 mm，肩关节阶跃 1.55°（和 v1 相同）。整套干跑 `results/sim_runs/20261024_165419_genesis_dryrun_v2/`：15 条、0 次中止；和 v1 比（`results/compare_20261024_165922_v1_vs_v2/`，工具 `scripts/analysis/compare_sessions.py`），最大差异 0.01° / 0.01 A。原因：手臂惯量相同，原始体碰撞体在这些轨迹里不引起额外接触。

## 已知坑 / ★
- ★ 官方夹爪 URDF（以及我们由它生成的 `qarm_gripper.urdf`）的手指连杆坐标系有问题：渲染出来 Link1/Link2 零件和夹爪底座是分开的（`results/collision_audit_20261024_162654_course_project_qarm_gripper/visual_vs_collision.png`）。`spec.GRIPPER_FINGER_CLOSE` 只是在这个错误几何上目测出来的近似值，不要拿来当真。
- ★ **给别人的仓库提 PR 时，改动范围要最小**（用户 2026-10-24 的明确要求）：只改需要改的那部分（这次是 URDF 的 `<collision>`），不新增或替换 mesh 文件，不顺手修别的问题；顺手发现的问题写进 PR 描述作为提醒，由对方决定。PR 描述里的数字必须从实际产物里统计出来，不能凭印象写（这次差点写错原始体数量）。
- ★ Genesis 1.4.4 默认的 URDF 解析器（MuJoCo 那套）默认丢弃视觉几何、只加载碰撞几何 → 超过 20 万面的限制只对**碰撞**里引用的 mesh 生效；碰撞改成原始体后，即使视觉 mesh 是全精度的，也不会再退回旧版解析器。
- ★ 用原始体拟合 CAD mesh 时：① CAD 导出有横跨整个零件的细长三角形，一个面就能把它所在那块的包围盒拉满整个零件 → 必须先 `trimesh.remesh.subdivide_to_size` 细分长边；② k-means 聚类切得很差（分成多块后总体积反而更大）→ 改用自顶向下贪心二分；③ `trimesh.nsphere.minimum_nsphere`（qhull）遇到共面点集会报错，trimesh 处理退化凸包时需要 `networkx` → 加兜底（包围盒中心球 / 主成分方向包围盒），并装 networkx。
- ★ 测试碰撞时，不要把方块丢在张开的两指中间：两指内侧间距约 36 mm，3 cm 的方块本来就会从缝里掉下去，这不能说明碰撞失效。要丢在手心或手指本体上，并检查方块实际碰到了哪个连杆（`verify_qarm_genesis.py`）。
- ★ mmic 的 `gripper_demo.py` 用了 `recompute_inertia=True`，会抛开 URDF 里的惯量，用（不封闭的）mesh 凸包重新计算，所以"质量和惯量是否正确"不能用这个示例来验证。
- ★ 通过 `ssh 4090 '...'` 传提交说明时，英文撇号（`part's`）会截断外层单引号，导致 zsh 解析错误 → 先在本地写好提交说明文件，scp 过去，再 `git commit -F`。
- ★ Genesis 1.4.4 默认把机器人每个连杆的碰撞几何变成**一个凸包**（`decompose_robot_error_threshold` 默认不分解），凹形零件（夹爪手心、U 形支架）会被填实。需要凹形碰撞时，离线把它分解成多个凸块，作为多个 `<collision>` 写进 URDF。
- ★ git push 报 "Repository not found"：本机 SSH key 绑定的是 kefusong 账号，访问不到 Jasonartieth 的私有仓 → remote 必须用 HTTPS（`https://github.com/Jasonartieth/course_project.git`，走 gh 凭据）。`gh repo create --source=. --push` 默认用 SSH，会失败，需要先 `git remote set-url` 改成 HTTPS。（**4090 例外**：它的 SSH key 就是 Jasonartieth，可以直接 `git@github.com:` 克隆和推送。）
- ★ **公开仓库前必须清理提交身份，而且强制推送并不够**（2026-10-23）。现象：Mac 的默认 git 身份是「真实姓名 <用户名@主机名.local>」，我在 4090 上提交时用了用户的 gmail；用 `git filter-repo --mailmap/--replace-text` 改写并强制推送后，GitHub 仍然保留旧的提交对象，公开后按旧 SHA（连短 SHA 也行）访问照样返回 200，个人信息还在。解法：先改回私有 → 把旧仓库**改名**保留为私有（`course_project_prerewrite_20261023`）→ 新建同名的公开仓库 → 推送清理后的历史；验证旧 SHA 返回 422。以后：两台机器都用 noreply 身份；**不要用 `git -c user.email=<个人邮箱>` 提交**。
- ★ 官方 QARM.urdf 不能直接用于物理仿真：① 所有关节 effort/velocity=0；② 惯性参数是占位值（文件头注明）；③ mesh 路径 `package://QARM/` 和目录 `qarm`、package 名 `QArm` 大小写不一致；④ 夹爪是单独的 URDF，限位全为 0、没有连到臂上；⑤ 没有 TCP，需在 END-EFFECTOR 后加固定关节 xyz=(0,0,0.162) 才能与官方 FK/IK（λ3=0.40）对齐（已实测，误差 <1 mm）；⑥ 腕关节限位 ±172.5°，手册为 ±160°。已由 `scripts/model/build_qarm_urdf.py` 全部修复。
- ★ 官方 mesh 面数太多（底座 21 万、夹爪底座 46 万）→ MuJoCo 报 "number of faces should be between 1 and 200000"。解法：`simplify_meshes.py` 生成视觉 mesh（≤5 万面，最大误差 6.8 mm）和碰撞凸包；视觉和碰撞文件用不同的后缀 `__vis`/`__col`。
- ★ 用 MuJoCo 检查 URDF 时有三个隐蔽的坑：① URDF 解析器默认 `discardvisual=true`，在解析**之前**就把视觉几何丢掉了，解析之后再改 `spec.compiler` 没用，必须把 `<mujoco><compiler discardvisual="false" fusestatic="false"/></mujoco>` 注入 URDF 文本；② `fusestatic` 会把固定关节连接的连杆（tcp、gripper_base）合并掉；③ 名字找不到时 `mj_name2id` 返回 −1，`d.xpos[-1]` 会静默取到最后一个 body → 一定要 assert id ≥ 0。
- ★ Genesis 会自动合并固定连杆（base_link→world，tcp 和 gripper_base→END-EFFECTOR）→ TCP 要用 END-EFFECTOR 的位姿加 `spec.URDF_EE_TO_TCP` 计算（`GenesisBackend.tcp_position()`）。
- ★ 4090 的 GPU 曾经不可用：`nvidia-smi` 报 "Driver/library version mismatch"，torch 报 CUDA error 804。根因：自动更新把用户态库升到 580.178，但内核模块还是 580.173（5 周没重启）。**2026-10-22 23:09 重启后已修好**（内核 7.0.0-38，驱动 580.178.04，CUDA 13.0，torch 能识别 RTX 4090 Laptop 16 GB）。以后再出现同样报错，先看 `cat /proc/driver/nvidia/version` 和 `dpkg -l nvidia-driver-*` 的版本是否一致，不一致就重启。
- ★ Genesis 只跑**单个环境**时，GPU 反而比 CPU 慢（steps_shoulder 31.6 s 的仿真：GPU 23 s，CPU 10 s，结果完全一致）→ 单条运行或重放默认用 CPU；批量参数辨识或强化学习（n_envs 很大）时再用 `--device gpu`。GPU 离屏相机渲染已验证可用（`results/render_check_20261022/`）。
- ★ Quanser `pal` 的行为：出错只 `print` 不抛异常（要检查 `status` 和读数）；`measJoint*` 是同一块内部缓冲区的引用（记日志必须复制）；`readMode=1` 是硬件时钟定时读取；打开设备时会写入 PID（P 8.89 / I 0.012 / D 10.23），但不下发位置。
- ★ 打开 `qarm_usb` 报 **HILError −108**（"kernel-level driver ... could not be found"）= 没检测到机械臂硬件（2026-10-22 没接臂时实测就是这个错误）。连上之后如果还报这个，就查 USB 线、电源、USB 口，以及 `lsusb` 有没有出现新设备。
- ★ Linux 上 Quanser 的 Python 接口是 apt 包 `python3-quanser-apis`，装在系统 Python 里 → 通过符号链接目录加 .pth 文件**只把 `quanser` 包**暴露给 conda 环境（`install_quanser_sdk.sh`），不要把整个系统 site-packages 加进去。
- ★ ProfileLimiter 不能用连续时间的制动律 √(2a|e|)：离散步进时刹车太晚，到达目标时速度会跳变（约 20 rad/s²）。改用离散制动律 v = −a·dt/2 + √((a·dt/2)² + 2a|e|) 后，只剩到达那一步 ≤ 2a·dt 的残余速度（有测试覆盖）。
- ★ SLEEP 位姿（肩 −85°、肘 +75°）正好卡在关节限位上 → 从休息位出发的接近段检查必须允许 2° 容差（`runner._approach_ok`），否则仿真的微小漂移就会被判越界。
- ★ 远程跑真机必须防 SSH 断线：在 tmux 里跑，并且 `run.py` 已把 SIGHUP/SIGTERM 转成 Ctrl+C 的中止流程（原地保持 1 s → 存日志 → 关闭连接，已用假后端测试过）。
- ★ `quanser/Quanser_Academic_Resources` 的默认分支是 `dev-windows`，没有 `main`，用 `gh api .../git/trees/main` 会 404。
- ★ 本机 `/usr/bin/python3` 的 numpy 装在用户 site-packages 里，`python3 -I` 会 import 失败；需要隔离时，把下载文件和脚本分目录存放，再去掉 `-I` 运行。

## 待确认（明天真机实测）
- 夹爪几何（v2 用的是 mmic 2b8ab95 的坐标系）、手指闭合限位和映射（`spec.GRIPPER_FINGER_CLOSE` 是按"两指视觉 mesh 刚好接触"算出来的，没在真机上核实；需要拍下 grip=0.1/0.9 时手指的实际张角）
- pal 的 PID 单位（假设是 PWM 比例/rad）、舵机的 armature 和摩擦（`servo.ServoParams` 默认值是估计）、Kt
- 程序结束（terminate）后机械臂是保持力矩还是变软

## 归档索引（任务 → 仓库/分支/tag/路径）
- mmic PR 的上一版完整方案（26 个文件：减面视觉 mesh、真实惯量、手指限位、README/LICENSE，提交 b6d3425）→ fork `Jasonartieth/mmic-project` 的分支 `archive/qarm-v2-full-assets-20261024`；内容和 course_project 的 `models/qarm_v2/` 相同。
- 公开前的原始提交历史（含旧作者信息，19 个提交，HEAD 9fb4778）→ 本地 `_backup/history_before_public_20261023/course_project_pre_rewrite_9fb4778.bundle`（已校验，附 `commit_list_pre_rewrite.txt`）+ 私有仓 `Jasonartieth/course_project_prerewrite_20261023`（**用户 2026-10-23 决定保留，不删**；不要再建议删除或改成公开）。找回方法：`git clone <bundle> <dir>`。
