# Quanser QArm 调研报告（2026-10-22）

> 目的：为大作业摸清 QArm（4 自由度）的公开资料，包括官方/开源库、URDF、电机参数，以及能直接用于仿真的资源。
> 范围：只读调研，没有克隆任何仓库。运动学交叉验证脚本见 `scripts/research/fk_crosscheck_urdf_vs_dh.py`，输出见 `results/research_2026-10-22/fk_crosscheck_output.txt`。

---

## 0. 结论速览

| 问题 | 结论 |
|---|---|
| 有没有官方 URDF？ | **有**。[`quanser/urdf_representations`](https://github.com/quanser/urdf_representations)，**BSD-3**，含 `qarm`（机械臂）和 `qarm_gripper`（夹爪）两个包及 STL mesh。MATLAB Robotics System Toolbox 自带的 `quanserQArm.urdf` 也是同一份。 |
| 能不能直接扔进仿真？ | **几何和运动学可以直接用**：已验证与官方 DH 一致。**物理仿真前必须修**：effort/velocity 全为 0，惯性参数是占位值，mesh 的 package 路径大小写不一致，夹爪没装上且关节限位为 0，没有 TCP 坐标系。见 §6。 |
| 有没有公开电机参数？ | **有**。官方 user manual 写明：底座、肩、肘用 Dynamixel **XM540-W270-R**（肩部**双电机**），腕用 **XM430-W350-R**，夹爪用 **XC430-W240-T**。手册 Table 7 给出瞬时/持续电流与力矩，并与 ROBOTIS 官方 e-Manual 对得上。见 §2。 |
| 有没有真实动力学参数？ | **有**。官方 `7. Dynamic Parameters.pdf` 给出 4 个连杆的质量、质心、惯量（见 §4），可替换 URDF 里的占位值。 |
| 官方软件栈 | Python `pal`/`hal` 库（BSD-3；FK/IK/微分运动学是纯 NumPy；接硬件需要 Quanser SDK）、QUARC/Simulink、ROS1 与 ROS2 示例，以及 QLabs 虚拟孪生（仅 Windows，付费订阅）。 |
| 社区现成仿真 | PyBullet：`aj-floater/qarm-hack`；Genesis：`Georgiaac3/qarm-rl`，含**合并好的臂+夹爪 URDF**；ROS2 Jazzy + RViz + Isaac USD：`ENTECH-Industrial-Solution/quanser_ros2`。**没有找到现成的 MuJoCo 模型**。 |

---

## 1. 基本规格（官方 Data Sheet 与 User Manual）

| 项 | 值 |
|---|---|
| 构型 | 4 DOF，roll-pitch-pitch-roll（底座偏航 / 肩俯仰 / 肘俯仰 / 腕滚转） |
| 臂重 | 8.25 kg |
| 负载 | 350–750 g（350 g 可连续工作；500 g 约 25 min；750 g 约 15 min，室温） |
| 臂展 | 750 mm（Data Sheet）；手册写"水平 0.800 m、离工作面竖直 0.940 m" |
| 重复定位精度 | ±0.05 mm |
| 关节范围 | 底座 ±170°，肩 ±85°，肘 −95°/+75°，腕 ±160°，夹爪 0–100% |
| 最大关节速度 | ±π/2 rad/s（90°/s） |
| 最大关节加速度 | ±π/3 rad/s²（60°/s²） |
| 相机 | Intel RealSense D415（固定在腕前，**不随腕转动**；最小深度 0.16 m） |
| 接口 | USB（QFLEX 2）；扩展 I/O：PWM/Analog/I²C/SPI/UART |
| 控制 | 外部控制频率 500 Hz，内部 ≥1000 Hz；写入模式：Position / PWM / Current / Limp |

> 注意：同名的 **QArm Mini** 是另一款产品（臂展 382 mm），资料不要混用。从照片看，你们用的是标准 QArm。

---

## 2. 电机 / 执行器参数（user 的追问）

### 2.1 型号（QArm User Manual §E）

| 关节 | 电机 | 备注 |
|---|---|---|
| 底座 Base (yaw) | Dynamixel **XM540-W270-R** | |
| 肩 Shoulder | **2 × XM540-W270-R** | 双电机并联，力矩翻倍 |
| 肘 Elbow | Dynamixel **XM540-W270-R** | |
| 腕 Wrist | Dynamixel **XM430-W350-R** | |
| 夹爪 Gripper | Dynamixel **XC430-W240-T** | 单电机通过腱绳驱动两指（每指 2 节），欠驱动，2–5 个接触点 |

所有关节都是 12-bit 磁编码器（4096 counts/rev），带速度、电流和温度反馈。RS-485 总线不对外开放，只能通过 QFLEX 2 / Quanser HIL 访问。

### 2.2 QArm 关节级限额（User Manual Table 7）

> 瞬时值持续时间**不得超过 0.1 s**；电机温度不得超过 50 °C。

| 项 | 底座 | 肩 | 肘 | 腕 | 夹爪 |
|---|---|---|---|---|---|
| 瞬时电流 (A) | 4.4 | 8.8 | 4.4 | 2.3 | 1.4 |
| 持续电流 (A) | 1.1 | 2.2 | 1.1 | 0.6 | 0.35 |
| **瞬时力矩 (N·m)** | **10.6** | **21.2** | **10.6** | **4.1** | 1.9 |
| **持续力矩 (N·m)** | **2.65** | **5.3** | **2.65** | **1.0** | 0.5 |
| 最大速度 | π/2 rad/s | π/2 | π/2 | π/2 | ±100 %/s |
| 最大加速度 | π/3 rad/s² | π/3 | π/3 | π/3 | ±400 %/s² |

规律：瞬时值等于单个舵机在 12 V 下的**堵转值**（肩部 ×2），持续值约为堵转值的 1/4。

### 2.3 舵机本身的数据（ROBOTIS e-Manual，12 V 工况）

| 型号 | 减速比 | 堵转力矩 | 堵转电流 | 空载转速 | 重量 |
|---|---|---|---|---|---|
| XM540-W270-R | 272.5 : 1 | 10.6 N·m | 4.4 A | 30 rpm（≈3.14 rad/s） | 165 g |
| XM430-W350-R | 353.5 : 1 | 4.1 N·m | 2.3 A | 46 rpm（≈4.82 rad/s） | 82 g |
| XC430-W240-T | 245.22 : 1 | 1.9 N·m | 1.4 A | 70 rpm | 65 g |

- 舵机空载转速高于 QArm 的 π/2 rad/s 上限。QArm 的速度上限是 Quanser 固件的 profile 设定，不是电机极限。
- **没有公开的**：转子惯量、反电动势/力矩常数的完整曲线（XC430 页面给了 Nm/A：12 V 时 1.357）、关节摩擦。做精细动力学仿真时，MuJoCo `armature`（转子惯量 × 减速比²）这类参数只能估计或辨识。

### 2.4 底层控制器参数（QUARC `qarm_usb` 文档与官方 `pal` 库）

- `pal.products.qarm.QArm` 默认 board options：每个关节 `profile_velocity = 1.5708 rad/s`，`profile_acceleration = 1.0472 rad/s²`（即 90°/s、60°/s²）。
- Position 模式下可读写 P/I/D 和前馈增益（QUARC 属性 128–151）。夹爪位置指令是 0.0（开）到 1.0（合），`pal` 里被限幅到 0.1–0.9。
- 电流反馈：模拟输入通道 5–9（底座、肩、肘、腕、夹爪），单位 A。

### 2.5 填进仿真的建议

| 用途 | 建议值 |
|---|---|
| URDF `<limit velocity>` | 四个关节都填 **1.5708** |
| URDF `<limit effort>` / 执行器力矩饱和 | 底座/肩/肘/腕 = **10.6 / 21.2 / 10.6 / 4.1**（瞬时上限）；做"长时间持续负载"评估时，再用持续值 2.65 / 5.3 / 2.65 / 1.0 检查 |
| 轨迹约束 | 速度 ≤ π/2 rad/s，加速度 ≤ π/3 rad/s² |

---

## 3. 运动学

### 3.1 官方 DH 参数（User Manual Table 2，`hal/products/qarm.py`，标准 DH）

L1 = 0.14, L2 = 0.35, L3 = 0.05, L4 = 0.25, L5 = 0.15（m）；β = atan(L3/L2) = 8.13°；
λ1 = L1 = 0.14，λ2 = √(L2² + L3²) = 0.3536，λ3 = L4 + L5 = 0.40。

| i | a | α | d | θ |
|---|---|---|---|---|
| 1 | 0 | −π/2 | λ1 | φ1 |
| 2 | λ2 | 0 | 0 | φ2 + β − π/2 |
| 3 | 0 | −π/2 | 0 | φ3 − β |
| 4 | 0 | 0 | λ3 | φ4 |

Home 位姿 φ = 0 时，末端位置为 (0.45, 0, 0.49) m。`hal.products.qarm.QArmUtilities` 提供解析 IK 和雅可比，是纯 NumPy 实现，不依赖硬件，可以直接拿来用。

### 3.2 官方 URDF 与 DH 的交叉验证（本次实测）

脚本：`scripts/research/fk_crosscheck_urdf_vs_dh.py`

- 4 组关节角下，URDF 和 DH 的关节方向、零位、几何关系**完全一致**。
- URDF 的 `END-EFFECTOR` 坐标系原点比 DH 的 λ3 端点**短一个固定偏移**：在 `END-EFFECTOR` 局部坐标系里是 **(0, 0, 0.162) m**，各位姿误差都小于 1 mm。
- **结论**：在 URDF 末尾加一个固定关节 `tool_joint`（xyz = "0 0 0.162"），得到的 TCP 就和官方 FK/IK 对齐。

### 3.3 三处关节限位对照（单位：度）

| 来源 | 底座 | 肩 | 肘 | 腕 |
|---|---|---|---|---|
| Data Sheet / Manual / `pal` LIMITS | ±170 | ±85 | −95/+75 | ±160 |
| 官方 URDF | ±170 | ±85 | −95/+75 | **±172.5** |
| `hal` IK 的 `_check_joint_limits` | ±170 | **±80** | −95/+75 | ±160 |

建议以手册值为准，把 URDF 腕关节改成 ±2.793 rad。

---

## 4. 动力学参数（官方 `7. Dynamic Parameters.pdf`）

简化模型（质心在主轴上，均匀圆柱体）：

| 连杆 | 质量 (kg) | 质心距离 λc (m) | 轴向惯量 I_A (kg·m²) | 横向惯量 I_L (kg·m²) |
|---|---|---|---|---|
| B1 | 0.7906 | 0.0399 | 1.489e-3 | — |
| B2 | 0.4591 | 0.1071 | 1.922e-4 | 9.610e-3 |
| B3 | 0.269 | 0.1561 | 2.679e-4 | 2.069e-3 |
| B4 | 0.257 | 0.0998 | 6.528e-4 | 1.120e-3 |

PDF 还给出了原始的完整质心向量和对角惯量，可以直接用于更精确的模型。B1–B4 对应 DH 坐标系 1–4 的连杆（偏航座、上臂、前臂、腕部）。B4 是否含夹爪，正文没有明说，需要看 PDF 图 2 核实。

**对比 URDF 占位值**：YAW 0.70，BICEP 0.71，FOREARM 0.43，END-EFFECTOR 0.025 kg。URDF 文件头也自己注明了"mass/inertial values are example values only"，所以**做动力学或力矩控制仿真前必须替换**。注意 URDF 的 link 坐标系和 DH 坐标系不同，惯量需要做坐标变换后才能填。

---

## 5. 开源资源清单

### 5.1 官方（Quanser）

| 仓库 / 资源 | 许可 | 内容 | 用途评价 |
|---|---|---|---|
| [quanser/urdf_representations](https://github.com/quanser/urdf_representations) | BSD-3 | `qarm/`：QARM.urdf + 5 个 STL + ROS1 catkin 包 + RViz/Gazebo launch；`qarm_gripper/`：夹爪 URDF + STL | ★ **仿真模型的源头**；2025-05 发布，QArm 部分之后没有更新 |
| [quanser/Quanser_Academic_Resources](https://github.com/quanser/Quanser_Academic_Resources)（默认分支 `dev-windows`） | BSD-3 | `0_libraries/python/{pal,hal}`：硬件驱动与 FK/IK；`3_user_manuals/qarm/`：用户手册 + 7 份运动学/动力学讲义 PDF；`5_research/qarm/`：Python/Simulink 基础 IO、RGBD、**ROS1 包**（action server/client）、**ROS2 示例**（`qarm_nodes`、`qarm_interfaces` + 指南 PDF）、动力学模型 .slx、迭代 IK、增益调度、HD2 遥操作 | ★ **最重要的资料库**；讲义 PDF 正好对应大作业要用的理论 |
| [quanser/Quanser_Interactive_Labs_Resources](https://github.com/quanser/Quanser_Interactive_Labs_Resources) | 无 | QLabs 的 Python/MATLAB API（`qvl/qarm.py`）、`.rt-win64` 模型 | 只在使用 QLabs 时需要 |
| QLabs Virtual QArm | 商业订阅 | "动力学准确"的虚拟孪生 | **仅 Windows 10+**，12 个月多席位订阅，需要询价；学校如果已购买，可以作为最接近真机的仿真 |
| [MathWorks File Exchange #125250](https://www.mathworks.com/matlabcentral/fileexchange/125250-quanser-qarm-pick-and-place-using-mathworks-toolboxes) | MathWorks | 用 Robotics/Simscape 工具箱做 pick & place（Simscape 模型 / QLabs / 真机） | 走 MATLAB 路线时参考 |
| [Quanser 博客：URDF → Simscape](https://www.quanser.com/blog/robotics-haptics/leveraging-matlab-simulink-toolboxes-to-rapidly-deploy-a-pick-place-application/) | — | 用 `smimport` 导入 URDF；指出 URDF 不含夹爪 | MATLAB 路线参考 |

### 5.2 社区

| 仓库 | 许可 | 内容 | 评价 |
|---|---|---|---|
| [ENTECH-Industrial-Solution/quanser_ros2](https://github.com/ENTECH-Industrial-Solution/quanser_ros2) | 包内 LICENSE 为 Quanser BSD-3，仓库根目录无 | 把 Quanser ROS2 示例改到 **ROS2 Jazzy**；`qarm_sim.py` 运动学仿真节点 + RViz；关节已改名并带 `tool_frame`；附 **USD（Isaac Sim，含 physx/mujoco physics schema）**；README 写了 Ubuntu 下 `apt install quanser-sdk` 的依赖坑 | ★ 走 ROS2 路线的最好起点（需要 Linux） |
| [Georgiaac3/qarm-rl](https://github.com/Georgiaac3/qarm-rl) | **无许可证** | Genesis 仿真 + gym 环境；**`qarm_with_gripper.urdf`**：夹爪已固定到末端，夹爪关节限位 ±0.9 rad / 10 N·m / 5 rad/s | 合并 URDF 的做法值得参考；没有许可证，**不宜直接拷贝** |
| [aj-floater/qarm-hack](https://github.com/aj-floater/qarm-hack)（及 fork：FHL-08/quanser-hackathon、bcaul/Quanser-Arm） | **无许可证** | 黑客松框架：**PyBullet** 仿真 + Panda3D 可视化 + 仿真/真机统一 API；docs 里有 Data Sheet 和 URDF 入门 | 结构清晰，可借鉴"sim/real 统一接口"的设计；没有许可证 |
| [PieroJF/Robot-qarm-fruit-sorting](https://github.com/PieroJF/Robot-qarm-fruit-sorting) | MIT | 真机 + RealSense 水果分拣：手眼标定、检测、抓取 | 视觉抓取类作业的优秀参考（无仿真） |
| [Heartpray3/Quanser-QArm-RL](https://github.com/Heartpray3/Quanser-QArm-RL) | MIT | MATLAB TD3 强化学习，QLabs + 真机 + YOLO | MATLAB/RL 路线参考 |
| [vanshs64/quanser-qarm](https://github.com/vanshs64/quanser-qarm) | BSD-3 | ME547 课程：Python FK/IK/微分运动学 | 课程作业风格参考 |

**没找到**：现成的 MuJoCo MJCF、MoveIt 配置（SRDF）、ros2_control/Gazebo（gz）描述。

---

## 6. 直接用官方 URDF 的坑与修复清单

| # | 问题 | 后果 | 修复 |
|---|---|---|---|
| 1 | 所有关节 `effort="0" velocity="0"` | Gazebo/ros2_control 会把力矩限为 0；MoveIt 速度规划失效 | 按 §2.5 填写 |
| 2 | 惯性参数是占位值（文件头注明） | 动力学和力矩控制结果失真 | 用 §4 的官方参数替换（注意坐标系变换） |
| 3 | mesh 路径写的是 `package://QARM/meshes/…`，但目录叫 `qarm`，`package.xml` 里的 name 是 `QArm` | Linux 区分大小写，ROS 找不到包；PyBullet/MuJoCo 不认 `package://` | 统一包名，或把路径改成相对路径 `../meshes/…` |
| 4 | 夹爪是单独的 URDF，关节限位全为 0，没有 mimic，也没有和臂连接 | 夹爪无法动作 | 加 `gripper_mount` 固定关节（参考 Georgiaac3 的做法），设限位，并用 mimic/等式约束近似腱绳联动；或者一开始先只用"抓取 = 吸附/焊接约束"简化 |
| 5 | 没有 TCP | IK 目标点和官方 FK 对不上 | 加 `tool_joint` xyz = 0 0 0.162（§3.2） |
| 6 | STL 很大（底座 10.5 MB，夹爪底座 23 MB），碰撞体直接用了视觉 mesh | 碰撞检测慢 | 碰撞体换成凸包或基本几何体 |
| 7 | 腕关节限位 ±172.5° | 和手册不一致 | 改为 ±160° |
| 8 | 包是 ROS1 catkin 格式，launch 也是 ROS1 | ROS2 不能直接用 | 用 ENTECH 的 ROS2 版本，或者自己改成 ament |
| 9 | 4 自由度只能控制 位置 + 偏航 | MoveIt 默认 6D IK 会求解失败 | IK 用 position-only，或直接用 `hal` 的解析 IK |

---

## 7. 仿真路线对比（待按作业要求选定）

| 路线 | 平台 | 优点 | 缺点 |
|---|---|---|---|
| **MuJoCo**（官方 URDF 修好后转 MJCF） | macOS/Linux/Windows，`pip install mujoco` | 物理好、快，适合控制和 RL，Mac 原生可用 | 没有现成模型，需要自己转换并加执行器（工作量约半天到一天） |
| **PyBullet** | 全平台 | `loadURDF` 一行加载；qarm-hack 可以作参考 | 维护放缓，渲染一般 |
| **ROS2 Jazzy + RViz（+ MoveIt / Gazebo）** | Ubuntu 24.04 | ENTECH 包能直接跑，与真机接口同构 | Mac 上需要虚拟机或 Docker；MoveIt 需要自己配 |
| **MATLAB Simscape / Robotics Toolbox** | 全平台 MATLAB | 官方资料最多（讲义、slx、File Exchange） | 需要许可证 |
| **QLabs Virtual QArm** | 仅 Windows | 最接近真机，代码和真机通用 | 付费订阅 |
| Isaac Sim / Genesis | NVIDIA GPU / 全平台 | 有现成 USD 或 URDF | 门槛高或偏小众 |

如果作业以控制、运动学、轨迹规划为主，并且在 Mac 上开发，**默认推荐 MuJoCo**：官方 URDF 修好后转 MJCF，配合 `hal` 的 FK/IK 和 §2 的执行器参数。如果作业要求 ROS 或要上真机，走 **ROS2 + ENTECH 包**。最终选哪条，取决于作业要求。

---

## 8. 未核实 / 待确认

- MATLAB `loadrobot` 是否能直接按名字加载 QArm：从 MathWorks Robot Library 数据包里能找到 `quanserQArm.urdf`，但没有确认确切的 robot 名。
- `hil_sdk_macos` 是否支持 `qarm_usb` 板卡（即能否在 Mac 上直连真机）：未确认。
- `Dynamic Parameters.pdf` 里 B4 是否包含夹爪质量：需要看图 2。
- 舵机转子惯量、摩擦参数：没有公开。

---

## 9. 来源

- Quanser QArm 产品页：<https://www.quanser.com/products/qarm/>
- QArm Data Sheet（2020）与 QArm User Manual：`Quanser_Academic_Resources/3_user_manuals/qarm/qarm_user_manual.pdf`
- QArm 讲义 `7. Dynamic Parameters.pdf`、`6. Dynamics - Torque Equations.pdf`（同上目录 `ancillaryMaterial/`）
- QUARC `qarm_usb` 文档：<https://docs.quanser.com/quarc/documentation/qarm_usb.html>
- ROBOTIS e-Manual：[XM540-W270](https://emanual.robotis.com/docs/en/dxl/x/xm540-w270/)、[XM430-W350](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/)、[XC430-W240](https://emanual.robotis.com/docs/en/dxl/x/xc430-w240/)
- QLabs Virtual QArm：<https://quanser.com/products/qlabs-virtual-qarm>
- 其余 GitHub 仓库链接见 §5
