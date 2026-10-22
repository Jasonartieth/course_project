# 上机清单：QArm 首次连接 + real2sim（2026-10-23）

**分工**：你在机械臂旁，一只手放在**电源开关**上；我从 Mac 通过 Tailscale `ssh 4090` 远程执行命令。**每一步动真机之前，我都会在对话里先说明要做什么，等你回复"可以"再执行。**

---

## A. 今晚 / 出发前（4090 上，需要你本人操作）

| # | 做什么 | 命令 / 说明 |
|---|---|---|
| A1 | 安装 Quanser SDK + Python 接口（需要 sudo 密码） | 在 4090 终端执行 `bash ~/course_project/env/4090/install_quanser_sdk.sh`，最后应输出 `OK: quanser.hardware + pal.products.qarm import ...` |
| A2 | （建议）重启 4090 修复 NVIDIA 驱动 | `sudo reboot`。驱动库已升级到 580.178，但内核模块还是 580.173，所以现在 GPU 不可用。仿真用 CPU 也能跑，重启只为能用 GPU |
| A3 | 确认 Tailscale 开机自启 | 已确认 `tailscaled` 是 enabled。到实验室接上网络后，我这边 `ssh 4090` 能连上即可 |
| A4 | 称重砝码 | 准备 2 个已知质量（约 200 g 和 350 g，用厨房秤称到 1 g），能被夹爪夹住或用胶带固定在指尖 |

## B. 要带的东西

- 4090 笔记本和电源
- QArm 的电源、USB 线（臂到电脑）、**C 型夹或固定螺栓**（手册要求底座必须固定在桌面）
- 砝码（见 A4）、胶带、卷尺或钢尺、手机（拍照，加一个**水平仪 App**）
- 机械臂周围清出约 1 m 的空间，桌面上不要放杂物

## C. 现场连接（你来做，5 分钟）

1. 底座夹紧在桌面上；确认电源开关处于 **OFF**，再接电源线和 USB 线（USB 线接到 4090）。
2. 4090 联网，打开盖子保持唤醒（或接外接显示器）。在对话里告诉我"已连好"。
3. 我这边执行：`ssh 4090` → `tmux new -s qarm`（断线不会中断）→ `conda activate qarm` → `cd ~/course_project && git pull`。

## D. 执行顺序（每一步我都会先确认再执行）

所有真机数据都写进 `data/real/<时间戳>_pal[_tag]/`，不会覆盖之前的数据。

| 步 | 命令（在 `~/course_project` 下） | 机械臂动作 | 通过标准 |
|---|---|---|---|
| 0 | `python scripts/hw/check_env.py` | 不动 | `ALL OK`；`lsusb` 能看到 QArm |
| 1 | **打开电源**，然后执行 `python scripts/hw/check_env.py --open` | 不动 | 能读到角度、电流、温度，数值合理 |
| 2 | `python scripts/run.py --backend pal --read-only 5` | 不动 | 读到的角度和实际姿态一致（拍照对照）；温度接近室温；采样周期中位数约 2 ms |
| 3 | `python scripts/run.py --backend pal --hold-here 5` | **第一次上力矩**，在原地锁住 5 s | 不晃、不抖、电流合理。**特别观察：程序结束后机械臂是保持住还是变软下垂**（决定以后的收臂流程） |
| 4 | `--traj data/trajectories/hwday_20261023/00_hold_home.npz` | 以 25% 速度慢慢移到 HOME（前伸水平），保持 20 s | 前臂水平：手机水平仪放在前臂上读数接近 0° |
| 5 | `01_steps_base` → `02_steps_shoulder` → `03_steps_elbow` → `04_steps_wrist`（逐条执行） | 单关节 ±6°/±11° 阶跃 | 每条跑完我做 `replay_in_sim.py` 对比 |
| 6 | `05_sine_base` … `08_sine_wrist`（逐条执行） | 单关节小幅正弦，频率逐渐升高 | 无中止，温度 < 45 °C |
| 7 | `09_gravity_poses`（约 4 min，33 个姿态） | 慢速走一圈静止姿态 | — |
| 8 | 同 7，加 `--tag payload200g`，夹爪夹住砝码；再做一次 `payload350g` | 同上 | 用来标定力矩常数 Kt（`fit_gravity.py --payload-run`） |
| 9 | `10_fourier_A`、`11_fourier_B` | 4 个关节同时平滑摆动，各 40 s | 无中止 |
| 10 | `12_cartesian_square` | TCP 在 30 cm 高度画一个 15×24 cm 的方形 | 用尺量 4 个角的位置，或拍照 |
| 11 | `13_gripper_cycles` | 夹爪开合 3 次 | 记录 grip=0.1/0.9 时手指的实际张角（拍照） |
| 12 | `99_park` | 收到休息位附近（离限位 5°） | 然后再关电源 |

每条跑完会自动生成 `plot.png`。我会立刻跑：
`python scripts/sim/replay_in_sim.py data/real/<session>/<run>` → 输出 `results/replay_*/compare.png` 和 `metrics.json`（真机与仿真的位置和电流 RMSE）。

## E. 现场实测项（不全信网上数据）

| 量 | 怎么测 | 网上 / 模型值 |
|---|---|---|
| 肩轴离桌面的高度 | 尺量 | 0.14 m（L1） |
| HOME 位姿下前臂水平、上臂前倾角 | 手机水平仪 | 前臂 0°，上臂离竖直前倾 8.13°（β） |
| 腕法兰到指尖中心的距离 | 尺量 | 0.15 m（L5） |
| 手指闭合平面是否经过腕轴 | 俯拍和侧拍 | 模型里偏了约 15 mm，**待确认** |
| 夹爪在 grip=0.1/0.9 时的张角 | 拍照 | 仿真里用的是近似值 |
| 手指收紧方向 | 观察 | 仿真：两指向中间收 |

## F. 异常处理

- **立刻停**：你**直接关电源**，或者在对话里说"停"，我按 Ctrl+C。程序收到 Ctrl+C、SSH 断线（SIGHUP）或 `kill`（SIGTERM）时，都会**原地保持 1 s → 存日志 → 关闭连接**。
- **自动中止条件**（`src/qarm_lab/safety.py`）：瞬时电流超过额定的 90%；持续电流超过额定的 1.3 倍且超过 1.5 s；温度超过 45 °C；关节越限；跟踪误差超过 0.15 rad 且持续 0.5 s；数据冻结。底座 LED 会变红。
- **开环模式**（PWM/Current）今天**不用**，只用 Position 模式。
- 连不上时，先确认没有别的程序（QUARC 或 Quanser 的例程）占用机械臂，再重插 USB。

## G. 收尾

- 4090 上：`git add data/real && git commit && git push`（4090 的 SSH key 就是 Jasonartieth 账号，可以直接推送）。
- Mac 上：`git pull`，然后做辨识，更新 `models/qarm/sim_params/` 里的参数。
