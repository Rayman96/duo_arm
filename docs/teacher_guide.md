# 教师手册

这份手册写给拿到这套材料、准备上课的老师。假设你有一台带显示器的 Linux 主机，已按根目录 `README.md` 的「安装」做完。所有命令都在这台主机的桌面终端里、仓库根目录下运行，仿真窗口直接显示在这台主机的屏幕上（接投影仪即可给全班看）。

先看「三种用法」，选一种适合你的。

## 三种用法

| 用法 | 适合 | 需要什么 | 时间 |
|---|---|---|---|
| **A. 只看结果** | 备课、了解课程能做到什么 | 什么都不用装 | 30 分钟 |
| **B. 课堂演示** | 第 1、6–8、12、14、16 课的现场展示 | 安装完成，并下载训练好的模型 | 开机 5 分钟 |
| **C. 完整复现** | 学生动手、改参数做实验 | 安装完成，GPU 空闲 | 全流程约 3 小时 |

### A. 只看结果

仓库里的文档已经带了结果和图片，不运行任何东西就可以看：

1. `docs/AlohaMini2_仿真可行性报告.html`：浏览器打开。数字孪生做了什么、验证了什么。
2. `docs/act_vs_groot_report.md`：两个模型的对比结论。
3. 图片（在 `docs/images/`）：
   - `summary_lift+0.0.png`：仿真 5 路相机画面。
   - `real_grid.png`：真机相机画面，可以和上一张对比。
   - `grasp_grid.png`：双臂抓起方块。
   - `act_40k_ep1.png`、`groot_15k_yaw30_ep0.png`：模型自主抓取的过程，每秒一帧。

### B. 课堂演示

```bash
tools/download_models.sh                                 # 第一次：下载训练好的 ACT 和 GR00T（约 13 GB）
lessons/03_sim_teleop/run_gui.sh                         # 只开仿真窗口：键盘、滑块、预设动作
lessons/03_sim_teleop/run_gui.sh stop
SIM_POLICY_AUTOPLAY=1 lessons/08_compare_demo/demo.sh    # 仿真窗口 + 两个模型自动轮流抓方块
lessons/08_compare_demo/demo.sh stop                     # 结束演示
```

窗口打开约需 1 分钟（第一次要编译着色器，会更久），GR00T 推理服务加载约需 1 分钟。

模型放在 Hugging Face 的私有仓库里，需要仓库所有者给你的账号授权。拿不到授权时，按第 06、07 节自己训练，训练完的模型放在同样的位置，演示命令不变。

### C. 完整复现

按 01 → 08 的顺序，每节目录里的 `README.md` 写了运行命令。前一节的输出是后一节的输入，都放在 `outputs/` 里。第 01 节生成的数字孪生在需要时会自动生成，所以直接从第 03 节或第 06 节开始也可以。

## 课前检查清单

开课前一天跑一遍，大约 10 分钟：

- [ ] `nvidia-smi`：显卡没有被别的任务占用（演示需要约 20 GB 空闲）。
- [ ] `docker images | grep isaac-sim`：能看到 `nvcr.io/nvidia/isaac-sim:6.1.0`。
- [ ] `tools/run_checks.sh`：生成数字孪生并跑完第 01、02、04 节的检查，约 6 分钟。每一行都应以 `[T0x]` 开头，结尾没有 `FAILED`。
- [ ] `lessons/03_sim_teleop/run_gui.sh`：屏幕上出现仿真窗口，按 W 底盘前进，再 `run_gui.sh stop` 关掉。这一步也会把着色器编译好，上课时打开就快了。
- [ ] 如果要演示模型：`ls outputs/train/*/checkpoints` 能看到 `act_pick_red` 的 040000 和 `groot_pick_red` 的 015000。
- [ ] 如果要用 GR00T：`.venv-lerobot/bin/hf auth whoami` 显示已登录。

## 每节的准备和用时

| 节 | 课堂上做什么 | 运行时间 | 显存 | 输出（都在 `outputs/` 下） |
|---|---|---|---|---|
| 01 URDF → 数字孪生 | 讲 URDF 的问题，现场导入一遍 | 1 分钟 | 4 GB | `usd/` |
| 02 检查数字孪生 | 看力矩、跟踪曲线、5 路相机 | 4 分钟 | 4 GB | `twin_check/` |
| 03 仿真遥操作 | 学生轮流用键盘和滑块操作 | 现场 | 6 GB | — |
| 04 逆运动学抓取 | 看算出的抓取姿态，双臂抓起 | 1 分钟 | 4 GB | `grasp/` |
| 05 示范数据 | 看一条示范由什么组成，生成几条 | 50 条约 12 分钟 | 4 GB | `demos/`、`datasets/` |
| 06 ACT | 一键训练，闭环验收 | 训练 40 分钟，验收 9 分钟 | 1.5 GB | `train/act_pick_red/`、`eval/` |
| 07 GR00T | 讲结构，教师集中微调，学生评估 | 训练 75 分钟，评估 15 分钟 | 训练 20 GB，推理 13 GB | `train/groot_pick_red/`、`eval/` |
| 08 对比与演示 | 看对比表，现场演示 | 演示随时；完整对比 30 分钟 | 20 GB | `eval/comparison.md` |

运行时间是在 RTX PRO 4500 32 GB 上测的，显卡不同会有差别。课时有限时，训练类的环节（06、07）建议课前跑好或直接下载模型，课堂上只启动训练看几分钟日志，然后用现成的模型评估。

## 显卡怎么排

- **可以同时跑**：仿真窗口 + ACT 训练 + ACT 推理。
- **不能同时跑**：GR00T 微调（20 GB）和 GR00T 推理（13 GB）。
- 运行第 08 节演示时（约 20 GB），不要再开 GR00T 微调。
- 显存 16 GB 的机器：仿真加 ACT 没有问题；仿真（6 GB）加 GR00T 推理（13 GB）超出 16 GB，GR00T 需要放到显存更大的教师机上。
- GR00T 微调至少需要 24 GB 显存；多组学生时由教师统一排队运行，学生只提交数据，这和大纲第 29 课的安排一致。

## 和大纲的关系

这套代码覆盖的是大纲里**能在仿真中完成**的部分。需要真机的课时（标定、主臂遥操作、真机采数据和部署），以及还没做的 Isaac Lab / Mimic / Cosmos 相关课时，见 [`outline_coverage.md`](outline_coverage.md)。

已知的不足和后续工作见 [`known_limitations.md`](known_limitations.md)。

## 常见问题

**`run_gui.sh` 提示 `no X cookie found`，或者容器日志里有 `Authorization required` / `cannot open display`**
脚本要在主机桌面上的终端里运行（不要通过 SSH 运行）。还不行就先运行 `xhost +local:`，再重新打开。

**仿真窗口一直弹「Isaac Sim 无响应」**
Ubuntu 桌面（GNOME）的检测：窗口 5 秒没反应就弹窗，点「等待」即可。可以把超时改成 60 秒：`gsettings set org.gnome.mutter check-alive-timeout 60000`（改回默认用 5000）。

**窗口打开了但是黑屏，或者半天不动**
第一次启动要编译着色器，可能需要几分钟。如果一直不动：`lessons/03_sim_teleop/run_gui.sh stop` 后重开；日志用 `docker logs -f duo-arm-sim` 查看。

**`CUDA out of memory`**
先看 `nvidia-smi` 里是谁占着显存。常见原因是上次的演示没有关：`lessons/08_compare_demo/demo.sh stop`。

**下载模型报 401 / 404**
模型仓库是私有的，需要所有者给你的 Hugging Face 账号授权，然后 `.venv-lerobot/bin/hf auth login`。

**GR00T 报 401 / 403 / gated repo**
需要在 Hugging Face 上用自己的账号打开 `nvidia/Cosmos-Reason2-2B` 页面并接受许可，然后 `.venv-lerobot/bin/hf auth login`。演示和评估 GR00T 时也需要。

**`outputs/` 里的文件删不掉，提示 Permission denied**
容器内的程序以另一个用户身份写文件，脚本结束时会改回来。如果脚本中途被强制结束，运行：
`docker run --rm -u 0 -v $PWD/outputs:/out --entrypoint chown nvcr.io/nvidia/isaac-sim:6.1.0 -R $(id -u):$(id -g) /out`

**评估结果和表里的不完全一样**
GR00T 的动作是随机采样的，同样的方块位置每次结果可能不同。ACT 是确定性的，同一台机器、同一个种子应该得到相同的结果；换显卡可能有细微差别。

**想清理磁盘**
`outputs/` 全部跑完约 20 GB，主要是 GR00T 检查点（自己训练的含优化器状态，16 GB；下载的只有 12.5 GB）和示范原始数据（1.8 GB）。训练 ACT 产生的 20k、30k 检查点只是中间结果，可以删除；10k 和 40k 在第 06 节用到。

## 文件说明

```
README.md                  总览：八节内容、硬件、安装、目录
docs/
  teacher_guide.md         本手册
  outline_coverage.md      和大纲 32 课时的对应关系
  known_limitations.md     已知不足和后续工作
  feasibility_report.md    技术报告：数字孪生（.html 是同内容的网页版）
  act_vs_groot_report.md   技术报告：两种模型的对比
  images/                  报告里用到的图片
lessons/NN_*/README.md     每节讲义：目标、命令、讲解要点、课堂练习
```
