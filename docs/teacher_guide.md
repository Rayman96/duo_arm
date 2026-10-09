# 教师手册

这份手册写给拿到这套材料、准备上课的老师。先看「三种用法」，选一种适合你的。

## 三种用法

| 用法 | 适合 | 需要什么 | 时间 |
|---|---|---|---|
| **A. 只看结果** | 备课、了解课程能做到什么 | 什么都不用装 | 30 分钟 |
| **B. 课堂演示** | 第 1、6–8、12、14、16 课的现场展示 | Isaac Sim 容器 + 训练环境（已装好） | 开机 5 分钟 |
| **C. 完整复现** | 学生动手、改参数做实验 | 同上，GPU 空闲 | 全流程约 3 小时 |

### A. 只看结果

`outputs/` 里已经有每一节跑完的结果，不运行任何东西就可以看：

1. `docs/AlohaMini2_仿真可行性报告.html`：浏览器打开。数字孪生做了什么、验证了什么。
2. `docs/act_vs_groot_report.md`：两个模型的对比结论。
3. 图片（都是 png）：
   - `outputs/twin_check/images/summary_lift+0.0.png`：仿真 5 路相机画面。
   - `outputs/grasp/images/grid.png`：双臂抓起方块。
   - `outputs/eval/act_40k_ep0.png`、`outputs/eval/groot_15k_ep0.png`：模型自主抓取的过程，每秒一帧。

### B. 课堂演示

```bash
cd ~/Documents/duo_arm
lessons/03_sim_teleop/run_gui.sh                         # 只开仿真窗口：键盘、滑块、预设动作
SIM_POLICY_AUTOPLAY=1 lessons/08_compare_demo/demo.sh    # 仿真窗口 + 两个模型自动轮流抓方块
lessons/08_compare_demo/demo.sh stop                     # 结束演示
```

窗口出现在服务器桌面上，用 NICE DCV 连接查看并投屏。窗口打开约需 1 分钟，GR00T 推理服务加载约需 1 分钟。

### C. 完整复现

按 01 → 08 的顺序，每节目录里的 `README.md` 写了运行命令。每节都可以单独重跑，因为前一节的结果已经在 `outputs/` 里。

## 课前检查清单

开课前一天跑一遍，大约 10 分钟：

- [ ] `nvidia-smi`：看显卡有没有被别的任务占用（演示需要约 20 GB 空闲）。
- [ ] `docker images | grep isaac-sim`：能看到 `nvcr.io/nvidia/isaac-sim:6.1.0`。
- [ ] `tools/run_checks.sh`：重建数字孪生并跑完第 01、02、04 节的检查，约 6 分钟。每一行都应以 `[T0x]` 开头，结尾没有 `FAILED`。
- [ ] `lessons/03_sim_teleop/run_gui.sh`：DCV 里能看到窗口，按 W 底盘前进，再 `run_gui.sh stop` 关掉。
- [ ] 如果要演示 GR00T：`.venv-lerobot/bin/hf auth whoami` 显示已登录。

## 每节的准备和用时

| 节 | 课堂上做什么 | 运行时间 | 显存 | 可以直接用的现成结果 |
|---|---|---|---|---|
| 01 URDF → 数字孪生 | 讲 URDF 的问题，现场导入一遍 | 2 分钟 | 4 GB | `outputs/usd/` |
| 02 检查数字孪生 | 看力矩、跟踪曲线、5 路相机 | 4 分钟 | 4 GB | `outputs/twin_check/` |
| 03 仿真遥操作 | 学生轮流用键盘和滑块操作 | 现场 | 6 GB | — |
| 04 逆运动学抓取 | 看算出的抓取姿态，双臂抓起 | 1 分钟 | 4 GB | `outputs/grasp/` |
| 05 示范数据 | 看一条示范由什么组成，生成几条 | 50 条约 12 分钟 | 4 GB | `outputs/demos/`、`outputs/datasets/` |
| 06 ACT | 一键训练，闭环验收 | 训练 40 分钟，验收 9 分钟 | 1.5 GB | `outputs/train/act_pick_red/` |
| 07 GR00T | 讲结构，教师集中微调，学生评估 | 训练 75 分钟，评估 15 分钟 | 训练 20 GB，推理 13 GB | `outputs/train/groot_pick_red/` |
| 08 对比与演示 | 看对比表，现场演示 | 演示随时；完整对比 30 分钟 | 20 GB | `outputs/eval/comparison.md` |

课时有限时，训练类的环节（06、07）建议课前跑好，课堂上只启动训练看几分钟日志，然后直接用现成的检查点评估。

## 显卡怎么排

服务器只有一块 32 GB 显卡：

- **可以同时跑**：仿真窗口 + ACT 训练 + ACT 推理。
- **不能同时跑**：GR00T 微调（20 GB）和 GR00T 推理（13 GB）。
- 运行第 08 节演示时（约 20 GB），不要再开 GR00T 微调。
- 多组学生共用这台服务器时，GR00T 微调由教师统一排队运行，学生只提交数据，这和大纲第 29 课的安排一致。

大纲建议学生机 16 GB 显存：仿真加 ACT 没有问题；仿真（6 GB）加 GR00T 推理（13 GB）会超出 16 GB，GR00T 推理需要放到教师机上，或者不开仿真窗口单独运行。

## 和大纲的关系

这套代码覆盖的是大纲里**能在仿真中完成**的部分。需要真机的课时（标定、主臂遥操作、真机采数据和部署），以及还没做的 Isaac Lab / Mimic / Cosmos 相关课时，见 [`outline_coverage.md`](outline_coverage.md)。

已知的不足和后续工作见 [`known_limitations.md`](known_limitations.md)。

## 常见问题

**仿真窗口一直弹「Isaac Sim 无响应」**
GNOME 的检测，点「等待」即可。这台服务器已经把超时改成 60 秒：`gsettings set org.gnome.mutter check-alive-timeout 60000`。换了服务器或用户要重新设置。

**窗口打开了但是黑屏，或者半天不动**
第一次启动要编译着色器，可能需要 2–3 分钟，之后会快很多。如果一直不动：`lessons/03_sim_teleop/run_gui.sh stop` 后重开；日志用 `docker logs -f duo-arm-sim` 查看。

**用 Isaac Sim WebRTC 客户端连不上，黑屏**
WebRTC 需要服务器放行 49100/TCP 和 47998/UDP。如果网络只开了 SSH，就用 NICE DCV 看服务器桌面。

**`CUDA out of memory`**
先看 `nvidia-smi` 里是谁占着显存。常见原因是上次的演示没有关：`lessons/08_compare_demo/demo.sh stop`。

**GR00T 报 401 / 403 / gated repo**
需要在 Hugging Face 上用自己的账号打开 `nvidia/Cosmos-Reason2-2B` 页面并接受许可，然后 `.venv-lerobot/bin/hf auth login`。

**`outputs/` 里的文件删不掉，提示 Permission denied**
容器内的程序以另一个用户身份写文件，脚本结束时会改回来。如果脚本中途被强制结束，运行：
`docker run --rm -u 0 -v $PWD/outputs:/out --entrypoint chown nvcr.io/nvidia/isaac-sim:6.1.0 -R $(id -u):$(id -g) /out`

**评估结果和表里的不完全一样**
GR00T 的动作是随机采样的，同样的方块位置每次结果可能不同。ACT 是确定性的，同一个种子应该得到完全相同的结果。

**想清理磁盘**
`outputs/` 约 20 GB，主要是 GR00T 检查点（16 GB）和示范原始数据（1.8 GB）。ACT 的 20k、30k 检查点只是中间结果，可以删除；10k 和 40k 在第 06 节用到，要保留。

## 文件说明

```
README.md                  总览：八节内容、准备、目录
docs/
  teacher_guide.md         本手册
  outline_coverage.md      和大纲 32 课时的对应关系
  known_limitations.md     已知不足和后续工作
  feasibility_report.md    技术报告：数字孪生（.html 是同内容的网页版）
  act_vs_groot_report.md   技术报告：两种模型的对比
lessons/NN_*/README.md     每节讲义：目标、命令、讲解要点、课堂练习
```
