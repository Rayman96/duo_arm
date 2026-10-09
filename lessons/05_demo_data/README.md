# 05 采集示范数据

对应大纲：第 10 课「采集示范数据」（一条 episode 由什么组成，好数据的标准），第 24 课「数据的规律」，第 28 课（为 GR00T 准备数据）。

## 目标

- 知道一条示范（episode）包含什么：每一帧的状态、动作、相机画面和任务描述。
- 在仿真里自动生成示范：任务是左臂抓起随机放置的红色方块。
- 转成 LeRobot 数据集格式，第 06、07 节的 ACT 和 GR00T 直接用它训练。

## 运行

```bash
tools/run_isaac.sh lessons/05_demo_data/gen_demos.py 50 0          # 50 条示范，随机种子 0，约 10 分钟 → outputs/demos/
.venv-lerobot/bin/python lessons/05_demo_data/make_dataset.py      # 约 2 分钟 → outputs/datasets/alohamini2_sim_pick_red/
```

第三个参数可以指定输出目录，例如 `gen_demos.py 10 1 outputs/demos_10`，用来做数据量对比实验。

## 你会看到

- 每条示范约 7.5 s、225 帧（30 Hz）。用到的 85 次尝试中有 35 次因为逆运动学够不到而跳过，执行了的 50 次全部成功。
- 数据格式（LeRobot v3）：
  - `observation.state`：18 维。左臂 7、右臂 7、底盘 `x.vel` / `y.vel` / `theta.vel`、升降 `lift_axis.height_mm`，命名和真机一致。
  - `observation.images.forward`：头部相机，俯视桌面；`observation.images.wrist_left`：左腕相机。都是 320×240。
  - `action`：18 维关节目标；`task`："pick up the red cube with the left arm"。

## 讲解要点

- **示范怎么来的**：随机摆放方块 → 逆运动学算出抓取姿态（第 04 节）→ 按「靠近 → 下探 → 夹紧 → 升降上抬」执行，每帧记录观测和动作。只保留方块真的被抓起来的回合。
- **工作区**：方块 x ∈ [0.07, 0.17] m，y ∈ [−0.40, −0.36] m（定义在 `common/policy_io.py`）。手指朝下时左臂最远只能到 y ≈ −0.41 m。
- **好数据**：脚本示范动作非常一致，对模仿学习有利，但不像人遥操作那样有变化。真机采数据时，起始位置要有变化，动作要流畅。
- **单位**：仿真里关节是弧度；真机的 LeRobot 数据是归一化数值（−100..100），两者换算需要每台机器人的标定文件（见 `docs/feasibility_report.md`）。

## 课堂练习

1. 打开 `outputs/demos/summary.json`，统计哪些方块位置逆运动学失败了，画在 x–y 平面上。
2. 生成 10 条和 25 条示范的数据集（用不同的输出目录和 `--root`），第 06 节分别训练 ACT，画出「数据量–成功率」曲线（第 24 课）。
3. 进阶：在 `gen_demos.py` 里给方块加上随机旋转，生成一批「更多样」的示范。

## 文件

- `gen_demos.py`：在 Isaac Sim 里生成示范（npz 格式）。
- `make_dataset.py`：npz 转 LeRobot 数据集（需要 `.venv-lerobot`）。
