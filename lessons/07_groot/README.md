# 07 GR00T N1.7：微调与评估

对应大纲：第 13 课「能听懂话的机器人：GR00T」，第 27 课「GR00T 模型结构」，第 28 课「为 GR00T 准备数据」，第 29 课「微调与评估」。

## 目标

- 了解视觉-语言-动作模型（VLA）：看画面、听指令、给动作。
- 在 AlohaMini2 数据上微调 GR00T N1.7-3B，并用和 ACT 完全相同的流程评估。
- 理解大模型微调的资源约束：显存、训练时间、推理延迟。

## 准备

- `nvidia/GR00T-N1.7-3B`（6.9 GB）会自动下载。
- 它的输入处理还需要 `nvidia/Cosmos-Reason2-2B` 的文件，这个仓库需要在 Hugging Face 上接受许可并登录：`.venv-lerobot/bin/hf auth login`。这台服务器已经登录。

## 运行

```bash
lessons/07_groot/train_groot.sh                   # 15k 步，约 75 分钟，显存约 20 GB → outputs/train/groot_pick_red/
lessons/07_groot/eval_groot.sh 20                 # 20 个回合 → outputs/eval/groot_15k.json
EVAL_YAW=30 lessons/07_groot/eval_groot.sh 20     # 方块随机旋转 ±30°
```

**GR00T 训练时不要同时评估**：推理还要约 13 GB 显存，两者加起来会超过 32 GB。

## 你会看到

| | 成功率（标准） | 成功率（旋转 ±30°） | 抓起用时 | 推理（平均 / 每次算动作块） |
|---|---|---|---|---|
| GR00T 15k 步 | 16/20 | 15/20 | 7.9 s | 8 ms / 127 ms |

9 次失败都是夹爪到了方块旁边却没夹住，方块只升高 0–1.6 cm，其中 7 次在工作区远端（y ≈ −0.39 m）。

## 讲解要点

- **结构**（「双系统」）：
  - 视觉语言模型（Cosmos-Reason）负责看画面、理解指令。
  - 扩散 Transformer 动作头负责生成接下来 16 步的动作。
  - `embodiment_tag=new_embodiment` 表示这是一种新机器人，状态和动作的编码层需要从头学。
- **相对动作**：机械臂关节的动作表示成「相对当前状态的变化量」。夹爪和底盘速度保持绝对值（`relative_exclude_joints`）。
- **为什么只做了部分微调**：
  - 官方配方要训练 1.6B 参数，在 32 GB 显卡上单独运行也会显存溢出。
  - 所以冻结了扩散动作头，只训练 529M 参数，batch 8（官方示例是 64）。
  - 大纲的安排正是「由教师在大显存机器上执行微调」。
- **随机性**：GR00T 的动作从随机噪声采样（flow matching），同一个场景每次结果可能不同。
- **实时性**：每 16 步算一次动作块，要 127 ms，约 4 个控制周期。真机上需要异步推理，否则会周期性地停顿。
- **这节没有体现 GR00T 的长处**：只有一个任务、一句指令。它的优势在多任务和语言泛化（见第 08 节和 `docs/act_vs_groot_report.md`）。

## 课堂练习

1. 推理时改动作块的执行步数，比如每 8 步重新规划一次：`EVAL_NAME=groot_n8 lessons/07_groot/eval_groot.sh 20 015000 groot_pick_red --n-action-steps 8`，比较成功率和耗时。
2. 第 28 课：给第 05 节加第二个任务（例如抓蓝色方块）和对应的语言指令，训练一个多任务 GR00T，看它能不能按指令选方块。
3. 有 40 GB 以上显存时：去掉 `train_groot.sh` 里的 `--policy.tune_diffusion_model=false`，把 batch 调到 32，看成功率能提高多少。

## 文件

- `train_groot.sh`：微调（`lerobot-train --policy.type=groot`）。
- `eval_groot.sh`：闭环评估。推理服务 `tools/serve_policy.py` 对 GR00T 的相对动作按整块转换成绝对动作后再执行。
