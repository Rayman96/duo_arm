# AlohaMini2 具身智能课程：仿真部分代码

配合《机器人与具身智能课程大纲》（本目录下的 docx）使用。这里是课程里所有能在 Isaac Sim 中完成的部分：从 URDF 到数字孪生、键盘遥操作、逆运动学抓取、采集示范数据，到 ACT 训练验收、GR00T 微调评估，最后是两种模型的对比展示。

## 老师从这里开始

1. [`docs/teacher_guide.md`](docs/teacher_guide.md)：教师手册。三种用法（只看结果 / 课堂演示 / 完整复现）、课前检查清单、每节用时和显存、常见问题。
2. [`docs/outline_coverage.md`](docs/outline_coverage.md)：和大纲 32 课时的逐课对应，哪些能直接上、哪些需要真机、哪些还没做。
3. [`docs/known_limitations.md`](docs/known_limitations.md)：已知不足，以及怎么在课堂上利用它们。

最快的体验：`SIM_POLICY_AUTOPLAY=1 lessons/08_compare_demo/demo.sh`，在服务器桌面上看两个模型轮流抓方块（`demo.sh stop` 结束）。

## 八节内容

| 节 | 目录 | 内容 | 对应大纲课时 | 用时 |
|---|---|---|---|---|
| 01 | `lessons/01_urdf_to_usd` | 修正官方 URDF，导入 Isaac Sim 生成数字孪生 | 第 6、19 课 | 几分钟 |
| 02 | `lessons/02_twin_check` | 检查数字孪生：力矩、跟踪、升降、底盘、5 路相机 | 第 5、7、19、25 课 | 约 4 分钟 |
| 03 | `lessons/03_sim_teleop` | 交互仿真：键盘开底盘和升降、滑块控制关节、预设动作 | 第 6–9 课 | 现场操作 |
| 04 | `lessons/04_ik_grasp` | 逆运动学：算出抓取姿态，双臂同时抓起方块 | 第 18 课 | 约 1 分钟 |
| 05 | `lessons/05_demo_data` | 生成 50 条示范，转成 LeRobot 数据集 | 第 10 课 | 约 12 分钟 |
| 06 | `lessons/06_act` | 训练 ACT，在仿真里闭环验收 | 第 11–12、22–24 课 | 训练约 40 分钟 |
| 07 | `lessons/07_groot` | 微调 GR00T N1.7，在仿真里闭环评估 | 第 13、27–29 课 | 训练约 75 分钟 |
| 08 | `lessons/08_compare_demo` | ACT 与 GR00T 大比拼：成功率、泛化、延迟，现场演示 | 第 14、16 课 | 约 30 分钟 |

每节目录里有 `README.md`，写了这节的目标、运行命令、讲解要点和课堂练习。各节按顺序依赖：前一节的输出是后一节的输入。所有生成的东西都在 `outputs/` 里，**现在已经全部跑过一遍**，任何一节都可以直接从中间开始。

## 准备（一次性）

- **Isaac Sim 6.1**：只需要 docker 和镜像 `nvcr.io/nvidia/isaac-sim:6.1.0`。所有仿真脚本都通过 `tools/run_isaac.sh <脚本>` 在容器里运行。
- **训练环境**：`tools/setup_env.sh`。它会创建 `.venv-lerobot/`（Python 3.12、PyTorch + CUDA，LeRobot 来自 `third_party/lerobot_alohamini`），缺少的上游仓库也会克隆下来。
- **GR00T 额外需要**：在 Hugging Face 上接受 `nvidia/Cosmos-Reason2-2B` 的许可，然后运行 `.venv-lerobot/bin/hf auth login`。
- **查看界面**：用 NICE DCV 连到服务器桌面（display `:0`）。WebRTC 推流需要放行 49100/TCP 和 47998/UDP，见第 03 节。

所有命令都在仓库根目录下运行。

### 仓库里没有的东西

git 仓库只包含代码和文档。下面这些太大或可以重新生成，没有放进来：

- `third_party/`：`tools/setup_env.sh` 会自动克隆。
- `.venv-lerobot/`：`tools/setup_env.sh` 创建。
- `data/alohamini2_move_test`（真机公开数据集）：`.venv-lerobot/bin/hf download yj678/alohamini2_move_test --repo-type dataset --local-dir data/alohamini2_move_test`
- `outputs/`（约 20 GB，含训练好的模型）：按第 01–08 节依次运行即可生成；也可以从原服务器 `~/Documents/duo_arm/outputs` 直接拷贝。

## 目录

```
lessons/         八节课的脚本和讲义
common/          公共代码
  paths.py         所有路径（宿主机和容器里都适用）
  alohamini_scene.py  机器人、驱动参数、桌子方块、5 路相机、底盘速度控制
  alohamini_ik.py     抓取逆运动学
  policy_io.py        18 维特征定义（与真机命名一致）、相机、工作区范围、通信协议
tools/           公共工具
  run_isaac.sh     在 Isaac Sim 容器里运行脚本
  setup_env.sh     安装训练环境
  serve_policy.py  加载训练好的模型，通过 TCP 给仿真提供动作（ACT / GR00T 通用）
  eval_in_sim.py / run_eval.sh   仿真闭环评估
  run_checks.sh    重新生成数字孪生并跑完第 01、02、04 节的检查
assets/          修正后的 URDF（第 01 节生成）
outputs/         所有生成结果：usd/ twin_check/ grasp/ demos/ datasets/ train/ eval/
data/            真机公开数据集（对比真机画面用）
docs/            教师手册、大纲对应表、已知不足、两份技术报告
third_party/     AlohaMini、lerobot_alohamini、AM-ARM 上游仓库
```

## 显卡

服务器是一块 32 GB 的 RTX PRO 4500。占用大致如下：

- 仿真窗口：约 6 GB
- ACT 训练：1.5 GB
- GR00T 微调：约 20 GB
- GR00T 推理：约 13 GB

**GR00T 训练和 GR00T 推理不要同时运行。** 课堂上多组同时使用时，GR00T 微调建议由教师集中排队运行，这和大纲的安排一致。

## 文档

| 文件 | 内容 | 读者 |
|---|---|---|
| `docs/teacher_guide.md` | 教师手册 | 授课教师 |
| `docs/outline_coverage.md` | 与大纲 32 课时的对应 | 授课教师、课程设计 |
| `docs/known_limitations.md` | 已知不足 | 授课教师、后续开发 |
| `docs/feasibility_report.md`（及 `AlohaMini2_仿真可行性报告.html`） | 数字孪生的技术细节：官方 URDF 的问题、物理与相机验证、踩过的坑 | 技术背景 |
| `docs/act_vs_groot_report.md` | ACT 与 GR00T 的对比结果和分析 | 技术背景 |
