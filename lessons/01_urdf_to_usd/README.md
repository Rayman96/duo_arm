# 01 从 URDF 到数字孪生

对应大纲：第 6 课「走进 Isaac Sim」（用成品），第 19 课「从 URDF 到数字孪生」（自己做一遍）。

## 目标

- 看懂 URDF 的结构：link、joint、惯性参数、碰撞体、关节限位。
- 知道官方 CAD 导出的 URDF 为什么不能直接用于仿真，以及怎么修正。
- 用 Isaac Sim 的 URDF 导入器生成 USD 数字孪生。

## 运行

```bash
python3 lessons/01_urdf_to_usd/fix_urdf.py alohamini2                 # 修正 URDF → assets/alohamini2_sim/
tools/run_isaac.sh lessons/01_urdf_to_usd/import_urdf.py              # 导入原版和修正版 → outputs/usd/
```

## 你会看到

- `assets/alohamini2_sim/fixes.json`：每一处修改的前后值。
- `outputs/usd/import_raw_alohamini2.json` 和 `import_sim_alohamini2.json`：两个版本导入后的关节、质量、驱动参数。
  - 都是 18 个自由度：左右臂各 6+1，升降 1，底盘 x / y / 转向 3（虚拟关节）。
  - 原版总质量 **75.5 kg**，修正后 **16.9 kg**（其中 2 kg 是底盘虚拟关节的质量）。
  - 原版关节力矩上限是**无穷大**，修正后按舵机型号设定（肩、肘 7.45 N·m，腕、夹爪 2.35 N·m）。
- 数字孪生文件：`outputs/usd/sim_alohamini2/alohamini2_sim/alohamini2_sim.usda`，后面所有课都用它。

## 讲解要点

官方 URDF 是 SolidWorks 直接导出的，有 6 处问题会让仿真出错：

1. **质量是 CAD 默认值**：5 个相机和 3 个轮子每个 10.095 kg。修正为估计值，惯量按同样比例缩放。
2. **`effort="0"`**：导入器把 0 当成「不限力矩」，机械臂会力大无穷。
3. **网格路径是 `package://`**：需要 ROS 包路径，改成相对路径。
4. **虚拟底盘关节的 link 没有质量**：导入时 x 方向自由度会丢失。
5. **虚拟 link 质量太小**（0.01 kg）：和 14 kg 的车身差 1400 倍，物理求解器会发散。
6. **轮子碰撞体贴着地面**：和地面摩擦导致底盘漂移，所以去掉轮子碰撞体。

导入器的两个坑：
- 物理参数放在 USD 的 `Physics` variant 里，默认没有选中，引用时要选 `physx`。
- 导入器不写速度上限和 PD 增益，要在代码里设置（第 02 节）。

## 课堂练习

1. 打开 `fixes.json`，找出质量被改得最多的 3 个部件，算一算改之前机器人有多重。
2. 对比两个 `import_*.json` 里 `left_shoulder_pan` 的 `maxForce`，解释为什么原版是 `inf`。
3. 进阶：把 `fix_urdf.py` 里 `VIRTUAL_MASS` 改回 0.01，重新导入，再跑第 02 节的底盘测试，观察会发生什么。

## 文件

- `fix_urdf.py`：读取 `third_party/AlohaMini/.../alohamini2.urdf`，修正后写到 `assets/`。
- `import_urdf.py`：调用 Isaac Sim URDF 导入器并统计结果。
