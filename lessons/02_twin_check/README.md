# 02 检查数字孪生：物理与相机

对应大纲：第 5 课「舵机与控制」（跟踪误差、力矩限制），第 7 课「搭建任务场景」（摆放和真机对应的相机），第 19 课（关节驱动的刚度和阻尼），第 25 课「物理差距」。

## 目标

- 用数据回答「这个数字孪生能不能用」：力矩够不够、跟得准不准、底盘和升降对不对、相机看到的对不对。
- 理解 PD 驱动（刚度、阻尼）和跟踪误差。
- 学会把相机装到机器人上，并和真机画面对比。

## 运行

```bash
tools/run_isaac.sh lessons/02_twin_check/check_physics.py        # 约 2 分钟 → outputs/twin_check/physics.json
tools/run_isaac.sh lessons/02_twin_check/check_cameras.py 0.0    # 升降在 0 位 → outputs/twin_check/images/
tools/run_isaac.sh lessons/02_twin_check/check_cameras.py 0.3    # 升到最高
```

## 你会看到

`check_physics.py`：

| 测试 | 结果 |
|---|---|
| A. 力矩裕量：100 个随机姿态下的静态保持力矩 / 舵机上限 | 最高 34%（wrist_flex） |
| B. 零位保持 3 秒 | 关节误差 ≤ 0.016 rad，升降 1.9 mm |
| C. 正弦跟踪（0.5 Hz，±0.4 rad） | RMS 误差 0.043 rad，滞后 45 ms |
| D. 升降 / 底盘速度控制 | 升降到达两端；底盘 4 秒位置误差 ≤ 2.5 mm |
| E. 打开自碰撞 | 稳定 |
| F. 仿真速度 | 单机器人约 1.1 倍实时（CPU 物理） |

`check_cameras.py`：`images/summary_lift+0.0.png` 是第三人称、头部、胸前、左腕四个视角，`geometry_lift*.json` 是每个零件的位置。

## 讲解要点

- **PD 驱动**：力矩 = 刚度 × 位置误差 − 阻尼 × 速度，并且不超过舵机上限。刚度越大跟得越紧，但太大会抖；第 25 课用真机阶跃响应来标定这两个数。
- **底盘是虚拟关节**：仿真里没有轮子滚动，而是用 x / y / 转向三个关节代替。按键时用速度控制（和真机的 `x.vel`、`y.vel`、`theta.vel` 一样），松开后保持位置。
- **相机**：光轴是相机 link 的 +z 方向（从网格上看镜头在哪一面）。USD 相机朝 −Z 看，所以要绕 x 转 180°；胸前、头部、后置相机还要再转 90°，画面才是水平的。
- **和真机对比**：`docs/images/real_grid.png` 是真机录的画面。朝向一致，但真机镜头有明显的桶形畸变，仿真是理想针孔相机，这是视觉上的 sim2real 差距（第 25–26 课）。
- **场景设计**：升降在 0 位时胸前相机高 0.54 m，和 0.55 m 的桌面差不多高，只能看到桌子侧面。

## 课堂练习

1. 把 `common/alohamini_scene.py` 里 `shoulder_pan` 等关节的刚度从 60 改成 20，重新跑 `check_physics.py`，对比 C 项的跟踪误差和滞后。
2. 看 `geometry_lift+0.3.json`：升降升到最高时，夹爪离地多高？
3. 对比 `images/lift+0.0_chest.png` 和 `docs/images/real_chest_0000.png`，列出至少三处仿真和真机的不同。

## 文件

- `check_physics.py`：A–F 六项物理测试。
- `check_cameras.py`：零件包围盒、5 路相机和第三人称渲染。
