# FiSTA 肘部预测集成说明

## 集成概览

已成功将 FiSTA 肘部预测模型集成到 `robot_arm_ik.py` 中，实现左臂肘部位姿预测与IK优化。

## 主要修改

### 1. 导入与初始化
- 添加 `torch` 和 `FiSTANetwork` 导入
- `__init__` 新增参数：
  - `use_elbow_prediction`: 是否启用肘部预测（默认False）
  - `fista_checkpoint_path`: FiSTA模型checkpoint路径（可选）

### 2. 核心功能

#### 历史帧管理
- `left_ee_history`: 存储左臂末端历史（肩部坐标系）
- `left_elbow_history`: 存储左臂肘部预测历史
- 自动维护最新10帧历史

#### 肘部预测流程
1. `_update_left_ee_history()`: 更新末端历史
2. `_build_fista_history()`: 构建模型输入（前10帧用硬编码初始值填充）
3. `predict_left_elbow()`: 调用FiSTA模型预测肘部位姿
4. `_update_left_elbow_history()`: 更新肘部预测历史

### 3. CasADi误差函数
- `elbow_translational_error_shoulder`: 肘部位置误差（肩部坐标系）
- `elbow_rotational_error_shoulder`: 肘部姿态误差（肩部坐标系）

### 4. 优化目标函数
启用肘部预测时的成本函数：
```python
50 * translational_cost_shoulder +      # 末端位置（权重50）
1 * rotation_cost_shoulder +            # 末端姿态（权重1）
5 * elbow_translational_cost_shoulder + # 肘部位置（权重5）
0.5 * elbow_rotational_cost_shoulder +  # 肘部姿态（权重0.5）
0.02 * regularization_cost +            # 正则化
0.1 * smooth_cost                       # 平滑
```

### 5. solve_ik 流程（肩部输入 + 肘部预测）

```
输入: Shoulder_L_tf_target (肩部坐标系)
  ↓
预测左肘位姿: predict_left_elbow()
  ↓
构建优化参数:
  - 末端目标: 肩系 → 世界 → param_tf_l
  - 肘部目标: left_elbow_target_shoulder → param_elbow_l_target
  - 肩部坐标系: param_tf_l_shoulder
  ↓
求解优化: IPOPT
  ↓
输出: sol_q (关节角度)
```

## 使用方法

### 启用肘部预测
```python
arm_ik = G1_29_ArmIK(
    Unit_Test=True, 
    Visualization=True, 
    use_elbow_prediction=True  # 启用肘部预测
)
```

### 输入目标位姿（肩部坐标系）
```python
Shoulder_L_tf_target = pin.SE3(
    pin.Quaternion(1, 0, 0, 0),
    np.array([0.25, +0.15, -0.15]),
)

arm_ik.solve_ik(
    Shoulder_L_tf_target.homogeneous, 
    Shoulder_R_tf_target.homogeneous,
    use_shoulder_frame=True,
    input_coordinate_frame='shoulder'
)
```

## 关键Frame信息
- 左肘关节: `left_elbow_link` (Frame ID: 50)
- 左手末端: `L_ee` (Frame ID: 107, 在left_wrist_yaw_joint基础上+0.05m x方向)

## 硬编码初始值
- 肘部初始位置（肩部坐标系）: `[0.0, 0.0, -0.2]`
- 肘部初始旋转: 单位四元数 `[1, 0, 0, 0]` (wxyz)

## 模型路径
默认checkpoint: `/home/ygx/IK/GMR/checkpoints/fista_merged_1024/checkpoint_epoch_50.pth`

## 对比验证
- 左臂: 启用肘部预测
- 右臂: 不启用（仅末端IK）
- 可对比两侧运动学求解质量

## 调试输出
启用 `DEBUG` 级别日志可查看：
- 肘部预测结果: `pos=[x,y,z], quat_wxyz=[w,x,y,z]`
- IK求解模式: `Using shoulder frame IK with shoulder input and elbow prediction`

## 注意事项
1. **四元数格式转换**：FiSTA输出wxyz，Pinocchio需要xyzw，已自动转换
2. **内存连续性**：所有传递给CasADi的4×4矩阵已转为连续float64，避免Bus error
3. **历史帧填充**：前10帧用硬编码值，之后逐步使用实际预测值
4. **仅左臂生效**：当前版本肘部预测仅应用于左臂，右臂保持原IK逻辑

## 测试建议
1. 先关闭可视化测试基本功能
2. 对比启用/禁用肘部预测的IK收敛速度
3. 观察左右臂运动差异
4. 调整肘部成本权重（5和0.5）以平衡末端与肘部约束
