# VR Teleoperation 项目技术总结文档

> **目的**：为AI助手和开发者提供清晰的项目技术文档，便于理解VR遥操作数据流和二次开发。

## 📋 目录

1. [项目概述](#项目概述)
2. [完整数据流详解](#完整数据流详解)
3. [核心模块详解](#核心模块详解)
4. [坐标系统与变换](#坐标系统与变换)
5. [关键代码位置](#关键代码位置)
6. [使用示例](#使用示例)
7. [常见问题与调试](#常见问题与调试)
8. [扩展开发指南](#扩展开发指南)

---

## 项目概述

### 核心功能

本项目是一个**基于VR/XR设备的Unitree人形机器人遥操作系统**，主要特点：

- **VR设备支持**：Apple Vision Pro、PICO 4 Ultra Enterprise、Meta Quest 3
- **追踪模式**：手部追踪（Hand Tracking）、控制器追踪（Controller Tracking）
- **控制对象**：Unitree G1/G1_29、G1_23、H1、H1_2 机器人
- **实时控制**：双臂逆运动学求解 + 灵巧手控制
- **可视化**：Meshcat 3D可视化 + WebRTC低延迟图像传输

### 系统架构图

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        VR Teleoperation System                              │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌──────────┐    ┌─────────────┐    ┌──────────┐    ┌──────────────┐     │
│  │ VR Device│ →→→ │  TeleVuer   │ →→→ │tv_wrapper│ →→→ │ robot_arm_ik │     │
│  │          │    │  (televuer) │    │          │    │    (solver)  │     │
│  │ • Vision │    │  • WebSocket│    │ • 坐标变换│    │ • CasADi     │     │
│  │ • PICO   │    │  • OpenXR   │    │ • 数据滤波│    │ • IPOPT      │     │
│  │ • Quest  │    │  • WebRTC   │    │ • 偏置调整│    │ • Pinocchio  │     │
│  └──────────┘    └─────────────┘    └──────────┘    └──────────────┘     │
│                                                      ↓                    │
│                                         ┌──────────────────────┐          │
│                                         │ Unitree G1 Robot     │          │
│                                         │ • DDS通信            │          │
│                                         │ • 双臂控制 (14 DoF)  │          │
│                                         │ • 灵巧手控制         │          │
│                                         └──────────────────────┘          │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 关键技术栈

| 类别 | 技术 | 用途 |
|------|------|------|
| **VR通信** | Vuer | WebSocket/WebRTC连接XR设备 |
| **运动学** | Pinocchio | 机器人模型加载、前向运动学 |
| **优化** | CasADi + IPOPT | IK优化求解 |
| **通信** | CycloneDDS | 实时机器人控制命令发送 |
| **可视化** | Meshcat | 3D机器人模型可视化 |

---

## 完整数据流详解

### 数据流：VR设备 → 机器人

```
┌──────────────────┐
│   1. VR设备输入    │
└────────┬─────────┘
         │
         ↓ 手部追踪或控制器追踪
┌──────────────────┐
│   2. TeleVuer     │  ← televuer.py
│  - WebSocket连接  │
│  - OpenXR协议     │
│  - 数据事件处理   │
└────────┬─────────┘
         │
         ↓ 原始位姿数据（OpenXR坐标系）
┌──────────────────┐
│   3. tv_wrapper   │  ← tv_wrapper.py
│  - 坐标系转换     │
│  - 偏置调整       │
│  - 数据滤波       │
└────────┬─────────┘
         │
         ↓ 机器人坐标系目标位姿
┌──────────────────┐
│   4. robot_arm_ik │  ← robot_arm_ik_nn.py
│  - IK优化求解     │
│  - 关节角度计算   │
│  - 力矩估计       │
└────────┬─────────┘
         │
         ↓ 关节角度（14维）
┌──────────────────┐
│   5. 机器人控制   │
│  - DDS发送       │
│  - 双臂执行       │
└──────────────────┘
```

### 数据格式转换

| 阶段 | 数据格式 | 坐标系 | 说明 |
|------|----------|--------|------|
| **VR设备** | OpenXR Pose | OpenXR | 7自由度位姿（position + quaternion） |
| **TeleVuer** | 4x4矩阵 | OpenXR | 齐次变换矩阵 |
| **tv_wrapper** | 4x4矩阵 | Robot | 坐标转换 + 偏置调整 |
| **IK求解** | 4x4矩阵 | Waist | 机器人腰部坐标系 |
| **机器人** | 14维向量 | Joint | 关节角度（左臂7 + 右臂7） |

---

## 核心模块详解

### 1. TeleVuer - VR通信模块

**文件位置**：[teleop/televuer/src/televuer/televuer.py](teleop/televuer/src/televuer/televuer.py)

#### 核心功能

```python
class TeleVuer:
    """VR设备通信管理器"""

    def __init__(self, use_hand_tracking: bool, ...):
        """
        参数：
            use_hand_tracking: True=手部追踪, False=控制器追踪
            binocular: 是否启用双目相机
            img_shape: 图像尺寸 (720, 1280)
        """
        self.vuer = Vuer(host='0.0.0.0', cert=cert_file, key=key_file)

        if self.use_hand_tracking:
            self.vuer.add_handler("HAND_MOVE")(self.on_hand_move)    # 手部追踪
        else:
            self.vuer.add_handler("CONTROLLER_MOVE")(self.on_controller_move)  # 控制器
```

#### 数据事件处理

**手部追踪模式** ([Line 267-312](teleop/televuer/src/televuer/televuer.py:267-312)):
```python
def on_hand_move(self, event):
    """
    接收25个手部关节点的位置和朝向数据
    输入格式：OpenXR坐标系，右手系
    """
    self.left_hand_joints = event['left']['joints']   # 25个关节
    self.right_hand_joints = event['right']['joints']
    # ... 存储到手部数据缓存
```

**控制器追踪模式** ([Line 228-265](teleop/televuer/src/televuer/televuer.py:228-265)):
```python
def on_controller_move(self, event):
    """
    接收控制器位姿和按钮状态
    数据包括：
        - gripper: 位置和旋转矩阵
        - buttons: 按钮状态（trigger, grip, etc.）
    """
    self.left_controller_pose = event['left']['gripper']
    self.right_controller_pose = event['right']['gripper']
    self.left_trigger = event['left']['buttons']['trigger']
```

#### 关键属性访问

| 属性 | 返回值 | 说明 |
|------|--------|------|
| `head_pose` | 4x4矩阵 | 头部位姿（世界坐标系） |
| `left_wrist_pose` | 4x4矩阵 | 左手腕/手柄位姿 |
| `right_wrist_pose` | 4x4矩阵 | 右手腕/手柄位姿 |
| `left_hand_joints` | List[25] | 左手25个关节点 |
| `right_hand_joints` | List[25] | 右手25个关节点 |

---

### 2. tv_wrapper - 数据包装与坐标变换

**文件位置**：[teleop/televuer/src/televuer/tv_wrapper.py](teleop/televuer/src/televuer/tv_wrapper.py)

#### 核心功能

这是数据流中**最关键的坐标变换模块**，负责：
1. OpenXR坐标系 → 机器人坐标系转换
2. 世界坐标系 → 相对坐标系（头部/腰部）转换
3. 偏置调整（offset compensation）

#### 坐标转换矩阵定义 ([Line 90-123](teleop/televuer/src/televuer/tv_wrapper.py:90-123))

```python
# 1. OpenXR → 机器人坐标系
T_ROBOT_OPENXR = np.array([
    [ 0,  0, -1,  0],   # X_robot = -Z_openxr
    [-1,  0,  0,  0],   # Y_robot = -X_openxr
    [ 0,  1,  0,  0],   # Z_robot = Y_openxr
    [ 0,  0,  0,  1]
])

# 2. 左臂特殊转换
T_TO_UNITREE_HUMANOID_LEFT_ARM = np.array([
    [1,  0,  0,  0],
    [0,  0, -1,  0],
    [0,  1,  0,  0],
    [0,  0,  0,  1]
])

# 3. 右臂特殊转换
T_TO_UNITREE_HUMANOID_RIGHT_ARM = np.array([
    [1,  0,  0,  0],
    [0,  0,  1,  0],
    [0, -1,  0,  0],
    [0,  0,  0,  1]
])
```

#### 关键偏置值（IMPORTANT!）

**手部追踪模式偏置** ([Line 305-308](teleop/televuer/src/televuer/tv_wrapper.py:305-308)):
```python
# 将手腕坐标从头部原点转换到腰部原点
left_IPunitree_Brobot_wrist_arm[0, 3] += 0.15  # x偏置：15cm向前
left_IPunitree_Brobot_wrist_arm[2, 3] += 0.45  # z偏置：45cm向上
```

**控制器追踪模式偏置** ([Line 406-409](teleop/televuer/src/televuer/tv_wrapper.py:406-409)):
```python
# 控制器模式使用不同的偏置值（因为控制器和手位置不同）
left_IPunitree_Brobot_wrist_arm[0, 3] += 0.05  # x偏置：5cm向前
left_IPunitree_Brobot_wrist_arm[2, 3] += 0.50  # z偏置：50cm向上
```

#### 完整转换流程 ([Line 397-409](teleop/televuer/src/televuer/tv_wrapper.py:397-409))

```python
# 步骤1: 获取原始数据（OpenXR坐标系）
left_wrist_openxr = self.televuer.left_wrist_pose

# 步骤2: 转换到机器人坐标系
left_wrist_robot = T_ROBOT_OPENXR @ left_wrist_openxr

# 步骤3: 转换到相对头部的坐标系
left_IPunitree_Brobot_head_arm = T_LEFT_ARM @ left_wrist_robot
left_IPunitree_Brobot_head_arm[0:3, 3] -= Brobot_world_head[0:3, 3]  # 减去头部位置

# 步骤4: 添加腰部偏置（世界坐标系 → 腰部坐标系）
left_IPunitree_Brobot_wrist_arm[0, 3] += 0.05  # x偏置
left_IPunitree_Brobot_wrist_arm[2, 3] += 0.50  # z偏置
```

#### 数据结构

```python
@dataclass
class TeleData:
    """遥操作数据包装类"""
    left_wrist_pose: np.ndarray      # 4x4矩阵，左手腕/手柄位姿
    right_wrist_pose: np.ndarray     # 4x4矩阵，右手腕/手柄位姿
    left_hand_joints: List           # 左手25个关节点（手部追踪模式）
    right_hand_joints: List          # 右手25个关节点
    head_pose: np.ndarray            # 4x4矩阵，头部位姿
    left_trigger_value: float        # 左扳机值（控制器模式）
    right_trigger_value: float       # 右扳机值
```

---

### 3. robot_arm_ik_nn - IK求解器

**文件位置**：[teleop/robot_control/robot_arm_ik_nn.py](teleop/robot_control/robot_arm_ik_nn.py)

#### 核心功能

使用 **CasADi + IPOPT** 进行双臂逆运动学优化求解。

#### IK优化问题定义 ([Line 142-178](teleop/robot_control/robot_arm_ik_nn.py:142-178))

```python
# 优化变量：14个关节角度（左臂7个 + 右臂7个）
self.var_q = self.opti.variable(14)

# 参数：
self.param_tf_l = self.opti.parameter(4, 4)  # 左手目标位姿
self.param_tf_r = self.opti.parameter(4, 4)  # 右手目标位姿
self.var_q_last = self.opti.parameter(14)    # 上一帧关节角度

# 目标函数：
self.opti.minimize(
    50 * translational_cost +      # 末端位置误差（权重最高）
    1 * rotation_cost +            # 末端姿态误差
    0.02 * regularization_cost +   # 正则化（防止奇异）
    0.1 * smooth_cost              # 平滑性（与上一帧的差异）
)

# 约束条件：
self.opti.subject_to(
    self.opti.bounded(
        self.reduced_robot.model.lowerPositionLimit,  # 关节下限
        self.var_q,
        self.reduced_robot.model.upperPositionLimit   # 关节上限
    )
)
```

#### 求解器配置 ([Line 160-178](teleop/robot_control/robot_arm_ik_nn.py:160-178))

```python
opts = {
    'ipopt.max_iter': 30,              # 最大迭代次数
    'ipopt.tol': 1e-4,                 # 收敛容差
    'ipopt.warm_start_init_point': 'yes',  # 热启动（使用上一帧解）
    'ipopt.print_level': 0,            # 不输出求解过程
}
self.opti.solver("ipopt", opts)
```

#### 热启动机制（CRITICAL!）

**为什么要热启动？**
- IK求解是迭代优化过程
- 使用上一帧的解作为初始猜测，可以大幅提升收敛速度
- 从零开始初始化会导致求解失败或速度慢

**实现方式** ([Line 254-256](teleop/robot_control/robot_arm_ik_nn.py:254-256)):
```python
def solve_ik(self, left_wrist, right_wrist, current_lr_arm_motor_q=None, ...):
    if current_lr_arm_motor_q is not None:
        self.init_data = current_lr_arm_motor_q  # 使用当前状态

    # 设置优化变量的初始值
    self.opti.set_initial(self.var_q, self.init_data)
```

**使用示例** ([Line 394-406](teleop/robot_control/robot_arm_ik_nn.py:394-406)):
```python
# 初始化状态
current_lr_arm_q = np.zeros(14)

# 主循环
while True:
    tele_data = tv_wrapper.get_tele_data()

    # 调用IK求解，传入当前状态
    sol_q, sol_tauff = arm_ik.solve_ik(
        tele_data.left_wrist_pose,
        tele_data.right_wrist_pose,
        current_lr_arm_q,  # ← 热启动参数（关键！）
        None
    )

    # 更新状态
    if sol_q is not None:
        current_lr_arm_q = sol_q.copy()
```

#### 平滑滤波器 ([Line 180-181](teleop/robot_control/robot_arm_ik_nn.py:180-181))

```python
from utils.weighted_moving_filter import WeightedMovingFilter

# 初始化滤波器（窗口大小4，权重递减）
self.smooth_filter = WeightedMovingFilter(
    np.array([0.4, 0.3, 0.2, 0.1]),  # 最新帧权重最大
    14  # 14个关节
)

# 在solve_ik中应用滤波
sol_q = self.opti.value(self.var_q)
self.smooth_filter.add_data(sol_q)
sol_q = self.smooth_filter.filtered_data  # 平滑后的关节角度
```

#### 机器人模型加载 ([Line 60-99](teleop/robot_control/robot_arm_ik_nn.py:60-99))

```python
# 支持模型缓存加速启动
if os.path.exists(self.cache_path):
    self.robot, self.reduced_robot = self.load_cache()  # 快速加载
else:
    # 首次加载（较慢）
    self.robot = pin.RobotWrapper.BuildFromURDF(self.urdf_path)

    # 降维：锁定腿部、腰部、手指关节
    self.reduced_robot = self.robot.buildReducedRobot(
        list_of_joints_to_lock=self.mixed_jointsToLockIDs
    )

    # 添加末端执行器frame
    self.reduced_robot.model.addFrame(
        pin.Frame('L_ee', joint_id, SE3(offset), pin.FrameType.OP_FRAME)
    )
```

**锁定的关节** ([Line 43-75](teleop/robot_control/robot_arm_ik_nn.py:43-75)):
- 左腿6个关节
- 右腿6个关节
- 腰部3个关节
- 手指14个关节

**降维后**：29 DoF → 14 DoF（仅双臂）

---

### 4. weighted_moving_filter - 加权移动平均滤波

**文件位置**：[teleop/utils/weighted_moving_filter.py](teleop/utils/weighted_moving_filter.py)

#### 核心功能

对IK求解的关节数据进行平滑处理，减少抖动。

```python
class WeightedMovingFilter:
    def __init__(self, weights, window_size):
        """
        参数：
            weights: 权重数组，例如 [0.4, 0.3, 0.2, 0.1]
                    最新数据权重最大，历史数据权重递减
            window_size: 窗口大小（必须等于weights长度）
        """
        self._weights = weights
        self._window_size = window_size
        self._data_queue = []

    def add_data(self, new_data):
        """添加新数据到队列"""
        if len(self._data_queue) >= self._window_size:
            self._data_queue.pop(0)  # 移除最旧的数据
        self._data_queue.append(new_data)

    @property
    def filtered_data(self):
        """计算加权平均"""
        # weighted_data = sum(data_i * weight_i)
        return sum([d * w for d, w in zip(self._data_queue, self._weights)])
```

#### 使用示例

```python
# 初始化滤波器（4帧窗口，权重递减）
filter = WeightedMovingFilter([0.4, 0.3, 0.2, 0.1], 14)

# 每次IK求解后滤波
raw_joint_angles = solve_ik(...)  # 原始解（可能抖动）
filter.add_data(raw_joint_angles)
smooth_joint_angles = filter.filtered_data  # 平滑解
```

---

## 坐标系统与变换

### 坐标系定义

| 坐标系 | 原点 | X轴 | Y轴 | Z轴 | 用途 |
|--------|------|-----|-----|-----|------|
| **OpenXR** | 头显 | 指向前 | 指向右 | 指向上 | VR设备默认 |
| **Robot** | 腰部 | 指向前 | 指向左 | 指向上 | 机器人运动学 |
| **Head** | 头部 | 指向前 | 指向左 | 指向上 | 相对头部控制 |
| **Waist** | 腰部 | 指向前 | 指向左 | 指向上 | IK求解原点 |

### 坐标变换公式

#### 1. OpenXR → Robot

```python
T_robot = T_ROBOT_OPENXR @ T_openxr
```

其中：
```
T_ROBOT_OPENXR = [
    [ 0,  0, -1,  0],
    [-1,  0,  0,  0],
    [ 0,  1,  0,  0],
    [ 0,  0,  0,  1]
]
```

#### 2. World → Head (相对坐标系)

```python
# 减去头部位置（平移部分）
P_head = P_world - P_head_origin
R_head = R_world  # 旋转不变
```

#### 3. Head → Waist (添加偏置)

```python
# 手部追踪模式
P_waist_x = P_head_x + 0.15  # 15cm向前
P_waist_z = P_head_z + 0.45  # 45cm向上

# 控制器追踪模式
P_waist_x = P_head_x + 0.05  # 5cm向前
P_waist_z = P_head_z + 0.50  # 50cm向上
```

### 偏置值的物理意义

**为什么需要偏置？**

1. **人体差异**：人的手臂长度和机器人不同
2. **控制方式差异**：
   - 手部追踪：追踪手腕位置
   - 控制器追踪：控制器在手掌位置
3. **原点对齐**：IK求解器以腰部为原点，需要从头部坐标转换

**调试建议**：
- 如果机器人手臂位置偏前：减小 x 偏置
- 如果机器人手臂位置偏后：增大 x 偏置
- 如果机器人手臂位置偏高：减小 z 偏置
- 如果机器人手臂位置偏低：增大 z 偏置

---

## 关键代码位置

### 主要文件清单

| 文件路径 | 行数范围 | 功能描述 |
|---------|---------|---------|
| **VR通信** |||
| [televuer.py](teleop/televuer/src/televuer/televuer.py) | 13-177 | TeleVuer类初始化 |
| [televuer.py](teleop/televuer/src/televuer/televuer.py) | 228-265 | 控制器事件处理 |
| [televuer.py](teleop/televuer/src/televuer/televuer.py) | 267-312 | 手部追踪事件处理 |
| **坐标变换** |||
| [tv_wrapper.py](teleop/televuer/src/televuer/tv_wrapper.py) | 90-123 | 坐标转换矩阵定义 |
| [tv_wrapper.py](teleop/televuer/src/televuer/tv_wrapper.py) | 305-308 | 手部追踪偏置 |
| [tv_wrapper.py](teleop/televuer/src/televuer/tv_wrapper.py) | 406-409 | 控制器偏置 |
| [tv_wrapper.py](teleop/televuer/src/televuer/tv_wrapper.py) | 242-432 | get_tele_data()主函数 |
| **IK求解** |||
| [robot_arm_ik_nn.py](teleop/robot_control/robot_arm_ik_nn.py) | 60-99 | 机器人模型加载 |
| [robot_arm_ik_nn.py](teleop/robot_control/robot_arm_ik_nn.py) | 142-178 | IK优化问题定义 |
| [robot_arm_ik_nn.py](teleop/robot_control/robot_arm_ik_nn.py) | 253-310 | solve_ik()主函数 |
| [robot_arm_ik_nn.py](teleop/robot_control/robot_arm_ik_nn.py) | 393-418 | VR模式主循环 |
| **滤波器** |||
| [weighted_moving_filter.py](teleop/utils/weighted_moving_filter.py) | 6-43 | 滤波器实现 |

### Git历史关键提交

| 提交 | 描述 | 影响 |
|------|------|------|
| afb1d0c | 集成FiSTA肘部预测模型 | 新增神经网络IK增强 |
| 4c0afdb | 版本1.5 - 模拟模式支持 | 新增--sim参数 |
| 8a74d84 | 日志优化 | 改善调试体验 |

---

## 使用示例

### 示例1：基本VR遥操作

```bash
# 启动VR模式（控制器追踪）
cd teleop/robot_control
python robot_arm_ik_nn.py --input-mode vr

# 如果使用手部追踪
# 修改代码：use_hand_tracking=True (Line 351)
```

### 示例2：测试固定轨迹

```bash
# 启动固定轨迹模式（用于测试IK求解器）
python robot_arm_ik_nn.py --input-mode fixed
```

### 示例3：测试TeleVuer数据接收

```bash
cd teleop/televuer/test
python test_tv_wrapper.py
```

### 示例4：Python脚本调用IK求解器

```python
import numpy as np
from teleop.robot_control.robot_arm_ik_nn import G1_29_ArmIK

# 初始化IK求解器
arm_ik = G1_29_ArmIK(Unit_Test=True, Visualization=True)

# 定义目标位姿（腰部坐标系）
left_target = np.eye(4)
left_target[0, 3] = 0.25  # x: 25cm
left_target[1, 3] = 0.25  # y: 25cm (左侧)
left_target[2, 3] = 0.1   # z: 10cm

right_target = np.eye(4)
right_target[0, 3] = 0.25
right_target[1, 3] = -0.25  # y: -25cm (右侧)
right_target[2, 3] = 0.1

# 初始化状态（用于热启动）
current_q = np.zeros(14)

# 求解IK
sol_q, sol_tauff = arm_ik.solve_ik(
    left_target,
    right_target,
    current_q,  # 热启动
    None
)

print(f"关节角度: {sol_q}")
print(f"关节力矩: {sol_tauff}")
```

---

## 常见问题与调试

### 问题1：VR控制器移动，但机器人手臂不动

**症状**：
- VR设备已连接
- 控制器数据正常（可通过test_tv_wrapper.py验证）
- 但Meshcat中机器人没有反应

**可能原因**：
1. **缺少热启动参数**（最常见）
   - 检查：[robot_arm_ik_nn.py:394-406](teleop/robot_control/robot_arm_ik_nn.py:394-406)
   - 确保：`solve_ik()` 的第三个参数传入了 `current_lr_arm_q`

2. **追踪模式配置错误**
   - 检查：[robot_arm_ik_nn.py:351](teleop/robot_control/robot_arm_ik_nn.py:351)
   - 确保：`use_hand_tracking=False`（控制器）或 `True`（手部追踪）

3. **偏置值不合理**
   - 检查：[tv_wrapper.py:406-409](teleop/televuer/src/televuer/tv_wrapper.py:406-409)
   - 调试：尝试增大/减小偏置值

**调试代码**：
```python
# 在VR模式主循环中添加调试日志
print(f"Left wrist: {tele_data.left_wrist_pose}")
print(f"Right wrist: {tele_data.right_wrist_pose}")
print(f"IK solution: {sol_q}")
```

### 问题2：头部移动引起机器人手臂移动

**这是正常行为！**

原因：坐标转换是基于头部相对坐标系的 ([tv_wrapper.py:397](teleop/televuer/src/televuer/tv_wrapper.py:397))

```python
# 减去头部位置（相对于头部）
left_IPunitree_Brobot_head_arm[0:3, 3] -= Brobot_world_head[0:3, 3]
```

如果需要改为绝对坐标，需要修改这部分逻辑。

### 问题3：机器人手臂抖动

**可能原因**：
1. IK求解未收敛
2. VR数据噪声
3. 缺少平滑滤波

**解决方案**：
1. 检查IK求解器输出（是否返回None）
2. 调整滤波器权重 ([robot_arm_ik_nn.py:180](teleop/robot_control/robot_arm_ik_nn.py:180))
3. 增大平滑性权重 ([robot_arm_ik_nn.py:172](teleop/robot_control/robot_arm_ik_nn.py:172))

### 问题4：IK求解失败

**错误信息**：
```
Ipopt: Maximum iterations exceeded
```

**解决方案**：
1. 检查目标位姿是否在工作空间内
2. 增加最大迭代次数 ([robot_arm_ik_nn.py:165](teleop/robot_control/robot_arm_ik_nn.py:165))
3. 放宽收敛容差 ([robot_arm_ik_nn.py:166](teleop/robot_control/robot_arm_ik_nn.py:166))

---

## 扩展开发指南

### 如何添加新的VR设备支持

1. **设备连接**：修改 [televuer.py](teleop/televuer/src/televuer/televuer.py)
2. **数据格式适配**：修改事件处理器
3. **偏置调整**：修改 [tv_wrapper.py](teleop/televuer/src/televuer/tv_wrapper.py) 的偏置值

### 如何修改IK优化目标

编辑 [robot_arm_ik_nn.py:172-175](teleop/robot_control/robot_arm_ik_nn.py:172-175)：

```python
self.opti.minimize(
    50 * translational_cost +      # 修改末端位置权重
    1 * rotation_cost +            # 修改末端姿态权重
    0.02 * regularization_cost +   # 修改正则化权重
    0.1 * smooth_cost              # 修改平滑性权重
)
```

### 如何调整滤波器参数

编辑 [robot_arm_ik_nn.py:180](teleop/robot_control/robot_arm_ik_nn.py:180)：

```python
# 增大平滑效果：增加窗口，降低最新帧权重
self.smooth_filter = WeightedMovingFilter(
    np.array([0.3, 0.25, 0.2, 0.15, 0.1]),  # 5帧窗口
    14
)

# 减小延迟：减小窗口，增大最新帧权重
self.smooth_filter = WeightedMovingFilter(
    np.array([0.7, 0.3]),  # 2帧窗口
    14
)
```

### 如何支持新机器人型号

1. 添加URDF文件到 [assets/](assets/) 目录
2. 修改 [robot_arm_ik_nn.py:43-75](teleop/robot_control/robot_arm_ik_nn.py:43-75) 的关节锁定列表
3. 更新末端执行器Frame定义 ([robot_arm_ik_nn.py:82-95](teleop/robot_control/robot_arm_ik_nn.py:82-95))

---

## 附录

### A. 依赖安装

```bash
# 基础环境
conda create -n vr_teleop python=3.10
conda activate vr_teleop

# 核心依赖
pip install pinocchio casadi numpy
pip install meshcat matplotlib

# TeleVuer
cd teleop/televuer
pip install -e .

# 机器人SDK
pip install unitree-sdk2-python
```

### B. 相关资源

- **Vuer文档**：https://vuer.ai/docs/
- **Pinocchio文档**：https://pinocchio.ai/
- **CasADi文档**：https://web.casadi.org/
- **Unitree机器人**：https://www.unitree.com/

### C. 许可证

请查看项目根目录的LICENSE文件。
