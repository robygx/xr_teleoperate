import casadi
import meshcat.geometry as mg
import numpy as np
import pinocchio as pin
import time
from pinocchio import casadi as cpin
from pinocchio.visualize import MeshcatVisualizer
import os
import sys
import pickle
import logging_mp
logger_mp = logging_mp.get_logger(__name__)
parent2_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(parent2_dir)

# 神经网络IK相关导入
import torch

# 添加RP_HL_IK项目路径
rp_hl_ik_path = '/home/wwb/ygx/RP_HL_IK'
if rp_hl_ik_path not in sys.path:
    sys.path.insert(0, rp_hl_ik_path)

# 延迟导入，只在需要时加载
NEURAL_IK_AVAILABLE = False
try:
    from FiSTA_Data.causal_ik_model_pieper2 import PieperCausalIK
    NEURAL_IK_AVAILABLE = True
    print("✓ PieperCausalIK 模型导入成功")
except ImportError as e:
    print(f"✗ 无法导入神经网络IK模型: {e}")
    logger_mp.warning(f"无法导入神经网络IK模型: {e}")

from teleop.utils.weighted_moving_filter import WeightedMovingFilter

class G1_29_ArmIK:
    def __init__(self, Unit_Test = False, Visualization = False):
        np.set_printoptions(precision=5, suppress=True, linewidth=200)

        self.Unit_Test = Unit_Test
        self.Visualization = Visualization

        # fixed cache file path
        self.cache_path = "g1_29_model_cache.pkl"

        if not self.Unit_Test:
            self.urdf_path = '../assets/g1/g1_body29_hand14.urdf'
            self.model_dir = '../assets/g1/'
        else:
            self.urdf_path = '../../assets/g1/g1_body29_hand14.urdf'
            self.model_dir = '../../assets/g1/'

        # Try loading cache first
        if os.path.exists(self.cache_path) and (not self.Visualization):
            logger_mp.info(f"[G1_29_ArmIK] >>> Loading cached robot model: {self.cache_path}")
            self.robot, self.reduced_robot = self.load_cache()
        else:
            logger_mp.info("[G1_29_ArmIK] >>> Loading URDF (slow)...")
            self.robot = pin.RobotWrapper.BuildFromURDF(self.urdf_path, self.model_dir)

            self.mixed_jointsToLockIDs = [
                                            "left_hip_pitch_joint" ,
                                            "left_hip_roll_joint" ,
                                            "left_hip_yaw_joint" ,
                                            "left_knee_joint" ,
                                            "left_ankle_pitch_joint" ,
                                            "left_ankle_roll_joint" ,
                                            "right_hip_pitch_joint" ,
                                            "right_hip_roll_joint" ,
                                            "right_hip_yaw_joint" ,
                                            "right_knee_joint" ,
                                            "right_ankle_pitch_joint" ,
                                            "right_ankle_roll_joint" ,
                                            "waist_yaw_joint" ,
                                            "waist_roll_joint" ,
                                            "waist_pitch_joint" ,
                                            
                                            "left_hand_thumb_0_joint" ,
                                            "left_hand_thumb_1_joint" ,
                                            "left_hand_thumb_2_joint" ,
                                            "left_hand_middle_0_joint" ,
                                            "left_hand_middle_1_joint" ,
                                            "left_hand_index_0_joint" ,
                                            "left_hand_index_1_joint" ,
                                            
                                            "right_hand_thumb_0_joint" ,
                                            "right_hand_thumb_1_joint" ,
                                            "right_hand_thumb_2_joint" ,
                                            "right_hand_index_0_joint" ,
                                            "right_hand_index_1_joint" ,
                                            "right_hand_middle_0_joint",
                                            "right_hand_middle_1_joint"
                                        ]

            self.reduced_robot = self.robot.buildReducedRobot(
                list_of_joints_to_lock=self.mixed_jointsToLockIDs,
                reference_configuration=np.array([0.0] * self.robot.model.nq),
            )

            self.reduced_robot.model.addFrame(
                pin.Frame('L_ee',
                          self.reduced_robot.model.getJointId('left_wrist_yaw_joint'),
                          pin.SE3(np.eye(3),
                                  np.array([0.05,0,0]).T),
                          pin.FrameType.OP_FRAME)
            )
            self.reduced_robot.model.addFrame(
                pin.Frame('R_ee',
                          self.reduced_robot.model.getJointId('right_wrist_yaw_joint'),
                          pin.SE3(np.eye(3),
                                  np.array([0.05,0,0]).T),
                          pin.FrameType.OP_FRAME)
            )
            # Save cache (only after everything is built)
            if not os.path.exists(self.cache_path):
                self.save_cache()
                logger_mp.info(f">>> Cache saved to {self.cache_path}")

        # for i in range(self.reduced_robot.model.nframes):
        #     frame = self.reduced_robot.model.frames[i]
        #     frame_id = self.reduced_robot.model.getFrameId(frame.name)
        #     logger_mp.debug(f"Frame ID: {frame_id}, Name: {frame.name}")

        # Creating Casadi models and data for symbolic computing
        self.cmodel = cpin.Model(self.reduced_robot.model)
        self.cdata = self.cmodel.createData()

        # Creating symbolic variables
        self.cq = casadi.SX.sym("q", self.reduced_robot.model.nq, 1) 
        self.cTf_l = casadi.SX.sym("tf_l", 4, 4)
        self.cTf_r = casadi.SX.sym("tf_r", 4, 4)
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)

        # Get the hand joint ID and define the error function
        self.L_hand_id = self.reduced_robot.model.getFrameId("L_ee")
        self.R_hand_id = self.reduced_robot.model.getFrameId("R_ee")

        self.translational_error = casadi.Function(
            "translational_error",
            [self.cq, self.cTf_l, self.cTf_r],
            [
                casadi.vertcat(
                    self.cdata.oMf[self.L_hand_id].translation - self.cTf_l[:3,3],
                    self.cdata.oMf[self.R_hand_id].translation - self.cTf_r[:3,3]
                )
            ],
        )
        self.rotational_error = casadi.Function(
            "rotational_error",
            [self.cq, self.cTf_l, self.cTf_r],
            [
                casadi.vertcat(
                    cpin.log3(self.cdata.oMf[self.L_hand_id].rotation @ self.cTf_l[:3,:3].T),
                    cpin.log3(self.cdata.oMf[self.R_hand_id].rotation @ self.cTf_r[:3,:3].T)
                )
            ],
        )

        # Defining the optimization problem
        self.opti = casadi.Opti()
        self.var_q = self.opti.variable(self.reduced_robot.model.nq)
        self.var_q_last = self.opti.parameter(self.reduced_robot.model.nq)   # for smooth
        self.param_tf_l = self.opti.parameter(4, 4)
        self.param_tf_r = self.opti.parameter(4, 4)
        self.translational_cost = casadi.sumsqr(self.translational_error(self.var_q, self.param_tf_l, self.param_tf_r))
        self.rotation_cost = casadi.sumsqr(self.rotational_error(self.var_q, self.param_tf_l, self.param_tf_r))
        self.regularization_cost = casadi.sumsqr(self.var_q)
        self.smooth_cost = casadi.sumsqr(self.var_q - self.var_q_last)

        # Setting optimization constraints and goals
        self.opti.subject_to(self.opti.bounded(
            self.reduced_robot.model.lowerPositionLimit,
            self.var_q,
            self.reduced_robot.model.upperPositionLimit)
        )
        self.opti.minimize(50 * self.translational_cost + self.rotation_cost + 0.02 * self.regularization_cost + 0.1 * self.smooth_cost)

        opts = {
            # CasADi-level options
            'expand': True, 
            'detect_simple_bounds': True,
            'calc_lam_p': False,  # https://github.com/casadi/casadi/wiki/FAQ:-Why-am-I-getting-%22NaN-detected%22in-my-optimization%3F
            'print_time':False,   # print or not
            # IPOPT solver options
            'ipopt.sb': 'yes',    # disable Ipopt's license message
            'ipopt.print_level': 0,
            'ipopt.max_iter': 30, 
            'ipopt.tol': 1e-4,
            'ipopt.acceptable_tol': 5e-4,
            'ipopt.acceptable_iter': 5,
            'ipopt.warm_start_init_point': 'yes',
            'ipopt.derivative_test': 'none',
            'ipopt.jacobian_approximation': 'exact',
            # 'ipopt.hessian_approximation': 'limited-memory',
        }
        self.opti.solver("ipopt", opts)

        self.init_data = np.zeros(self.reduced_robot.model.nq)
        self.smooth_filter = WeightedMovingFilter(np.array([0.4, 0.3, 0.2, 0.1]), 14)
        self.vis = None

        # 神经网络IK配置
        self.use_nn_ik = True  # 开关：启用/禁用神经网络IK
        self.nn_device = "cuda" if torch.cuda.is_available() else "cpu"
        self.nn_history_length = 10  # 历史帧数
        self.left_arm_history = []  # 左臂历史缓冲区

        # 初始化神经网络模型（静默模式，避免多线程print阻塞）
        if self.use_nn_ik and NEURAL_IK_AVAILABLE:
            self._init_neural_ik()
        else:
            self.neural_ik_model = None
            if self.use_nn_ik:
                logger_mp.warning("神经网络IK未启用：模型不可用")

        if self.Visualization:
            # Initialize the Meshcat visualizer for visualization
            self.vis = MeshcatVisualizer(self.reduced_robot.model, self.reduced_robot.collision_model, self.reduced_robot.visual_model)
            self.vis.initViewer(open=True) 
            self.vis.loadViewerModel("pinocchio") 
            self.vis.displayFrames(True, frame_ids=[107, 108], axis_length = 0.15, axis_width = 5)
            self.vis.display(pin.neutral(self.reduced_robot.model))

            # Enable the display of end effector target frames with short axis lengths and greater width.
            frame_viz_names = ['L_ee_target', 'R_ee_target']
            FRAME_AXIS_POSITIONS = (
                np.array([[0, 0, 0], [1, 0, 0],
                          [0, 0, 0], [0, 1, 0],
                          [0, 0, 0], [0, 0, 1]]).astype(np.float32).T
            )
            FRAME_AXIS_COLORS = (
                np.array([[1, 0, 0], [1, 0.6, 0],
                          [0, 1, 0], [0.6, 1, 0],
                          [0, 0, 1], [0, 0.6, 1]]).astype(np.float32).T
            )
            axis_length = 0.1
            axis_width = 20
            for frame_viz_name in frame_viz_names:
                self.vis.viewer[frame_viz_name].set_object(
                    mg.LineSegments(
                        mg.PointsGeometry(
                            position=axis_length * FRAME_AXIS_POSITIONS,
                            color=FRAME_AXIS_COLORS,
                        ),
                        mg.LineBasicMaterial(
                            linewidth=axis_width,
                            vertexColors=True,
                        ),
                    )
                )

    # Save both robot.model and reduced_robot.model
    def save_cache(self):
        data = {
            "robot_model": self.robot.model,
            "reduced_model": self.reduced_robot.model,
        }

        with open(self.cache_path, "wb") as f:
            pickle.dump(data, f)

    # Load both robot.model and reduced_robot.model
    def load_cache(self):
        with open(self.cache_path, "rb") as f:
            data = pickle.load(f)

        robot = pin.RobotWrapper()
        robot.model = data["robot_model"]
        robot.data = robot.model.createData()

        reduced_robot = pin.RobotWrapper()
        reduced_robot.model = data["reduced_model"]
        reduced_robot.data = reduced_robot.model.createData()

        return robot, reduced_robot

    def scale_arms(self, human_left_pose, human_right_pose, human_arm_length=0.60, robot_arm_length=0.75):
        scale_factor = robot_arm_length / human_arm_length
        robot_left_pose = human_left_pose.copy()
        robot_right_pose = human_right_pose.copy()
        robot_left_pose[:3, 3] *= scale_factor
        robot_right_pose[:3, 3] *= scale_factor
        return robot_left_pose, robot_right_pose

    def _init_neural_ik(self):
        """初始化神经网络IK模型"""
        try:
            # 模型权重路径（使用RP_HL_IK项目路径）
            checkpoint_path = '/home/wwb/ygx/RP_HL_IK/FiSTA_Data/pieper_causal_ik_092.pth'

            # 检查文件是否存在
            if not os.path.exists(checkpoint_path):
                logger_mp.warning(f"神经网络权重文件不存在: {checkpoint_path}")
                self.neural_ik_model = None
                return

            # 加载权重
            checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=False)

            # 初始化模型
            self.neural_ik_model = PieperCausalIK(
                num_joints=7,
                num_frames=self.nn_history_length,
                hidden_dim=512,
                num_layers=2
            ).to(self.nn_device)

            # 加载模型权重
            self.neural_ik_model.load_state_dict(checkpoint["model_state_dict"])
            self.neural_ik_model.eval()  # 切换到推理模式

            logger_mp.info(f"✓ 神经网络IK模型已加载 (device: {self.nn_device})")

        except Exception as e:
            logger_mp.error(f"神经网络IK初始化失败: {e}")
            self.neural_ik_model = None

    def _solve_neural_ik_left(self, left_wrist_pose, current_left_arm_q):
        """
        使用神经网络求解左臂IK

        参数:
            left_wrist_pose: 左手腕目标位姿 (4x4 numpy数组)
            current_left_arm_q: 当前左臂关节角度 (7维)

        返回:
            left_arm_joints: 预测的左臂关节角度 (7维)
        """
        if self.neural_ik_model is None:
            # 模型未加载，返回传统IK结果
            return current_left_arm_q

        try:
            # 1. 提取目标位姿
            target_pos = left_wrist_pose[:3, 3]  # [x, y, z]
            target_rot_mat = left_wrist_pose[:3, :3]
            target_quat = pin.Quaternion(target_rot_mat).coeffs()  # [qx, qy, qz, qw]

            # 2. 维护历史缓冲区
            self.left_arm_history.append(current_left_arm_q.copy())

            # 缓冲区未满时，重复第一帧填充
            while len(self.left_arm_history) < self.nn_history_length:
                self.left_arm_history.insert(0, self.left_arm_history[0])

            # 超出长度时移除最旧的帧
            if len(self.left_arm_history) > self.nn_history_length:
                self.left_arm_history.pop(0)

            # 3. 准备输入数据（修复NumPy数组不可写警告）
            history_frames = np.array(self.left_arm_history)  # [10, 7]
            history_tensor = torch.from_numpy(history_frames.copy()).float().unsqueeze(0).to(self.nn_device)  # [1, 10, 7]
            target_pos_tensor = torch.from_numpy(target_pos.copy()).float().unsqueeze(0).to(self.nn_device)  # [1, 3]
            target_quat_tensor = torch.from_numpy(target_quat.copy()).float().unsqueeze(0).to(self.nn_device)  # [1, 4]

            # 4. 神经网络推理
            with torch.no_grad():
                pred_angles, info = self.neural_ik_model(
                    history_tensor,
                    target_pos_tensor,
                    target_quat_tensor
                )
                left_arm_joints = pred_angles.cpu().numpy().flatten()  # [7]

            return left_arm_joints

        except Exception as e:
            logger_mp.error(f"神经网络IK求解失败: {e}，使用传统IK结果")
            return current_left_arm_q

    def solve_ik(self, left_wrist, right_wrist, current_lr_arm_motor_q = None, current_lr_arm_motor_dq = None):
        if current_lr_arm_motor_q is not None:
            self.init_data = current_lr_arm_motor_q
        self.opti.set_initial(self.var_q, self.init_data)

        # left_wrist, right_wrist = self.scale_arms(left_wrist, right_wrist)
        if self.Visualization:
            self.vis.viewer['L_ee_target'].set_transform(left_wrist)   # for visualization
            self.vis.viewer['R_ee_target'].set_transform(right_wrist)  # for visualization

        self.opti.set_value(self.param_tf_l, left_wrist)
        self.opti.set_value(self.param_tf_r, right_wrist)
        self.opti.set_value(self.var_q_last, self.init_data) # for smooth

        try:
            sol = self.opti.solve()
            # sol = self.opti.solve_limited()

            sol_q = self.opti.value(self.var_q)
            self.smooth_filter.add_data(sol_q)
            sol_q = self.smooth_filter.filtered_data

            if current_lr_arm_motor_dq is not None:
                v = current_lr_arm_motor_dq * 0.0
            else:
                v = (sol_q - self.init_data) * 0.0

            self.init_data = sol_q

            sol_tauff = pin.rnea(self.reduced_robot.model, self.reduced_robot.data, sol_q, v, np.zeros(self.reduced_robot.model.nv))

            # ===== 混合IK：神经网络替换左臂 =====
            if self.use_nn_ik and self.neural_ik_model is not None:
                # 使用神经网络预测左臂
                left_arm_traditional = sol_q[:7].copy()  # 保存传统IK结果
                left_arm_nn = self._solve_neural_ik_left(left_wrist, sol_q[:7])
                # 替换左臂结果
                sol_q[:7] = left_arm_nn

                # 计算差异（调试用，静默模式）
                diff = np.linalg.norm(left_arm_nn - left_arm_traditional)
                logger_mp.debug(f"[NN IK] 与传统IK差异: {diff:.4f} rad")

            if self.Visualization:
                self.vis.display(sol_q)  # for visualization

            return sol_q, sol_tauff
        
        except Exception as e:
            logger_mp.error(f"ERROR in convergence, plotting debug info.{e}")

            sol_q = self.opti.debug.value(self.var_q)
            self.smooth_filter.add_data(sol_q)
            sol_q = self.smooth_filter.filtered_data

            if current_lr_arm_motor_dq is not None:
                v = current_lr_arm_motor_dq * 0.0
            else:
                v = (sol_q - self.init_data) * 0.0

            self.init_data = sol_q

            sol_tauff = pin.rnea(self.reduced_robot.model, self.reduced_robot.data, sol_q, v, np.zeros(self.reduced_robot.model.nv))

            logger_mp.error(f"sol_q:{sol_q} \nmotorstate: \n{current_lr_arm_motor_q} \nleft_pose: \n{left_wrist} \nright_pose: \n{right_pose}")

            # ===== 混合IK：神经网络替换左臂（异常分支）=====
            if self.use_nn_ik and self.neural_ik_model is not None:
                # 尝试使用神经网络预测左臂
                left_arm_nn = self._solve_neural_ik_left(left_wrist, sol_q[:7])
                # 替换显示用的左臂结果
                sol_q[:7] = left_arm_nn

            if self.Visualization:
                self.vis.display(sol_q)  # for visualization

            # return sol_q, sol_tauff
            return current_lr_arm_motor_q, np.zeros(self.reduced_robot.model.nv)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='G1 Arm IK (Pure) - Fixed Pose or VR Input')
    parser.add_argument('--input-mode', type=str, choices=['fixed', 'vr'], default='fixed',
                        help='Input mode: "fixed" for hardcoded poses, "vr" for TeleVuer VR device')
    parser.add_argument('--img-server-ip', type=str, default='192.168.123.164',
                        help='IP address of image server for TeleVuer')
    args = parser.parse_args()

    # 初始化 IK 求解器
    arm_ik = G1_29_ArmIK(Unit_Test=True, Visualization=True)

    # 初始化 TeleVuer (如果使用 VR 模式)
    tv_wrapper = None
    if args.input_mode == 'vr':
        try:
            from televuer import TeleVuerWrapper
            tv_wrapper = TeleVuerWrapper(
                use_hand_tracking=False,  # ← 改成 False，使用手柄
                binocular=False,
                img_shape=(720, 1280),
                display_mode='pass-through',
                zmq=False,
                webrtc=False,
            )
            logger_mp.info("✓ TeleVuer initialized for VR input mode")
        except ImportError as e:
            logger_mp.error(f"Failed to import TeleVuer: {e}")
            logger_mp.error("Falling back to fixed pose mode")
            args.input_mode = 'fixed'
        except Exception as e:
            logger_mp.error(f"Failed to initialize TeleVuer: {e}")
            logger_mp.error("Falling back to fixed pose mode")
            args.input_mode = 'fixed'

    # ===== 模式配置 =====
    if args.input_mode == 'vr':
        logger_mp.info("VR Input Mode enabled - waiting for VR device connection...")
    else:
        # ===== 固定位姿模式配置 =====
        rotation_speed = 0.005
        noise_amplitude_translation = 0.001
        noise_amplitude_rotation = 0.01

        # initial positon
        L_tf_target = pin.SE3(
            pin.Quaternion(1, 0, 0, 0),
            np.array([0.25, +0.25, 0.1]),
        )

        R_tf_target = pin.SE3(
            pin.Quaternion(1, 0, 0, 0),
            np.array([0.25, -0.25, 0.1]),
        )

        logger_mp.info("=" * 70)
        logger_mp.info("📏 FIXED POSE MODE (Pure G1 IK)")
        logger_mp.info("=" * 70)
        logger_mp.info(f"Left target:  {L_tf_target.translation}")
        logger_mp.info(f"Right target: {R_tf_target.translation}")
        logger_mp.info("=" * 70)

    try:
        if args.input_mode == 'fixed':
            user_input = input("\nPress 's' to start fixed pose IK test:\n")
            if user_input.lower() != 's':
                logger_mp.info("Exiting...")
                sys.exit(0)

        step = 0
        logger_mp.info("\n" + "=" * 70)
        logger_mp.info("🚀 Starting IK loop...")
        logger_mp.info("=" * 70 + "\n")

        # 初始化当前状态跟踪（VR 模式需要）
        current_lr_arm_q = np.zeros(14)  # 左臂7 + 右臂7

        while True:
            # ===== VR 输入模式 =====
            if args.input_mode == 'vr' and tv_wrapper is not None:
                try:
                    # 从 TeleVuer 获取手部位姿
                    tele_data = tv_wrapper.get_tele_data()
                    left_wrist_pose = tele_data.left_wrist_pose
                    right_wrist_pose = tele_data.right_wrist_pose

                    # 调用 IK 求解，传入当前状态并接收返回值
                    sol_q, sol_tauff = arm_ik.solve_ik(
                        left_wrist_pose,
                        right_wrist_pose,
                        current_lr_arm_q,  # ← 传入当前关节角度
                        None               # current_lr_arm_dq 设为 None
                    )

                    # 更新当前状态
                    if sol_q is not None:
                        current_lr_arm_q = sol_q.copy()

                    step += 1
                    time.sleep(0.033)  # ~30Hz

                except Exception as e:
                    logger_mp.error(f"VR input error: {e}")
                    time.sleep(0.1)
                    continue

            # ===== 固定位姿模式 =====
            else:
                # Apply rotation noise with bias towards y and z axes
                rotation_noise_L = pin.Quaternion(
                    np.cos(np.random.normal(0, noise_amplitude_rotation) / 2),0,np.random.normal(0, noise_amplitude_rotation / 2),0).normalized()  # y bias

                rotation_noise_R = pin.Quaternion(
                    np.cos(np.random.normal(0, noise_amplitude_rotation) / 2),0,0,np.random.normal(0, noise_amplitude_rotation / 2)).normalized()  # z bias

                if step <= 120:
                    angle = rotation_speed * step
                    L_tf_target.rotation = (rotation_noise_L * pin.Quaternion(np.cos(angle / 2), 0, np.sin(angle / 2), 0)).toRotationMatrix()  # y axis
                    R_tf_target.rotation = (rotation_noise_R * pin.Quaternion(np.cos(angle / 2), 0, 0, np.sin(angle / 2))).toRotationMatrix()  # z axis
                    L_tf_target.translation += (np.array([0.001,  0.001, 0.001]) + np.random.normal(0, noise_amplitude_translation, 3))
                    R_tf_target.translation += (np.array([0.001, -0.001, 0.001]) + np.random.normal(0, noise_amplitude_translation, 3))
                else:
                    angle = rotation_speed * (240 - step)
                    L_tf_target.rotation = (rotation_noise_L * pin.Quaternion(np.cos(angle / 2), 0, np.sin(angle / 2), 0)).toRotationMatrix()  # y axis
                    R_tf_target.rotation = (rotation_noise_R * pin.Quaternion(np.cos(angle / 2), 0, 0, np.sin(angle / 2))).toRotationMatrix()  # z axis
                    L_tf_target.translation -= (np.array([0.001,  0.001, 0.001]) + np.random.normal(0, noise_amplitude_translation, 3))
                    R_tf_target.translation -= (np.array([0.001, -0.001, 0.001]) + np.random.normal(0, noise_amplitude_translation, 3))

                arm_ik.solve_ik(L_tf_target.homogeneous, R_tf_target.homogeneous)

                step += 1
                if step > 240:
                    step = 0
                time.sleep(0.1)

    except KeyboardInterrupt:
        logger_mp.info("Interrupted by user. Shutting down cleanly.")
    finally:
        # 清理 TeleVuer 连接
        if tv_wrapper is not None:
            try:
                tv_wrapper.close()
                logger_mp.info("✓ TeleVuer connection closed")
            except Exception as e:
                logger_mp.error(f"Failed to close TeleVuer: {e}")
        sys.exit(0)