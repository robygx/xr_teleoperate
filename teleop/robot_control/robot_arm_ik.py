import casadi                                                                       
import meshcat.geometry as mg
import numpy as np
import pinocchio as pin                             
import time
import torch
import importlib.util
from pinocchio import casadi as cpin    
from pinocchio.visualize import MeshcatVisualizer   
import os
import sys
import pickle
import logging_mp
logger_mp = logging_mp.get_logger(__name__, level=logging_mp.DEBUG)
parent2_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(parent2_dir)

from teleop.utils.weighted_moving_filter import WeightedMovingFilter

# 添加 FiSTA 模型路径
fista_parent_dir = os.path.expanduser("~/IK/GMR")
sys.path.insert(0, fista_parent_dir)

try:
    # 尝试直接导入包，如果失败则按文件路径加载，避免 __init__ 依赖其他模块
    from hl_ik.fista_network import FiSTANetwork
    FISTA_AVAILABLE = True
except ImportError:
    try:
        fista_path = os.path.join(fista_parent_dir, "hl_ik", "fista_network.py")
        spec = importlib.util.spec_from_file_location("fista_network", fista_path)
        fista_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(fista_module)
        FiSTANetwork = fista_module.FiSTANetwork
        FISTA_AVAILABLE = True
        logger_mp.info(f"FiSTA loaded from file path: {fista_path}")
    except Exception as e:
        logger_mp.warning(f"FiSTA module not available: {e}")
        FISTA_AVAILABLE = False

class G1_29_ArmIK:
    def __init__(self, Unit_Test = False, Visualization = False, use_elbow_prediction = False, fista_checkpoint_path = None):
        np.set_printoptions(precision=5, suppress=True, linewidth=200)

        self.Unit_Test = Unit_Test
        self.Visualization = Visualization
        self.use_elbow_prediction = use_elbow_prediction and FISTA_AVAILABLE

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

        for i in range(self.reduced_robot.model.nframes):
            frame = self.reduced_robot.model.frames[i]
            frame_id = self.reduced_robot.model.getFrameId(frame.name)
            logger_mp.debug(f"Frame ID: {frame_id}, Name: {frame.name}")

        # ========== FiSTA 肘部预测模块（仅用于左臂）==========
        self.fista_model = None
        self.fista_normalization_stats = None
        self.fista_history_length = 10
        # 为稳定性强制使用 CPU，避免 GPU 环境导致潜在的硬件相关段错误
        self.fista_device = 'cpu'
        
        # 历史帧缓冲区（肩部坐标系）
        self.left_elbow_history = []  # 每项: [position(3), quaternion_wxyz(4)]
        self.left_ee_history = []     # 每项: [position(3), quaternion_wxyz(4)]
        
        # 初始肘部位姿（肩部坐标系）- 使用训练数据的均值
        # 训练数据: elbow_mean = [0.073, -0.056, -0.005]
        self.elbow_init_pos = np.array([0.073, -0.056, -0.005])
        self.elbow_init_quat_wxyz = np.array([0.727, -0.039, 0.083, -0.181])  # 训练数据四元数均值
        
        if self.use_elbow_prediction:
            if fista_checkpoint_path is None:
                fista_checkpoint_path = os.path.join(fista_parent_dir, "checkpoints/fista_merged_1024/checkpoint_epoch_50.pth")
            self._load_fista_model(fista_checkpoint_path)
            logger_mp.info(f"[G1_29_ArmIK] FiSTA elbow prediction enabled for left arm on {self.fista_device}")

        # Creating Casadi models and data for symbolic computing
        self.cmodel = cpin.Model(self.reduced_robot.model)
        self.cdata = self.cmodel.createData()

        # Creating symbolic variables
        self.cq = casadi.SX.sym("q", self.reduced_robot.model.nq, 1)
        self.cTf_l = casadi.SX.sym("tf_l", 4, 4)
        self.cTf_r = casadi.SX.sym("tf_r", 4, 4)

        # 新增: 肩部坐标系变换矩阵 (用于肩部坐标系下的IK求解)
        self.cTf_l_shoulder = casadi.SX.sym("tf_l_shoulder", 4, 4)
        self.cTf_r_shoulder = casadi.SX.sym("tf_r_shoulder", 4, 4)

        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)

        # Get the hand joint ID and define the error function
        self.L_hand_id = self.reduced_robot.model.getFrameId("L_ee")
        self.R_hand_id = self.reduced_robot.model.getFrameId("R_ee")

        # ========== 世界坐标系下的误差函数 (原始) ==========
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

        # ========== 肩部坐标系下的误差函数 (新增) ==========
        # 数学原理:
        # 1. 将末端执行器世界位姿转换到肩部坐标系
        # 2. 将目标世界位姿转换到肩部坐标系
        # 3. 在肩部坐标系下计算误差

        # 提取肩部坐标系参数
        L_shoulder_origin = self.cTf_l_shoulder[:3, 3]
        R_shoulder_origin = self.cTf_r_shoulder[:3, 3]
        L_shoulder_rot = self.cTf_l_shoulder[:3, :3]
        R_shoulder_rot = self.cTf_r_shoulder[:3, :3]

        # 末端执行器世界坐标位姿
        L_ee_world_pos = self.cdata.oMf[self.L_hand_id].translation
        R_ee_world_pos = self.cdata.oMf[self.R_hand_id].translation
        L_ee_world_rot = self.cdata.oMf[self.L_hand_id].rotation
        R_ee_world_rot = self.cdata.oMf[self.R_hand_id].rotation

        # 将末端执行器转换到肩部坐标系
        # 位置: P_shoulder = R_shoulder^T @ (P_world - P_shoulder_origin)
        L_ee_shoulder_pos = L_shoulder_rot.T @ (L_ee_world_pos - L_shoulder_origin)
        R_ee_shoulder_pos = R_shoulder_rot.T @ (R_ee_world_pos - R_shoulder_origin)

        # 姿态: R_shoulder = R_shoulder_frame^T @ R_ee_world
        L_ee_shoulder_rot = L_shoulder_rot.T @ L_ee_world_rot
        R_ee_shoulder_rot = R_shoulder_rot.T @ R_ee_world_rot

        # 目标位置也转换到肩部坐标系
        L_target_shoulder_pos = L_shoulder_rot.T @ (self.cTf_l[:3, 3] - L_shoulder_origin)
        R_target_shoulder_pos = R_shoulder_rot.T @ (self.cTf_r[:3, 3] - R_shoulder_origin)

        # 目标姿态也转换到肩部坐标系
        L_target_shoulder_rot = L_shoulder_rot.T @ self.cTf_l[:3, :3]
        R_target_shoulder_rot = R_shoulder_rot.T @ self.cTf_r[:3, :3]

        # 在肩部坐标系下计算位置误差
        self.translational_error_shoulder = casadi.Function(
            "translational_error_shoulder",
            [self.cq, self.cTf_l, self.cTf_r, self.cTf_l_shoulder, self.cTf_r_shoulder],
            [
                casadi.vertcat(
                    L_ee_shoulder_pos - L_target_shoulder_pos,
                    R_ee_shoulder_pos - R_target_shoulder_pos
                )
            ],
        )

        # 在肩部坐标系下计算姿态误差
        self.rotational_error_shoulder = casadi.Function(
            "rotational_error_shoulder",
            [self.cq, self.cTf_l, self.cTf_r, self.cTf_l_shoulder, self.cTf_r_shoulder],
            [
                casadi.vertcat(
                    cpin.log3(L_ee_shoulder_rot @ L_target_shoulder_rot.T),
                    cpin.log3(R_ee_shoulder_rot @ R_target_shoulder_rot.T)
                )
            ],
        )

        # ========== 肘部误差函数（肩部坐标系，仅左臂）==========
        # 新增符号变量：肘部目标位姿
        self.cElbow_l_target = casadi.SX.sym("elbow_l_target", 4, 4)

        # 获取左肘关节frame ID
        self.L_elbow_id = self.reduced_robot.model.getFrameId("left_elbow_link")
        logger_mp.debug(f"Left elbow frame ID: {self.L_elbow_id}")

        # 肘部世界位姿
        L_elbow_world_pos = self.cdata.oMf[self.L_elbow_id].translation
        L_elbow_world_rot = self.cdata.oMf[self.L_elbow_id].rotation

        # 将肘部转换到肩部坐标系
        L_elbow_shoulder_pos = L_shoulder_rot.T @ (L_elbow_world_pos - L_shoulder_origin)
        L_elbow_shoulder_rot = L_shoulder_rot.T @ L_elbow_world_rot

        # 肘部目标在肩部坐标系
        L_elbow_target_shoulder_pos = self.cElbow_l_target[:3, 3]
        L_elbow_target_shoulder_rot = self.cElbow_l_target[:3, :3]

        # 肘部位置误差
        self.elbow_translational_error_shoulder = casadi.Function(
            "elbow_translational_error_shoulder",
            [self.cq, self.cElbow_l_target, self.cTf_l_shoulder],
            [L_elbow_shoulder_pos - L_elbow_target_shoulder_pos],
        )

        # 肘部姿态误差
        self.elbow_rotational_error_shoulder = casadi.Function(
            "elbow_rotational_error_shoulder",
            [self.cq, self.cElbow_l_target, self.cTf_l_shoulder],
            [cpin.log3(L_elbow_shoulder_rot @ L_elbow_target_shoulder_rot.T)],
        )

        # Defining the optimization problem
        self.opti = casadi.Opti()
        self.var_q = self.opti.variable(self.reduced_robot.model.nq)
        self.var_q_last = self.opti.parameter(self.reduced_robot.model.nq)   # for smooth
        self.param_tf_l = self.opti.parameter(4, 4)
        self.param_tf_r = self.opti.parameter(4, 4)

        # 新增: 肩部坐标系参数 (用于肩部坐标系下的IK求解)
        self.param_tf_l_shoulder = self.opti.parameter(4, 4)
        self.param_tf_r_shoulder = self.opti.parameter(4, 4)

        # 肘部目标参数
        self.param_elbow_l_target = self.opti.parameter(4, 4)

        # 世界坐标系下的成本
        self.translational_cost = casadi.sumsqr(self.translational_error(self.var_q, self.param_tf_l, self.param_tf_r))
        self.rotation_cost = casadi.sumsqr(self.rotational_error(self.var_q, self.param_tf_l, self.param_tf_r))

        # 肩部坐标系下的成本 (新增)
        self.translational_cost_shoulder = casadi.sumsqr(
            self.translational_error_shoulder(self.var_q, self.param_tf_l, self.param_tf_r,
                                              self.param_tf_l_shoulder, self.param_tf_r_shoulder)
        )
        self.rotation_cost_shoulder = casadi.sumsqr(
            self.rotational_error_shoulder(self.var_q, self.param_tf_l, self.param_tf_r,
                                           self.param_tf_l_shoulder, self.param_tf_r_shoulder)
        )

        # 肘部坐标系下的成本
        self.elbow_translational_cost_shoulder = casadi.sumsqr(
            self.elbow_translational_error_shoulder(self.var_q, self.param_elbow_l_target, self.param_tf_l_shoulder)
        )
        self.elbow_rotational_cost_shoulder = casadi.sumsqr(
            self.elbow_rotational_error_shoulder(self.var_q, self.param_elbow_l_target, self.param_tf_l_shoulder)
        )

        self.regularization_cost = casadi.sumsqr(self.var_q)
        self.smooth_cost = casadi.sumsqr(self.var_q - self.var_q_last)

        # Setting optimization constraints and goals
        self.opti.subject_to(self.opti.bounded(
            self.reduced_robot.model.lowerPositionLimit,
            self.var_q,
            self.reduced_robot.model.upperPositionLimit)
        )

        # 注意: 这里使用世界坐标系下的成本作为默认
        # 在 solve_ik 中会根据 use_shoulder_frame 动态修改目标函数
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

        if self.Visualization:
            # Initialize the Meshcat visualizer for visualization
            self.vis = MeshcatVisualizer(self.reduced_robot.model, self.reduced_robot.collision_model, self.reduced_robot.visual_model)
            self.vis.initViewer(open=True) 
            self.vis.loadViewerModel("pinocchio") 
            self.vis.displayFrames(True, frame_ids=[107, 108, 44, 50, 78, 84], axis_length = 0.15, axis_width = 5)
            self.vis.display(pin.neutral(self.reduced_robot.model))

            # Enable the display of end effector target frames with short axis lengths and greater width.
            frame_viz_names = ['L_ee_target', 'R_ee_target', 'L_elbow_target']
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
            ELBOW_AXIS_COLORS = (
                np.array([[0.6, 0, 0], [0.6, 0.36, 0],
                          [0, 0.6, 0], [0.36, 0.6, 0],
                          [0, 0, 0.6], [0, 0.36, 0.6]]).astype(np.float32).T
            )
            axis_length = 0.1
            axis_width = 10
            elbow_axis_length = 0.06
            elbow_axis_width = 14
            for frame_viz_name in frame_viz_names:
                if frame_viz_name == 'L_elbow_target':
                    length = elbow_axis_length
                    width = elbow_axis_width
                    colors = ELBOW_AXIS_COLORS
                else:
                    length = axis_length
                    width = axis_width
                    colors = FRAME_AXIS_COLORS

                self.vis.viewer[frame_viz_name].set_object(
                    mg.LineSegments(
                        mg.PointsGeometry(
                            position=length * FRAME_AXIS_POSITIONS,
                            color=colors,
                        ),
                        mg.LineBasicMaterial(
                            linewidth=width,
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

    def get_torso_aligned_shoulder_frame(self, q_reduced, arm_side='left'):
        """
        计算躯干对齐的肩部坐标系 (torso-aligned shoulder frame)
        与 data_extractor.py 中的定义保持一致

        坐标系构成:
        - 位置: shoulder_pitch_joint 的世界坐标位置
        - 旋转: torso_link 的完整旋转矩阵 (从降维模型的 frame 获取)

        坐标转换公式 (世界 -> 局部):
        P_local = R_torso^T @ (P_world - P_shoulder)
        R_local = R_torso^T @ R_world

        @param q_reduced: 降维模型的关节角度 (14维,左右臂各7个)
        @param arm_side: 'left' or 'right'
        @return: 4x4 齐次变换矩阵 (numpy array), 表示肩部坐标系在世界坐标系中的位姿
        """
        # 执行降维模型的正向运动学
        pin.framesForwardKinematics(self.reduced_robot.model, self.reduced_robot.data, q_reduced)

        # 获取肩关节位置 (作为坐标系原点)
        if arm_side == 'left':
            shoulder_joint_id = self.reduced_robot.model.getJointId('left_shoulder_pitch_joint')
        else:
            shoulder_joint_id = self.reduced_robot.model.getJointId('right_shoulder_pitch_joint')

        shoulder_pose = self.reduced_robot.data.oMi[shoulder_joint_id]
        shoulder_translation = shoulder_pose.translation

        # 获取躯干旋转 (完整旋转矩阵,包含yaw/pitch/roll)
        # 使用 waist_yaw_link 的 frame 来获取躯干姿态
        # 注意: waist_yaw_joint 被锁定了,但对应的 frame 仍然存在
        try:
            # 尝试获取 waist_yaw_link 对应的 frame
            torso_frame_id = self.reduced_robot.model.getFrameId('waist_yaw_link')
            torso_pose = self.reduced_robot.data.oMf[torso_frame_id]
            torso_rotation = torso_pose.rotation
        except:
            # 如果找不到 waist_yaw_link,尝试使用 joint
            try:
                torso_joint_id = self.reduced_robot.model.getJointId('left_shoulder_pitch_joint')
                torso_pose = self.reduced_robot.data.oMi[torso_joint_id]
                # 使用肩关节的父级来获取躯干旋转
                # 简化方案:使用单位矩阵作为fallback
                logger_mp.warning("Cannot find torso frame, using identity rotation")
                torso_rotation = np.eye(3)
            except Exception as e:
                logger_mp.error(f"Error getting torso rotation: {e}")
                torso_rotation = np.eye(3)

        # 构建躯干对齐的肩部坐标系
        # 位置: 肩部世界坐标位置
        # 旋转: 躯干完整旋转矩阵
        custom_transform = np.eye(4)
        custom_transform[:3, :3] = torso_rotation
        custom_transform[:3, 3] = shoulder_translation

        return custom_transform

    def transform_target_to_shoulder_frame(self, target_world, shoulder_frame_world):
        """
        将目标位姿从世界坐标系转换到肩部相对坐标系

        数学公式:
        T_target_shoulder = T_shoulder_world^(-1) @ T_target_world

        其中:
        - T_target_world: 目标在世界坐标系的位姿 (4×4)
        - T_shoulder_world: 肩部坐标系在世界坐标系的位姿 (4×4)
        - T_target_shoulder: 目标在肩部坐标系的位姿 (4×4)

        @param target_world: 4×4 齐次变换矩阵 (目标在世界坐标系)
        @param shoulder_frame_world: 4×4 齐次变换矩阵 (肩部坐标系在世界坐标系)
        @return: 4×4 齐次变换矩阵 (目标在肩部坐标系)
        """
        # 计算 shoulder_frame_world 的逆矩阵
        shoulder_frame_inv = np.linalg.inv(shoulder_frame_world)

        # 转换目标位姿: T_shoulder = shoulder_frame_inv @ target_world
        target_shoulder = shoulder_frame_inv @ target_world

        return target_shoulder

    def transform_pose_from_shoulder_to_world(self, pose_shoulder, shoulder_frame_world):
        """
        将位姿从肩部相对坐标系转换到世界坐标系

        数学公式:
        T_world = T_shoulder_world @ T_shoulder

        @param pose_shoulder: 4×4 齐次变换矩阵 (位姿在肩部坐标系)
        @param shoulder_frame_world: 4×4 齐次变换矩阵 (肩部坐标系在世界坐标系)
        @return: 4×4 齐次变换矩阵 (位姿在世界坐标系)
        """
        pose_world = shoulder_frame_world @ pose_shoulder
        return pose_world

    def _load_fista_model(self, checkpoint_path):
        """加载 FiSTA 模型"""
        logger_mp.info(f"Loading FiSTA checkpoint from {checkpoint_path}")
        checkpoint = torch.load(checkpoint_path, map_location=self.fista_device)
        
        self.fista_model = FiSTANetwork(
            hidden_dim=checkpoint['hidden_dim'],
            spatial_hidden_dim=checkpoint['spatial_hidden_dim'],
        )
        self.fista_model.load_state_dict(checkpoint['model_state_dict'])
        self.fista_model.to(self.fista_device)
        self.fista_model.eval()
        
        self.fista_normalization_stats = checkpoint['normalization_stats']
        self.fista_history_length = checkpoint['history_length']
        logger_mp.info(f"✓ FiSTA model loaded (history_length={self.fista_history_length})")

    def _se3_to_fista_format(self, se3_pose):
        """
        将 pin.SE3 转换为 FiSTA 输入格式 [7]: [position(3), quaternion_wxyz(4)]
        """
        position = se3_pose.translation
        quat_xyzw = pin.Quaternion(se3_pose.rotation).coeffs()
        quat_wxyz = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])
        return np.concatenate([position, quat_wxyz])

    def _get_actual_left_elbow_pose_shoulder(self, q, left_shoulder_frame):
        """
        获取 IK 求解后的实际左肘位姿（肩部坐标系）
        
        Args:
            q: 关节角度
            left_shoulder_frame: 4x4 肩部坐标系变换矩阵
        
        Returns:
            elbow_vec: np.array [7] - [position(3), quaternion_wxyz(4)]
        """
        # 执行正向运动学
        pin.framesForwardKinematics(self.reduced_robot.model, self.reduced_robot.data, q)
        
        # 获取肘部世界位姿
        elbow_world_pose = self.reduced_robot.data.oMf[self.L_elbow_id]
        
        # 转换到肩部坐标系
        shoulder_inv = np.linalg.inv(left_shoulder_frame)
        elbow_world_mat = np.eye(4)
        elbow_world_mat[:3, :3] = elbow_world_pose.rotation
        elbow_world_mat[:3, 3] = elbow_world_pose.translation
        
        elbow_shoulder_mat = shoulder_inv @ elbow_world_mat
        
        # 转换为 FiSTA 格式
        position = elbow_shoulder_mat[:3, 3]
        quat_xyzw = pin.Quaternion(elbow_shoulder_mat[:3, :3]).coeffs()
        quat_wxyz = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])
        
        return np.concatenate([position, quat_wxyz])

    def _update_left_ee_history(self, ee_target_shoulder):
        """
        更新左臂末端历史帧（肩部坐标系）
        
        Args:
            ee_target_shoulder: pin.SE3 - 肩部坐标系下的末端目标位姿
        """
        ee_vec = self._se3_to_fista_format(ee_target_shoulder)
        self.left_ee_history.append(ee_vec)
        
        # 保持最新的 history_length 帧
        if len(self.left_ee_history) > self.fista_history_length:
            self.left_ee_history.pop(0)

    def _update_left_elbow_history(self, elbow_pred):
        """
        更新左臂肘部预测历史
        
        Args:
            elbow_pred: np.array [7] - 预测的肘部位姿 [position(3), quaternion_wxyz(4)]
        """
        self.left_elbow_history.append(elbow_pred.copy())
        
        if len(self.left_elbow_history) > self.fista_history_length:
            self.left_elbow_history.pop(0)

    def _build_fista_history(self):
        """
        构建 FiSTA 模型的历史状态输入 [T, 14]
        
        前 N 帧（< history_length）使用硬编码初始值填充
        """
        history_states = np.zeros((self.fista_history_length, 14))
        
        for t in range(self.fista_history_length):
            if t < len(self.left_ee_history):
                # 使用实际的末端历史
                history_states[t, 0:7] = self.left_ee_history[t]
            else:
                # 使用最早的末端历史或当前目标填充
                if len(self.left_ee_history) > 0:
                    history_states[t, 0:7] = self.left_ee_history[0]
                else:
                    # 完全没有历史时使用零填充
                    history_states[t, 0:7] = np.concatenate([np.zeros(3), [1, 0, 0, 0]])
            
            if t < len(self.left_elbow_history):
                # 使用实际的肘部预测历史
                history_states[t, 7:14] = self.left_elbow_history[t]
            else:
                # 使用硬编码初始值
                history_states[t, 7:14] = np.concatenate([self.elbow_init_pos, self.elbow_init_quat_wxyz])
        
        return history_states

    def predict_left_elbow(self, ee_target_shoulder):
        """
        使用 FiSTA 模型预测左臂肘部位姿（肩部坐标系）
        
        Args:
            ee_target_shoulder: pin.SE3 - 肩部坐标系下的末端目标位姿
        
        Returns:
            elbow_pred: np.array [7] - 预测的肘部位姿 [position(3), quaternion_wxyz(4)]
        """
        if not self.use_elbow_prediction or self.fista_model is None:
            # 返回硬编码初始值
            return np.concatenate([self.elbow_init_pos, self.elbow_init_quat_wxyz])
        
        # 更新末端历史
        self._update_left_ee_history(ee_target_shoulder)
        
        # 构建历史状态 - 确保连续内存和正确的数据类型
        history_states = np.ascontiguousarray(self._build_fista_history(), dtype=np.float64)
        
        # 当前目标 - 确保连续内存和正确的数据类型
        ee_target_vec = np.ascontiguousarray(self._se3_to_fista_format(ee_target_shoulder), dtype=np.float64)
        
        # 推理（加保护，异常或非有限值则回退初始肘部）
        try:
            with torch.no_grad():
                elbow_pred = self.fista_model.predict(
                    ee_target=ee_target_vec.copy(),  # 传入副本避免内存问题
                    history_states=history_states.copy(),  # 传入副本避免内存问题
                    device=self.fista_device,
                    normalization_stats=self.fista_normalization_stats,
                )

            elbow_pred = np.asarray(elbow_pred, dtype=np.float64).copy()  # 确保独立的内存副本

            # 检查数值有效性
            if not np.isfinite(elbow_pred).all():
                logger_mp.warning("FiSTA elbow prediction contains non-finite values, fallback to init")
                elbow_pred = np.concatenate([self.elbow_init_pos, self.elbow_init_quat_wxyz])
        except Exception as e:
            logger_mp.warning(f"FiSTA elbow prediction failed, fallback to init: {e}")
            elbow_pred = np.concatenate([self.elbow_init_pos, self.elbow_init_quat_wxyz])

        # 注意：不在这里更新肘部历史！
        # 肘部历史应该用 IK 求解后的实际肘部位姿来更新，而不是预测值
        # 这样可以避免预测误差累积
        
        return elbow_pred

    def solve_ik(self, left_wrist, right_wrist, current_lr_arm_motor_q = None, current_lr_arm_motor_dq = None,
                 use_shoulder_frame=False, input_coordinate_frame='world'):
        """
        求解逆运动学

        Args:
            left_wrist: 左手目标位姿 (4×4 齐次变换矩阵)
            right_wrist: 右手目标位姿 (4×4 齐次变换矩阵)
            current_lr_arm_motor_q: 当前关节角度 (可选)
            current_lr_arm_motor_dq: 当前关节速度 (可选)
            use_shoulder_frame: 如果为True,在肩部坐标系下进行IK求解;
                              如果为False,在世界坐标系下进行IK求解 (默认)
            input_coordinate_frame: 输入目标的坐标系
                - 'world': 世界坐标系 (默认)
                - 'shoulder': 肩部坐标系 (无需转换,直接使用)

        Returns:
            sol_q: 关节角度解
            sol_tauff: 关节力矩
        """
        if current_lr_arm_motor_q is not None:
            self.init_data = current_lr_arm_motor_q
        self.opti.set_initial(self.var_q, self.init_data)

        left_elbow_target_viz = None
        self._last_left_shoulder_frame = None  # 用于 IK 求解后更新肘部历史

        # 根据是否使用肩部坐标系,设置不同的参数和成本函数
        if use_shoulder_frame:
            # 获取当前关节角度下的肩部坐标系 (世界坐标系下的位姿)
            q_for_shoulder = self.init_data
            left_shoulder_frame = self.get_torso_aligned_shoulder_frame(q_for_shoulder, arm_side='left')
            right_shoulder_frame = self.get_torso_aligned_shoulder_frame(q_for_shoulder, arm_side='right')
            self._last_left_shoulder_frame = left_shoulder_frame  # 保存供后续更新历史使用

            # 根据输入坐标系,决定是否需要转换目标
            if input_coordinate_frame == 'world':
                # ===== 调试输出: 坐标系转换可视化 =====
                print("\n" + "="*70)
                print("坐标系转换调试信息")
                print("="*70)

                # 1. 世界坐标系目标位姿
                print("\n【世界坐标系目标位姿】")
                print(f"左手目标位置: {left_wrist[:3, 3]}")
                print(f"右手目标位置: {right_wrist[:3, 3]}")

                # 将旋转矩阵转换为四元数显示
                from scipy.spatial.transform import Rotation as R
                L_quat = R.from_matrix(left_wrist[:3, :3]).as_quat()  # xyzw
                R_quat = R.from_matrix(right_wrist[:3, :3]).as_quat()  # xyzw
                print(f"左手目标旋转(xyzw): [{L_quat[0]:.8f}, {L_quat[1]:.8f}, {L_quat[2]:.8f}, {L_quat[3]:.8f}]")
                print(f"右手目标旋转(xyzw): [{R_quat[0]:.8f}, {R_quat[1]:.8f}, {R_quat[2]:.8f}, {R_quat[3]:.8f}]")

                # 2. 肩部坐标系位姿(在世界坐标系中)
                print("\n【肩部坐标系位姿(在世界坐标系中)】")
                print(f"左肩位置: {left_shoulder_frame[:3, 3]}")
                print(f"右肩位置: {right_shoulder_frame[:3, 3]}")
                print(f"躯干旋转矩阵:\n{left_shoulder_frame[:3, :3]}")

                # 3. 转换后的肩部坐标系目标位姿
                left_target_shoulder = self.transform_target_to_shoulder_frame(left_wrist, left_shoulder_frame)
                right_target_shoulder = self.transform_target_to_shoulder_frame(right_wrist, right_shoulder_frame)

                print("\n【肩部坐标系目标位姿(转换后)】")
                print(f"左手目标位置: {left_target_shoulder[:3, 3]}")
                print(f"右手目标位置: {right_target_shoulder[:3, 3]}")

                L_quat_shoulder = R.from_matrix(left_target_shoulder[:3, :3]).as_quat()  # xyzw
                R_quat_shoulder = R.from_matrix(right_target_shoulder[:3, :3]).as_quat()  # xyzw
                print(f"左手目标旋转(xyzw): [{L_quat_shoulder[0]:.8f}, {L_quat_shoulder[1]:.8f}, {L_quat_shoulder[2]:.8f}, {L_quat_shoulder[3]:.8f}]")
                print(f"右手目标旋转(xyzw): [{R_quat_shoulder[0]:.8f}, {R_quat_shoulder[1]:.8f}, {R_quat_shoulder[2]:.8f}, {R_quat_shoulder[3]:.8f}]")

                print("="*70 + "\n")
                # ===== 调试输出结束 =====

                # ===== 肘部预测（世界坐标系输入，需先转换到肩部坐标系）=====
                if self.use_elbow_prediction:
                    # 将世界坐标系的左臂目标转换到肩部坐标系
                    left_wrist_shoulder_mat = self.transform_target_to_shoulder_frame(left_wrist, left_shoulder_frame)
                    left_wrist_shoulder_se3 = pin.SE3(left_wrist_shoulder_mat[:3, :3], left_wrist_shoulder_mat[:3, 3])
                    
                    # 预测左肘位姿（肩部坐标系）
                    elbow_pred = self.predict_left_elbow(left_wrist_shoulder_se3)
                    
                    # 构建肘部目标齐次矩阵（肩部坐标系）
                    elbow_quat_wxyz = np.asarray(elbow_pred[3:7], dtype=np.float64)
                    elbow_rot = pin.Quaternion(
                        float(elbow_quat_wxyz[0]),
                        float(elbow_quat_wxyz[1]),
                        float(elbow_quat_wxyz[2]),
                        float(elbow_quat_wxyz[3]),
                    ).toRotationMatrix()
                    
                    left_elbow_target_shoulder = np.eye(4, dtype=np.float64)
                    left_elbow_target_shoulder[:3, :3] = elbow_rot
                    left_elbow_target_shoulder[:3, 3] = np.asarray(elbow_pred[:3], dtype=np.float64)
                    
                    logger_mp.debug(f"Predicted left elbow (shoulder): pos={elbow_pred[:3]}, quat_wxyz={elbow_pred[3:7]}")

                    # 可视化用：转换到世界坐标系
                    left_elbow_target_viz = self.transform_pose_from_shoulder_to_world(
                        left_elbow_target_shoulder,
                        left_shoulder_frame,
                    )

                # 输入是世界坐标系,直接使用
                left_wrist_for_opt = np.ascontiguousarray(left_wrist, dtype=np.float64)
                right_wrist_for_opt = np.ascontiguousarray(right_wrist, dtype=np.float64)

                # 设置肩部坐标系参数
                self.opti.set_value(self.param_tf_l_shoulder, np.ascontiguousarray(left_shoulder_frame, dtype=np.float64))
                self.opti.set_value(self.param_tf_r_shoulder, np.ascontiguousarray(right_shoulder_frame, dtype=np.float64))

                # 设置肘部目标参数并修改目标函数
                if self.use_elbow_prediction:
                    self.opti.set_value(self.param_elbow_l_target, np.ascontiguousarray(left_elbow_target_shoulder, dtype=np.float64))
                    
                    # 修改目标函数，包含肘部成本（权重较小）
                    self.opti.minimize(
                        50 * self.translational_cost_shoulder + self.rotation_cost_shoulder +
                        5 * self.elbow_translational_cost_shoulder + 0.5 * self.elbow_rotational_cost_shoulder +
                        0.02 * self.regularization_cost + 0.1 * self.smooth_cost
                    )
                    
                    logger_mp.debug(f"Using shoulder frame IK with world input and elbow prediction")
                else:
                    # 修改目标函数为肩部坐标系下的成本（不含肘部）
                    self.opti.minimize(50 * self.translational_cost_shoulder + self.rotation_cost_shoulder +
                                      0.02 * self.regularization_cost + 0.1 * self.smooth_cost)
                    
                    logger_mp.debug(f"Using shoulder frame IK with world input")

                logger_mp.debug(f"Left target (world): {left_wrist[:3, 3]}")

            elif input_coordinate_frame == 'shoulder':
                # 输入已经是肩部坐标系,直接使用
                # 注意: 误差函数在肩系中比较时,会将 world 目标/末端用肩框做变换。
                # 因此这里需要将肩系输入的目标位姿先转换回世界坐标以保持一致。
                
                # ===== 肘部预测（仅左臂）=====
                if self.use_elbow_prediction:
                    # 将左臂目标转换为 pin.SE3（肩部坐标系）
                    left_wrist_shoulder_se3 = pin.SE3(left_wrist[:3, :3], left_wrist[:3, 3])
                    
                    # 预测左肘位姿（肩部坐标系）
                    elbow_pred = self.predict_left_elbow(left_wrist_shoulder_se3)
                    
                    # 构建肘部目标齐次矩阵（肩部坐标系）
                    elbow_quat_wxyz = np.asarray(elbow_pred[3:7], dtype=np.float64)
                    elbow_rot = pin.Quaternion(
                        float(elbow_quat_wxyz[0]),
                        float(elbow_quat_wxyz[1]),
                        float(elbow_quat_wxyz[2]),
                        float(elbow_quat_wxyz[3]),
                    ).toRotationMatrix()
                    
                    left_elbow_target_shoulder = np.eye(4, dtype=np.float64)
                    left_elbow_target_shoulder[:3, :3] = elbow_rot
                    left_elbow_target_shoulder[:3, 3] = np.asarray(elbow_pred[:3], dtype=np.float64)
                    
                    logger_mp.debug(f"Predicted left elbow (shoulder): pos={elbow_pred[:3]}, quat_wxyz={elbow_pred[3:7]}")

                    # 可视化用：转换到世界坐标系，单独绘制肘部目标
                    left_elbow_target_viz = self.transform_pose_from_shoulder_to_world(
                        left_elbow_target_shoulder,
                        left_shoulder_frame,
                    )
                
                # 将肩系目标转换到世界坐标
                left_wrist_world = self.transform_pose_from_shoulder_to_world(left_wrist, left_shoulder_frame)
                right_wrist_world = self.transform_pose_from_shoulder_to_world(right_wrist, right_shoulder_frame)

                # 将世界坐标目标写入优化参数（确保为连续的 float64）
                left_wrist_for_opt = np.ascontiguousarray(left_wrist_world, dtype=np.float64)
                right_wrist_for_opt = np.ascontiguousarray(right_wrist_world, dtype=np.float64)

                # 设置真实的肩部坐标系参数 (用于将 world 目标/末端转换到肩系下计算误差)
                self.opti.set_value(self.param_tf_l_shoulder, np.ascontiguousarray(left_shoulder_frame, dtype=np.float64))
                self.opti.set_value(self.param_tf_r_shoulder, np.ascontiguousarray(right_shoulder_frame, dtype=np.float64))

                # 设置肘部目标参数并修改目标函数
                if self.use_elbow_prediction:
                    self.opti.set_value(self.param_elbow_l_target, np.ascontiguousarray(left_elbow_target_shoulder, dtype=np.float64))
                    
                    # 修改目标函数，包含肘部成本（权重较小）
                    self.opti.minimize(
                        50 * self.translational_cost_shoulder + self.rotation_cost_shoulder +
                        5 * self.elbow_translational_cost_shoulder + 0.5 * self.elbow_rotational_cost_shoulder +
                        0.02 * self.regularization_cost + 0.1 * self.smooth_cost
                    )
                    
                    logger_mp.debug(f"Using shoulder frame IK with shoulder input and elbow prediction")
                else:
                    # 修改目标函数为肩部坐标系下的成本（不含肘部）
                    self.opti.minimize(50 * self.translational_cost_shoulder + self.rotation_cost_shoulder +
                                       0.02 * self.regularization_cost + 0.1 * self.smooth_cost)
                    
                    logger_mp.debug(f"Using shoulder frame IK with shoulder input (converted to world for opt)")
            else:
                raise ValueError(f"Unknown input_coordinate_frame: {input_coordinate_frame}. Must be 'world' or 'shoulder'")

            # 用于可视化的目标位姿 (如果输入是世界坐标,显示原始目标;如果是肩部坐标,转换后显示)
            if input_coordinate_frame == 'world':
                left_wrist_viz = left_wrist
                right_wrist_viz = right_wrist
            else:
                # 将肩部坐标系目标转换到世界坐标系用于可视化
                left_wrist_viz = self.transform_pose_from_shoulder_to_world(left_wrist, left_shoulder_frame)
                right_wrist_viz = self.transform_pose_from_shoulder_to_world(right_wrist, right_shoulder_frame)
        else:
            # 使用世界坐标系下的IK求解 (原始方式)
            left_wrist_for_opt = left_wrist
            right_wrist_for_opt = right_wrist
            left_wrist_viz = left_wrist
            right_wrist_viz = right_wrist

            # 恢复世界坐标系下的目标函数
            self.opti.minimize(50 * self.translational_cost + self.rotation_cost +
                              0.02 * self.regularization_cost + 0.1 * self.smooth_cost)

            logger_mp.debug(f"Using world frame IK")

        # left_wrist, right_wrist = self.scale_arms(left_wrist, right_wrist)
        if self.Visualization:
            self.vis.viewer['L_ee_target'].set_transform(left_wrist_viz)   # for visualization
            self.vis.viewer['R_ee_target'].set_transform(right_wrist_viz)  # for visualization
            if left_elbow_target_viz is not None:
                self.vis.viewer['L_elbow_target'].set_transform(left_elbow_target_viz)

        # 设置目标位姿参数
        self.opti.set_value(self.param_tf_l, left_wrist_for_opt)
        self.opti.set_value(self.param_tf_r, right_wrist_for_opt)
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

            # 用实际 IK 解的肘部位姿更新历史（避免预测误差累积）
            if self.use_elbow_prediction and self._last_left_shoulder_frame is not None:
                actual_elbow = self._get_actual_left_elbow_pose_shoulder(sol_q, self._last_left_shoulder_frame)
                self._update_left_elbow_history(actual_elbow)

            if self.Visualization:
                self.vis.display(sol_q)  # for visualization

            return sol_q, sol_tauff

        except KeyboardInterrupt:
            logger_mp.info("User interrupted optimization. Exiting solve_ik.")
            raise
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

            logger_mp.error(f"sol_q:{sol_q} \nmotorstate: \n{current_lr_arm_motor_q} \nleft_pose: \n{left_wrist} \nright_pose: \n{right_wrist}")
            if self.Visualization:
                self.vis.display(sol_q)  # for visualization

            # return sol_q, sol_tauff
            return current_lr_arm_motor_q, np.zeros(self.reduced_robot.model.nv)     
        
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='G1 Arm IK Test - Fixed Pose or VR Input')
    parser.add_argument('--input-mode', type=str, choices=['fixed', 'vr'], default='fixed',
                        help='Input mode: "fixed" for hardcoded poses, "vr" for TeleVuer VR device')
    parser.add_argument('--use-elbow-pred', action='store_true', help='Enable FiSTA elbow prediction')
    parser.add_argument('--img-server-ip', type=str, default='192.168.123.164',
                        help='IP address of image server for TeleVuer')
    args = parser.parse_args()

    # 初始化 IK 求解器
    arm_ik = G1_29_ArmIK(Unit_Test=True, Visualization=True, use_elbow_prediction=args.use_elbow_pred)

    # 初始化 TeleVuer (如果使用 VR 模式)
    tv_wrapper = None
    if args.input_mode == 'vr':
        try:
            from televuer import TeleVuerWrapper
            tv_wrapper = TeleVuerWrapper(
                use_hand_tracking=True,
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

    # ===== 选择坐标系模式 =====
    USE_SHOULDER_FRAME = True  # True: 在肩部坐标系下进行IK求解; False: 在世界坐标系下进行IK求解
    USE_SHOULDER_INPUT = False  # True: 使用肩部坐标系输入; False: 使用世界坐标系输入

    # ===== 定义目标位姿 (两种坐标系) =====

    # 方式1: 世界坐标系目标位姿
    L_tf_target = pin.SE3(
        pin.Quaternion(1, 0, 0, 0),
        np.array([0.25, +0.25, 0.1]),
    )

    R_tf_target = pin.SE3(
        pin.Quaternion(1, 0, 0, 0),
        np.array([0.25, -0.25, 0.1]),
    )

    # 方式2: 肩部坐标系目标位姿 (相对于肩关节)
    Shoulder_L_tf_target = pin.SE3(
        pin.Quaternion(1, 0, 0, 0),
        np.array([0.25, +0.15, -0.15]),
    )

    Shoulder_R_tf_target = pin.SE3(
        pin.Quaternion(1, 0, 0, 0),
        np.array([0.25, -0.15, -0.15]),
    )

    # ===== VR 模式配置 =====
    if args.input_mode == 'vr':
        logger_mp.info("=" * 70)
        logger_mp.info("🥽 VR INPUT MODE ENABLED")
        logger_mp.info("=" * 70)
        logger_mp.info("TeleVuer will provide real-time hand poses from VR device")
        logger_mp.info(f"Elbow prediction: {'ENABLED' if args.use_elbow_pred else 'DISABLED'}")
        logger_mp.info(f"IK coordinate frame: {'Shoulder' if USE_SHOULDER_FRAME else 'World'}")
        logger_mp.info("=" * 70)
    else:
        # ===== 固定位姿模式配置 =====
        rotation_speed = 0.02
        noise_amplitude_translation = 0.005
        noise_amplitude_rotation = 0.05

        logger_mp.info("=" * 70)
        logger_mp.info("📏 FIXED POSE MODE")
        logger_mp.info("=" * 70)
        logger_mp.info(f"IK求解模式: {'肩部坐标系' if USE_SHOULDER_FRAME else '世界坐标系'}")
        if USE_SHOULDER_INPUT:
            logger_mp.info(f"左目标 (肩部坐标): 位置={Shoulder_L_tf_target.translation}")
            logger_mp.info(f"右目标 (肩部坐标): 位置={Shoulder_R_tf_target.translation}")
        else:
            logger_mp.info(f"左目标 (世界坐标): 位置={L_tf_target.translation}")
            logger_mp.info(f"右目标 (世界坐标): 位置={R_tf_target.translation}")
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

        while True:
            # ===== VR 输入模式 =====
            if args.input_mode == 'vr' and tv_wrapper is not None:
                try:
                    # 从 TeleVuer 获取手部位姿
                    tele_data = tv_wrapper.get_tele_data()
                    left_wrist_pose = tele_data.left_wrist_pose
                    right_wrist_pose = tele_data.right_wrist_pose

                    if step % 30 == 0:  # 每30帧打印一次状态
                        logger_mp.info(f"[VR Mode] Step {step}: "
                                     f"L_pos={left_wrist_pose[:3, 3]}, "
                                     f"R_pos={right_wrist_pose[:3, 3]}")

                    # 调用 IK 求解（VR输入通常是世界坐标系）
                    arm_ik.solve_ik(
                        left_wrist_pose,
                        right_wrist_pose,
                        use_shoulder_frame=USE_SHOULDER_FRAME,
                        input_coordinate_frame='world'
                    )

                    step += 1
                    time.sleep(0.033)  # ~30Hz

                except Exception as e:
                    logger_mp.error(f"VR input error: {e}")
                    time.sleep(0.1)
                    continue

            # ===== 固定位姿模式 =====
            else:
                logger_mp.info(f"IK loop step={step}")
                # Apply rotation noise with bias towards y and z axes
                rotation_noise_L = pin.Quaternion(
                    np.cos(np.random.normal(0, noise_amplitude_rotation) / 2),0,np.random.normal(0, noise_amplitude_rotation / 2),0).normalized()  # y bias

                rotation_noise_R = pin.Quaternion(
                    np.cos(np.random.normal(0, noise_amplitude_rotation) / 2),0,0,np.random.normal(0, noise_amplitude_rotation / 2)).normalized()  # z bias

                if USE_SHOULDER_INPUT:
                    # 使用肩部坐标系目标
                    if step <= 120:
                        angle = rotation_speed * step
                        # 左右对称的旋转噪声
                        Shoulder_L_tf_target.rotation = (rotation_noise_L * pin.Quaternion(np.cos(angle / 2), 0, np.sin(angle / 2), 0)).toRotationMatrix()  # y axis
                        Shoulder_R_tf_target.rotation = (rotation_noise_R * pin.Quaternion(np.cos(angle / 2), 0, -np.sin(angle / 2), 0)).toRotationMatrix()  # -y axis (镜像)
                        # 左右对称的位移噪声
                        translation_noise = np.random.normal(0, noise_amplitude_translation, 3)
                        Shoulder_L_tf_target.translation += (np.array([0.003,  0.002, 0.002]) + translation_noise)
                        Shoulder_R_tf_target.translation += (np.array([0.003, -0.002, 0.002]) + translation_noise * np.array([1, -1, 1]))  # y 方向镜像
                    else:
                        angle = rotation_speed * (240 - step)
                        Shoulder_L_tf_target.rotation = (rotation_noise_L * pin.Quaternion(np.cos(angle / 2), 0, np.sin(angle / 2), 0)).toRotationMatrix()  # y axis
                        Shoulder_R_tf_target.rotation = (rotation_noise_R * pin.Quaternion(np.cos(angle / 2), 0, -np.sin(angle / 2), 0)).toRotationMatrix()  # -y axis (镜像)
                        translation_noise = np.random.normal(0, noise_amplitude_translation, 3)
                        Shoulder_L_tf_target.translation -= (np.array([0.003,  0.002, 0.002]) + translation_noise)
                        Shoulder_R_tf_target.translation -= (np.array([0.003, -0.002, 0.002]) + translation_noise * np.array([1, -1, 1]))  # y 方向镜像

                    # 调用 IK 求解 (肩部坐标系输入)
                    arm_ik.solve_ik(Shoulder_L_tf_target.homogeneous, Shoulder_R_tf_target.homogeneous,
                                   use_shoulder_frame=USE_SHOULDER_FRAME,
                                   input_coordinate_frame='shoulder')
                else:
                    # 使用世界坐标系目标
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

                    # 调用 IK 求解 (世界坐标系输入)
                    arm_ik.solve_ik(L_tf_target.homogeneous, R_tf_target.homogeneous,
                                   use_shoulder_frame=USE_SHOULDER_FRAME,
                                   input_coordinate_frame='world')

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