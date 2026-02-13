#!/usr/bin/env python3
"""
简洁的真机神经网络IK控制脚本

功能：
- 左臂：神经网络IK（PieperCausalIK）
- 右臂：传统优化IK（CasADi + IPOPT）
- VR输入：手部追踪或控制器
- 控制频率：30Hz

使用：
    python teleop_hand_and_arm_nn.py --input-mode hand --motion
    python teleop_hand_and_arm_nn.py --input-mode controller --motion

控制：
    按 [r] 开始跟踪
    按 [q] 退出并回到原点
"""

import time
import argparse
import threading
import logging_mp
logging_mp.basic_config(level=logging_mp.INFO)
logger_mp = logging_mp.get_logger(__name__)

import os
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from televuer import TeleVuerWrapper
from teleop.robot_control.robot_arm import G1_29_ArmController
from teleop.robot_control.robot_arm_ik_nn import G1_29_ArmIK
from teleimager.image_client import ImageClient
from teleop.utils.motion_switcher import MotionSwitcher
from sshkeyboard import listen_keyboard, stop_listening

# 全局状态
START = False  # 控制是否进入主循环（按r键）
ACTION = False  # 控制是否发送机器人指令（按a键切换）
STOP = False  # 控制是否退出程序


def on_press(key):
    """键盘控制回调"""
    global START, ACTION, STOP
    if key == 'r':
        START = True
        logger_mp.info("🚀 进入主控制循环")
    elif key == 'a':
        ACTION = not ACTION
        status = "✅ 开始发送机器人指令" if ACTION else "⏸️  停止发送机器人指令"
        logger_mp.info(status)
    elif key == 'q':
        STOP = True
        logger_mp.info("退出程序...")


if __name__ == '__main__':
    # ========== 参数解析 ==========
    parser = argparse.ArgumentParser(description='真机神经网络IK控制（左臂NN + 右臂传统）')
    parser.add_argument('--frequency', type=float, default=30.0, help='控制频率 (Hz)')
    parser.add_argument('--input-mode', type=str, choices=['hand', 'controller'],
                        default='hand', help='输入模式: hand=手部追踪, controller=控制器')
    parser.add_argument('--display-mode', type=str, choices=['immersive', 'ego', 'pass-through'],
                        default='immersive', help='VR显示模式')
    parser.add_argument('--img-server-ip', type=str, default='192.168.123.164',
                        help='图像服务器IP地址')
    parser.add_argument('--motion', action='store_true',
                        help='启用运动模式（G1: R1+X常规模式，非R2+A跑步模式）')
    parser.add_argument('--network-interface', type=str, default=None,
                        help='DDS网络接口，如 eth0, wlan0。None则使用默认接口')

    args = parser.parse_args()
    logger_mp.info(f"参数: {args}")

    try:
        # ========== DDS初始化 ==========
        ChannelFactoryInitialize(0, networkInterface=args.network_interface)
        logger_mp.info("DDS通信初始化完成")

        # ========== 键盘监听 ==========
        listen_keyboard_thread = threading.Thread(
            target=listen_keyboard,
            kwargs={"on_press": on_press, "until": None, "sequential": False},
            daemon=True
        )
        listen_keyboard_thread.start()
        logger_mp.info("键盘监听已启动")

        # ========== 图像客户端 ==========
        img_client = ImageClient(host=args.img_server_ip)
        camera_config = img_client.get_cam_config()
        logger_mp.debug(f"摄像头配置: {camera_config}")

        xr_need_local_img = not (
            args.display_mode == 'pass-through' or
            camera_config['head_camera']['enable_webrtc']
        )

        # ========== VR输入包装器 ==========
        tv_wrapper = TeleVuerWrapper(
            use_hand_tracking=(args.input_mode == "hand"),
            binocular=camera_config['head_camera']['binocular'],
            img_shape=camera_config['head_camera']['image_shape'],
            display_mode=args.display_mode,
            zmq=camera_config['head_camera']['enable_zmq'],
            webrtc=camera_config['head_camera']['enable_webrtc'],
            webrtc_url=f"https://{args.img_server_ip}:{camera_config['head_camera']['webrtc_port']}/offer",
        )
        logger_mp.info("VR输入包装器已初始化")

        # ========== 运动模式切换 ==========
        if not args.motion:
            motion_switcher = MotionSwitcher()
            status, result = motion_switcher.Enter_Debug_Mode()
            logger_mp.info(f"进入调试模式: {'成功' if status == 0 else '失败'}")

        # ========== 机器人控制（神经网络IK版本） ==========
        logger_mp.info("初始化机器人控制（包含神经网络IK）...")
        arm_ik = G1_29_ArmIK(Visualization=True)  # 已包含神经网络IK
        arm_ctrl = G1_29_ArmController(motion_mode=args.motion, simulation_mode=False)
        logger_mp.info("机器人控制初始化完成")

        # ========== 等待启动 ==========
        logger_mp.info("=" * 60)
        logger_mp.info("🟢  按 [r] 进入主控制循环（meshcat开始更新）")
        logger_mp.info("🟡  按 [a] 发送机器人控制指令（可在循环中切换）")
        logger_mp.info("🔴  按 [q] 停止并退出程序")
        logger_mp.info("⚠️  重要：请保持安全距离")
        logger_mp.info("=" * 60)

        while not START and not STOP:
            time.sleep(0.033)
            if camera_config['head_camera']['enable_zmq'] and xr_need_local_img:
                head_img, _ = img_client.get_head_frame()
                tv_wrapper.render_to_xr(head_img)

        if STOP:
            logger_mp.info("程序已退出")
            exit(0)

        logger_mp.info("=" * 60)
        logger_mp.info("🚀 已进入主控制循环")
        logger_mp.info("📊 meshcat实时更新中")
        logger_mp.info("🟡  按 [a] 开始/停止发送机器人指令")
        logger_mp.info("🔴  按 [q] 退出程序")
        logger_mp.info("=" * 60)

        # ========== 速度渐加速 ==========
        arm_ctrl.speed_gradual_max()

        # ========== 主控制循环 ==========
        frame_count = 0
        while not STOP:
            start_time = time.time()

            # 获取图像并渲染到VR
            if camera_config['head_camera']['enable_zmq']:
                if xr_need_local_img:
                    head_img, head_img_fps = img_client.get_head_frame()
                    tv_wrapper.render_to_xr(head_img)

            # 获取VR遥操作数据
            tele_data = tv_wrapper.get_tele_data()

            # 获取当前机器人状态
            current_lr_arm_q = arm_ctrl.get_current_dual_arm_q()
            current_lr_arm_dq = arm_ctrl.get_current_dual_arm_dq()

            # IK求解（神经网络左臂 + 传统右臂）
            time_ik_start = time.time()
            sol_q, sol_tauff = arm_ik.solve_ik(
                tele_data.left_wrist_pose,
                tele_data.right_wrist_pose,
                current_lr_arm_q,
                current_lr_arm_dq
            )
            time_ik_end = time.time()

            if frame_count % 30 == 0:  # 每30帧记录一次
                logger_mp.debug(f"IK求解耗时: {round(time_ik_end - time_ik_start, 6)}s")

            # 打印IK求解结果（3Hz = 每10帧打印一次）
            if frame_count % 10 == 0:
                logger_mp.info(f"[{'控制' if ACTION else '观察'}] sol_q: {sol_q}")
                logger_mp.info(f"[{'控制' if ACTION else '观察'}] sol_tauff: {sol_tauff}")

            # 发送控制指令到机器人（只有ACTION=True时才控制）
            if ACTION:
                arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)
            else:
                logger_mp.debug("ACTION=False，不发送控制指令")

            # 控制频率控制
            current_time = time.time()
            elapsed = current_time - start_time
            sleep_time = max(0, (1 / args.frequency) - elapsed)
            time.sleep(sleep_time)

            frame_count += 1

    except KeyboardInterrupt:
        logger_mp.info("⛔ 键盘中断，退出程序...")
    except Exception as e:
        import traceback
        logger_mp.error(traceback.format_exc())
    finally:
        # ========== 清理工作 ==========
        logger_mp.info("清理资源...")

        try:
            logger_mp.info("回到原点...")
            arm_ctrl.ctrl_dual_arm_go_home()
            logger_mp.info("已回到原点")
        except Exception as e:
            logger_mp.error(f"回到原点失败: {e}")

        try:
            logger_mp.info("停止键盘监听...")
            stop_listening()
            listen_keyboard_thread.join(timeout=2.0)
            logger_mp.info("键盘监听已停止")
        except Exception as e:
            logger_mp.error(f"停止键盘监听失败: {e}")
            # 强制停止：尝试恢复终端设置
            try:
                import subprocess
                subprocess.run(["stty", "sane"], capture_output=True)
                logger_mp.info("已尝试恢复终端设置")
            except:
                pass

        try:
            img_client.close()
            logger_mp.info("图像客户端已关闭")
        except Exception as e:
            logger_mp.error(f"关闭图像客户端失败: {e}")

        try:
            tv_wrapper.close()
            logger_mp.info("VR包装器已关闭")
        except Exception as e:
            logger_mp.error(f"关闭VR包装器失败: {e}")

        try:
            if not args.motion:
                pass
                # 可选：退出调试模式
                # status, result = motion_switcher.Exit_Debug_Mode()
                # logger_mp.info(f"退出调试模式: {'成功' if status == 3104 else '失败'}")
        except Exception as e:
            logger_mp.error(f"退出调试模式失败: {e}")

        logger_mp.info("✅ 程序退出")
        exit(0)
