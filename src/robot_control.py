# robot_control.py

import os
import sys
import time
import math

# Ensure src/ is on sys.path whether invoked from repository root or src/
_SRC_DIR = os.path.dirname(os.path.abspath(__file__))
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)

# --- 硬件驱动与运动学模块导入 ---
from scservo_sdk import *
from multi_turn_handler import MultiTurnController
from kinematics.kinematics_segment_1 import KinematicsSegment1
from kinematics.kinematics_segment_2 import KinematicsSegment2
from kinematics.kinematics_segment_3 import KinematicsSegment3
from kinematics.kinematics_segment_4 import KinematicsSegment4

class RobotController:
    # --- 硬件配置常量 ---
    PORT_NAME = 'COM3' # 请确保这是您正确的串口号
    BAUDRATE = 1000000
    SERVO_IDS_BY_SEGMENT = [
        [11, 12, 13], [21, 22, 23], [31, 32, 33], [41, 42, 43],
    ]
    ALL_SERVO_IDS = [id for segment in SERVO_IDS_BY_SEGMENT for id in segment]
    MOVE_SPEED = 1500
    MOVE_ACC = 50

    def __init__(self):
        print("\n" + "="*60)
        print("正在初始化机器人主控制器 (角度耦合模型)...")
        
        self.portHandler = PortHandler(self.PORT_NAME)
        self.packetHandler = sms_sts(self.portHandler)
        self.position_handler = None
        self.zero_positions = {}
        self.is_connected = False
        
        print("正在加载各段运动学模型...")
        self.kin_modules = {
            1: KinematicsSegment1(),
            2: KinematicsSegment2(),
            3: KinematicsSegment3(),
            4: KinematicsSegment4()
        }
        print("运动学模型加载完成。")
        print("="*60)

    def connect(self):
        # (此函数无需修改)
        print("\n--- 正在连接机器人硬件 ---")
        try:
            if not self.portHandler.openPort(): raise IOError(f"无法打开串口 {self.PORT_NAME}")
            print(f"成功打开串口 {self.PORT_NAME}")
            if not self.portHandler.setBaudRate(self.BAUDRATE): raise IOError(f"无法设置波特率 {self.BAUDRATE}")
            print(f"成功设置波特率 {self.BAUDRATE}")
            self.position_handler = MultiTurnController(self.packetHandler, self.ALL_SERVO_IDS)
            if any(pos is None for pos in self.position_handler.servo_absolute_positions.values()):
                raise ConnectionError("未能初始化所有舵机的位置。")
            self.zero_positions = self.position_handler.servo_absolute_positions.copy()
            print("\n--- 所有舵机“物理零点”位置已记录 ---")
            for scs_id, pos in self.zero_positions.items():
                print(f"  - 舵机ID {scs_id}: 物理零点 = {pos} 步")
            self.is_connected = True
            print("\n--- 机器人已连接并准备就绪 ---")
            return True
        except (IOError, ConnectionError) as e:
            print(f"连接失败: {e}")
            self.disconnect()
            return False

    def disconnect(self):
        # (此函数无需修改)
        if self.is_connected:
            print("\n--- 正在断开机器人连接 ---")
            print("正在将机器人恢复到初始物理零点...")
            self.go_home()
            time.sleep(3)
            print("正在释放所有舵机的力矩...")
            for scs_id in self.ALL_SERVO_IDS:
                self.packetHandler.write1ByteTxRx(scs_id, SMS_STS_TORQUE_ENABLE, 0)
            self.portHandler.closePort()
            print("串口已关闭。")
        self.is_connected = False
        print("--- 机器人已安全断开 ---")

    # --------------------------------------------------------------------
    # 核心修正点
    # --------------------------------------------------------------------
    def calculate_absolute_steps_for_pose(self, pose):
        """
        根据一个完整的姿态指令，计算所有舵机相对于理论零点的总步数。
        pose: 一个包含4个(phi, theta)元组的列表。
        """
        total_theoretical_steps = {scs_id: 0 for scs_id in self.ALL_SERVO_IDS}

        for driving_seg_idx in range(1, 5):
            phi_deg, theta_deg = pose[driving_seg_idx - 1]
            if theta_deg == 0:
                continue

            print(f"\n  - 计算由【第 {driving_seg_idx} 段】弯曲 ({phi_deg}°, {theta_deg}°) 产生的效应...")

            for affected_seg_idx in range(driving_seg_idx, 5):
                kin_module = self.kin_modules[affected_seg_idx]
                affected_servo_ids = self.SERVO_IDS_BY_SEGMENT[affected_seg_idx - 1]
                
                # --- 核心逻辑修正 ---
                # 1. 先计算出基础的步数列表
                base_delta_steps = kin_module.calculate_servo_steps(theta_deg, phi_deg)
                
                # 2. 判断是主动段还是耦合段
                if affected_seg_idx > driving_seg_idx:
                    # 这是耦合效应，应用0.8的折扣系数
                    coupling_factor = 0.8
                    # 使用列表推导式，将系数应用到列表的每个元素上
                    final_delta_steps = [int(step * coupling_factor) for step in base_delta_steps]
                    print(f"    -> 对【第 {affected_seg_idx} 段】产生耦合步数: {base_delta_steps} * {coupling_factor} = {final_delta_steps}")
                else:
                    # 这是主动段自身的运动，不打折
                    final_delta_steps = base_delta_steps
                    print(f"    -> 对【第 {affected_seg_idx} 段】(自身)产生主动步数: {final_delta_steps}")
                # --------------------

                for i in range(3):
                    scs_id = affected_servo_ids[i]
                    total_theoretical_steps[scs_id] += final_delta_steps[i]

        return total_theoretical_steps

    def move_to_pose(self, target_pose):
        # (此函数无需修改)
        if not self.is_connected:
            print("错误: 机器人未连接。")
            return None

        print("\n--- 正在计算目标姿态的累积步数 ---")
        target_theoretical_steps = self.calculate_absolute_steps_for_pose(target_pose)
        
        current_physical_positions = self.position_handler.servo_absolute_positions.copy()
        
        target_physical_positions = {}
        for scs_id, theoretical_steps in target_theoretical_steps.items():
            target_physical_positions[scs_id] = self.zero_positions[scs_id] + theoretical_steps
        
        print("\n--- 准备同步写入指令包 ---")
        groupSyncWrite = self.packetHandler.groupSyncWrite
        groupSyncWrite.clearParam()
        for scs_id, target_pos in target_physical_positions.items():
            position_to_send = int(target_pos)
            print(f"  - 舵机ID {scs_id}: 目标物理位置 = {position_to_send}")
            add_param_ok = self.packetHandler.SyncWritePosEx(scs_id, position_to_send, self.MOVE_SPEED, self.MOVE_ACC)
            if not add_param_ok: print(f"  ! 警告: 添加舵机ID {scs_id} 失败。")
        
        print("\n正在执行同步运动...")
        scs_comm_result = groupSyncWrite.txPacket()
        if scs_comm_result != COMM_SUCCESS:
            print(f"!!! 同步运动执行失败: {self.packetHandler.getTxRxResult(scs_comm_result)}")
        else:
            print("--- 同步运动指令发送成功 ---")
            for scs_id, target_pos in target_physical_positions.items():
                self.position_handler.servo_absolute_positions[scs_id] = target_pos
            print("多圈控制器内部状态已更新。")
        groupSyncWrite.clearParam()
        
        delta_steps = {id: target_physical_positions[id] - current_physical_positions[id] for id in self.ALL_SERVO_IDS}
        return {
            "delta_steps": delta_steps,
            "current_positions": current_physical_positions,
            "target_positions": target_physical_positions
        }

    def go_home(self):
        # (此函数无需修改)
        home_pose = [(0, 0), (0, 0), (0, 0), (0, 0)]
        print("\n正在执行归位(Home)操作...")
        return self.move_to_pose(home_pose)

# --- 主程序入口 ---
if __name__ == "__main__":
    robot = RobotController()
    
    def print_move_result(move_result):
        # (此函数无需修改)
        if not move_result: return
        print("\n--- 计算结果: 各舵机步数变化 (Delta Steps) ---")
        for seg in range(1, 5):
            ids = robot.SERVO_IDS_BY_SEGMENT[seg-1]
            line = f"  段 {seg} (舵机 {ids}): "
            steps = [move_result['delta_steps'][id] for id in ids]
            print(line + f"[{steps[0]:+7d}, {steps[1]:+7d}, {steps[2]:+7d}]")
            
        print("\n--- 机器人状态: 舵机物理绝对位置 (Physical Positions) ---")
        for seg in range(1, 5):
            ids = robot.SERVO_IDS_BY_SEGMENT[seg-1]
            line = f"  段 {seg} (舵机 {ids}): "
            old_pos = [move_result['current_positions'][id] for id in ids]
            new_pos = [move_result['target_positions'][id] for id in ids]
            print(line + f"从 [{old_pos[0]:7d}, {old_pos[1]:7d}, {old_pos[2]:7d}] -> 至 [{new_pos[0]:7d}, {new_pos[1]:7d}, {new_pos[2]:7d}]")
        print("#"*70)

    try:
        if robot.connect():
            print("\n\n" + "#"*70)
            print("### 欢迎来到连续体机器人【硬件控制台 - 角度耦合模型】 ###")
            print("#"*70)
            print("指令: 'move' (移动到新姿态), 'home' (归零), 'status' (查看状态), 'quit' (退出)")
            while True:
                command = input("\n请输入指令: ").lower()
                if command in ['q', 'quit']: break
                elif command == 'home':
                    result = robot.go_home()
                    print_move_result(result)
                elif command == 'status':
                    positions = robot.position_handler.servo_absolute_positions.copy()
                    print("\n--- 当前机器人物理绝对位置 ---")
                    for seg in range(1, 5):
                        ids = robot.SERVO_IDS_BY_SEGMENT[seg-1]
                        line = f"  段 {seg} (舵机 {ids}): "
                        pos = [positions[id] for id in ids]
                        print(line + f"[{pos[0]:7d}, {pos[1]:7d}, {pos[2]:7d}]")
                elif command == 'move':
                    target_pose = []
                    print("\n--- 请输入目标姿态 (共4段) ---")
                    for i in range(1, 5):
                        kin_module = robot.kin_modules[i]
                        theta_max = kin_module.max_bend_angle
                        print(f"\n--- 配置【第 {i} 段】(最大弯曲 {theta_max}°)...")
                        phi_deg = float(input(f"  - 弯曲方向 Φ (0-360): "))
                        theta_deg = float(input(f"  - 弯曲程度 θ (0-{theta_max}): "))
                        if not (0 <= theta_deg <= theta_max):
                            print(f"警告: 第{i}段的弯曲程度超出范围，将自动修正。")
                            theta_deg = max(0, min(theta_deg, theta_max))
                        target_pose.append((phi_deg, theta_deg))
                    result = robot.move_to_pose(target_pose)
                    print_move_result(result)
                else:
                    print("无效指令。")
    except KeyboardInterrupt:
        print("\n检测到用户中断 (Ctrl+C)，将安全退出。")
    finally:
        robot.disconnect()