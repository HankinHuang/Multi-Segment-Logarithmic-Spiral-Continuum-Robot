# multi_turn_handler.py

import os
import sys
import time

_SRC_DIR = os.path.dirname(os.path.abspath(__file__))
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)
from scservo_sdk import *

class MultiTurnController:
    """
    一个用于舵机多圈模式的控制器。

    本控制器通过在软件中追踪每个舵机的绝对位置，解决了舵机硬件
    位置寄存器 (0-4095) 无法记录多圈累积位置的问题。

    工作原理:
    1. 初始化时，读取所有舵机的初始单圈位置，并将其作为绝对位置的起点。
    2. 当设置一个新的绝对位置目标时，它会计算出对应的单圈位置 (0-4095)。
    3. 它将这个计算出的单圈位置发送给舵机。舵机会自动选择最短路径移动到该位置。
    4. 指令发送成功后，控制器会更新其内部软件“账本”，将该舵机的绝对位置记录为目标值。
    
    这样，软件就拥有了舵机硬件所缺乏的“多圈记忆”。
    """

    def __init__(self, packetHandler, servo_ids):
        """
        初始化多圈控制器。

        :param packetHandler: scservo_sdk 中的 packetHandler 实例 (例如 sms_sts)。
        :param servo_ids: 一个包含所有需要控制的舵机ID的列表。
        """
        if not packetHandler:
            raise ValueError("packetHandler 实例不能为空。")
        
        self.packetHandler = packetHandler
        self.servo_ids = servo_ids
        # 这是核心：一个字典，用于存储每个舵机ID对应的理论绝对位置（单位：步）
        self.servo_absolute_positions = {}

        print("正在初始化多圈控制器...")
        self._initialize_positions()

    def _initialize_positions(self):
        """
        私有方法，用于在启动时读取所有舵机的初始位置。
        这是建立我们软件“账本”的第一步。
        """
        print("正在读取舵机初始位置...")
        all_servos_found = True
        for scs_id in self.servo_ids:
            # 使用 ReadPos 读取舵机的当前位置
            # 注意：ReadPos 返回的是 scs_tohost 转换后的值，范围可能为负，我们用0-4095范围即可
            scs_present_position, scs_comm_result, scs_error = self.packetHandler.ReadPos(scs_id)
            
            if scs_comm_result == COMM_SUCCESS:
                # 将舵机上电时的初始位置作为我们的绝对位置起点
                self.servo_absolute_positions[scs_id] = scs_present_position
                print(f"  - 舵机 ID {scs_id}: 初始位置读取成功 = {scs_present_position} 步")
            else:
                print(f"  - 错误: 无法读取舵机 ID {scs_id} 的位置。请检查连接和ID是否正确。")
                print(f"    {self.packetHandler.getTxRxResult(scs_comm_result)}")
                self.servo_absolute_positions[scs_id] = None  # 标记为读取失败
                all_servos_found = False
        
        if all_servos_found:
            print("所有舵机初始位置已记录。")
        else:
            print("警告: 部分舵机未能成功初始化。")

    def set_absolute_position(self, scs_id, target_absolute_steps, speed, acc):
        """
        将舵机移动到一个绝对位置。

        :param scs_id: 舵机ID。
        :param target_absolute_steps: 目标绝对位置（单位：步），可以远超4095。
        :param speed: 舵机移动速度。
        :param acc: 舵机加速度。
        :return: 如果指令发送成功则返回 True，否则返回 False。
        """
        if scs_id not in self.servo_ids:
            print(f"错误: 舵机 ID {scs_id} 不在本控制器的管理列表中。")
            return False

        if self.servo_absolute_positions.get(scs_id) is None:
            print(f"错误: 舵机 ID {scs_id} 未被成功初始化，无法移动。")
            return False

        # 核心计算：将绝对位置转换为单圈位置 (0-4095)
        # 舵机将自动朝这个位置以最短路径运动
        STEPS_PER_REVOLUTION = 4096
        target_in_one_turn = int(target_absolute_steps % STEPS_PER_REVOLUTION)

        # 发送位置写入指令
        scs_comm_result, scs_error = self.packetHandler.WritePosEx(scs_id, target_in_one_turn, speed, acc)
        
        if scs_comm_result != COMM_SUCCESS:
            print(f"舵机ID {scs_id} 移动失败: {self.packetHandler.getTxRxResult(scs_comm_result)}")
            return False
        elif scs_error != 0:
            print(f"舵机ID {scs_id} 返回错误: {self.packetHandler.getRxPacketError(scs_error)}")
            return False
        else:
            # 指令发送成功后，立即更新我们软件中的“账本”
            self.servo_absolute_positions[scs_id] = target_absolute_steps
            # print(f"舵机ID {scs_id}: 已发送目标 {target_in_one_turn} (绝对值 {target_absolute_steps})")
            return True

    def get_absolute_position(self, scs_id):
        """
        获取一个舵机当前被软件追踪的绝对位置。

        :param scs_id: 舵机ID。
        :return: 该舵机的理论绝对位置（单位：步），如果ID不存在则返回 None。
        """
        return self.servo_absolute_positions.get(scs_id)

# ------------------- 主程序: 用于演示和测试 -------------------
if __name__ == '__main__':
    # --- 初始化通信 (请根据您的实际情况修改) ---
    PORT_NAME = 'COM4'  # Linux: "/dev/ttyUSB0", Windows: "COM3"
    BAUDRATE = 1000000
    TEST_SERVO_ID = 11          # 修改为您要测试的舵机ID
    STEPS_PER_REVOLUTION = 4095

    portHandler = PortHandler(PORT_NAME)
    packetHandler = sms_sts(portHandler)

    if not portHandler.openPort():
        print(f"无法打开端口 {PORT_NAME}")
        quit()
    print(f"成功打开端口 {PORT_NAME}")

    if not portHandler.setBaudRate(BAUDRATE):
        print(f"无法设置波特率 {BAUDRATE}")
        quit()
    print(f"成功设置波特率 {BAUDRATE}")
    
    # --- 测试开始 ---
    # 创建控制器实例，管理我们测试的单个舵机
    controller = MultiTurnController(packetHandler, [TEST_SERVO_ID])
    
    # 检查初始化是否成功
    if controller.get_absolute_position(TEST_SERVO_ID) is None:
        print("控制器初始化失败，退出程序。")
        portHandler.closePort()
        quit()

    try:
        initial_pos = controller.get_absolute_position(TEST_SERVO_ID)
        print(f"\n--- 测试开始 ---")
        print(f"舵机 {TEST_SERVO_ID} 的初始绝对位置是: {initial_pos}\n")
        time.sleep(1)

        # 测试1: 向前移动超过一圈
        target1 = initial_pos + int(STEPS_PER_REVOLUTION * 1.5) # 移动1.5圈
        print(f"指令1: 移动到绝对位置 {target1} (前进1.5圈)...")
        controller.set_absolute_position(TEST_SERVO_ID, target1, 1200, 50)
        time.sleep(3) # 等待舵机移动
        print(f"移动后，软件记录的绝对位置: {controller.get_absolute_position(TEST_SERVO_ID)}\n")
        
        # 测试2: 从当前位置向后移动超过两圈
        target2 = target1 + int(STEPS_PER_REVOLUTION * -2.2) # 移动-2.2圈
        print(f"指令2: 移动到绝对位置 {target2} (后退2.2圈)...")
        controller.set_absolute_position(TEST_SERVO_ID, target2, 1200, 50)
        time.sleep(4) # 等待舵机移动
        print(f"移动后，软件记录的绝对位置: {controller.get_absolute_position(TEST_SERVO_ID)}\n")

        # 测试3: 回到初始位置
        print(f"指令3: 移动回到初始位置 {initial_pos}...")
        controller.set_absolute_position(TEST_SERVO_ID, initial_pos, 1200, 50)
        time.sleep(4) # 等待舵机移动
        print(f"移动后，软件记录的绝对位置: {controller.get_absolute_position(TEST_SERVO_ID)}\n")

    except KeyboardInterrupt:
        print("程序被用户中断。")
    finally:
        # 程序结束时关闭端口
        portHandler.closePort()
        print("端口已关闭。")
