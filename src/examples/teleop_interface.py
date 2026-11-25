# 1. 导入我们需要的库
import pygame
import sys

# --- 全局设置 ---
# 窗口尺寸
WINDOW_WIDTH = 600
WINDOW_HEIGHT = 400

# 舵机数量
NUM_SERVOS = 12

# 舵机角度的初始值、最小值、最大值和步长
INITIAL_ANGLE = 90
MIN_ANGLE = 0
MAX_ANGLE = 720 # 未知
STEP_ANGLE = 1  # 每次按键改变1度

# -----------------------------------------------------------------------------
# 2. 定义我们的主控制类 (可以想象成是机器人的“驾驶舱”)
class TeleopController:
    def __init__(self):
        # 初始化Pygame
        pygame.init()

        # 设置窗口标题
        pygame.display.set_caption("VLA遥操作界面")

        # 创建窗口
        self.screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))

        # 创建一个列表，用来存放12个舵机的当前角度
        # 我们让所有舵机都从90度开始
        self.servo_angles = [INITIAL_ANGLE] * NUM_SERVOS

        # 定义键盘按键和它所控制的舵机以及方向
        # 格式: pygame的按键名: (舵机编号, 改变方向)，共12个舵机
        # 改变方向: 1 代表增加角度, -1 代表减少角度
        self.key_mapping = {
            pygame.K_q: (0, -1), pygame.K_a: (0, 1),   # Q/A 控制 舵机0
            pygame.K_w: (1, -1), pygame.K_s: (1, 1),   # W/S 控制 舵机1
            pygame.K_e: (2, -1), pygame.K_d: (2, 1),   # E/D 控制 舵机2
            pygame.K_r: (3, -1), pygame.K_f: (3, 1),   # R/F 控制 舵机3
            pygame.K_t: (4, -1), pygame.K_g: (4, 1),   # T/G 控制 舵机4
            pygame.K_y: (5, -1), pygame.K_h: (5, 1),   # Y/H 控制 舵机5
            pygame.K_u: (6, -1), pygame.K_j: (6, 1),   # U/J 控制 舵机6
            pygame.K_i: (7, -1), pygame.K_k: (7, 1),   # I/K 控制 舵机7
            pygame.K_o: (8, -1), pygame.K_l: (8, 1),   # O/L 控制 舵机8
            pygame.K_z: (9, -1), pygame.K_x: (9, 1),   # Z/X 控制 舵机9
            pygame.K_c: (10, -1), pygame.K_v: (10, 1),   # C/V 控制 舵机10
            pygame.K_b: (11, -1), pygame.K_n: (11, 1),   # B/N 控制 舵机11
        }
    
    # 3. 打印舵机角度的函数 (这是我们的核心输出)
    def print_servo_angles(self):
        # 将列表中的每个角度都格式化成3位数，方便阅读
        formatted_angles = [f"{angle:03}" for angle in self.servo_angles]
        
        # 用逗号连接起来，并打印到终端
        # 使用 \r 可以让新信息覆盖旧信息，看起来更清爽
        print(f"舵机角度: [{', '.join(formatted_angles)}]", end='\r')

    # 4. 主循环 (程序的核心，会一直运行)
    def run(self):
        running = True
        while running:
            # --- 事件处理 ---
            # 检查窗口中发生的所有事件 (比如按键、关闭窗口)
            for event in pygame.event.get():
                # 如果用户点击了窗口的关闭按钮
                if event.type == pygame.QUIT:
                    running = False # 结束循环
                
                # 如果用户按下了键盘上的某个键
                if event.type == pygame.KEYDOWN:
                    # 检查按下的键是否在我们定义的 key_mapping 中
                    if event.key in self.key_mapping:
                        # 获取这个键对应的舵机编号和改变方向
                        servo_index, direction = self.key_mapping[event.key]
                        
                        # 计算新的角度
                        new_angle = self.servo_angles[servo_index] + (direction * STEP_ANGLE)
                        
                        # 确保新角度不会超出范围 (0-180度)
                        if MIN_ANGLE <= new_angle <= MAX_ANGLE:
                            self.servo_angles[servo_index] = new_angle
                            # 每当角度变化时，就打印一次最新的状态
                            self.print_servo_angles()

            # --- 画面渲染 ---
            # 用纯黑色填充窗口背景
            self.screen.fill((0, 0, 0)) # RGB颜色: (黑)

            # --- 更新显示 ---
            # 将我们画的所有东西，一次性显示在屏幕上
            pygame.display.flip()

        # --- 退出程序 ---
        pygame.quit()
        sys.exit()

# -----------------------------------------------------------------------------
# 5. 程序的入口 (当我们运行这个文件时，从这里开始)
if __name__ == '__main__':
    # 创建一个我们驾驶舱的实例
    controller = TeleopController()
    
    # 打印一下初始角度
    print("遥操作界面已启动。请按键盘Q/A, W/S, E/D等来控制舵机。")
    controller.print_servo_angles()
    
    # 运行主循环
    controller.run()
