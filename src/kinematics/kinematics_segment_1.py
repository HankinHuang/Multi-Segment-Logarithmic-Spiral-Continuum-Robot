# kinematics_segment_1.py

import numpy as np

class KinematicsSegment1:
    """
    第一段运动学模型。
    负责计算给定弯曲角度和方向下，驱动第一段的三根绳索的舵机目标步数。
    """
    def __init__(self, 
                 a=2.0, b=0.1764, 
                 spool_diameter=20.0, steps_per_revolution=4095,
                 num_segments=3, 
                 L_actual_straight=244.76,
                 theta_start=6.5*np.pi, theta_end=7*np.pi,
                 max_bend_angle=90.0):
        """
        初始化第一段运动学模型。
        """
        # --- 存储参数 ---
        self.a = a
        self.b = b
        self.spool_diameter = spool_diameter
        self.steps_per_revolution = steps_per_revolution
        self.num_segments = num_segments
        self.L_actual_straight = L_actual_straight
        self.max_bend_angle = max_bend_angle
        self.phi_ropes = np.array([np.deg2rad(0), np.deg2rad(120), np.deg2rad(240)])
        self.spool_circumference = self.spool_diameter * np.pi
        
        # --- 一次性预计算几何参数 ---
        self.delta_theta = np.pi / 6
        self.beta = np.exp(b * self.delta_theta)
        
        # 预计算理想状态下的最小绳长 (完全弯曲时，内侧绳索的长度)
        self.l_wire_min = (self.a * np.sqrt(self.b**2 + 1) / self.b) * \
                          (np.exp(self.b * theta_end) - np.exp(self.b * theta_start))
        
        # 注意: 第一段的d_robot是最小的关节，所以我们用它来生成更大的关节
        l_center_segments = [81.38] # 第一段的初始l_center_segments
        for i in range(1, self.num_segments):
            l_center_segments.append(l_center_segments[-1] * self.beta)
        self.L_ideal_straight = sum(l_center_segments)

    def calculate_servo_steps(self, theta_bend_deg, phi_bend_deg):
        """
        根据期望的弯曲角度和方向，计算舵机步数。

        返回:
        list[int]: 包含三根驱动绳 [S1, S2, S3] 的舵机目标步数。
        """
        if not (0 <= theta_bend_deg <= self.max_bend_angle):
            raise ValueError(f"弯曲角度 theta_bend_deg 必须在 0 到 {self.max_bend_angle} 度之间。")
        
        angle_ratio = theta_bend_deg / self.max_bend_angle
        
        # 1. 计算当前角度下的动态中心轴长度
        dynamic_center_length = self.L_actual_straight + angle_ratio * (self.L_ideal_straight - self.L_actual_straight)
        
        # 2. 计算当前弯曲程度下的最大长度变化量
        max_possible_change = self.L_ideal_straight - self.l_wire_min
        delta_L_max = angle_ratio * max_possible_change
        
        phi_bend_rad = np.deg2rad(phi_bend_deg)
        
        # 3. 使用动态中心轴长度作为基准计算各绳长
        target_lengths = []
        for phi_r in self.phi_ropes:
            length = dynamic_center_length - delta_L_max * np.cos(phi_r - phi_bend_rad)
            target_lengths.append(length)
            
        # 4. 将目标绳长转换为舵机步数
        servo_steps = []
        for l_target in target_lengths:
            delta_L = self.L_actual_straight - l_target
            steps = (delta_L / self.spool_circumference) * self.steps_per_revolution
            servo_steps.append(int(steps))
            
        return servo_steps

# --- 验证模块 ---
if __name__ == '__main__':
    print("="*60)
    print("--- 正在验证 KinematicsSegment1 类的计算结果 ---")
    print("="*60)
    
    # 使用新类进行计算
    kinematics_s1 = KinematicsSegment1()
    
    # 示例1
    theta_1 = 90.0
    phi_1 = 0.0
    servo_S_1 = kinematics_s1.calculate_servo_steps(theta_1, phi_1)
  
    # 打印新类的计算结果
    print("--- 新 Class 结构的输出 ---")
    print(f"  驱动绳1 (S1) 步数: {servo_S_1[0]}")
    print(f"  驱动绳2 (S2) 步数: {servo_S_1[1]}")
    print(f"  驱动绳3 (S3) 步数: {servo_S_1[2]}")
