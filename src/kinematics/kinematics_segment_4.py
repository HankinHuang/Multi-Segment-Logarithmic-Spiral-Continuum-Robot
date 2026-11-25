# kinematics_segment_4.py

import numpy as np

class KinematicsSegment4:
    """
    第四段运动学模型 (末端)。
    负责计算给定弯曲角度和方向下，驱动第四段的三根绳索的舵机目标步数。
    """
    def __init__(self, 
                 a=2.0, b=0.1764, 
                 spool_diameter=20, steps_per_revolution=4095,
                 num_segments=6, 
                 L_actual_straight=114.47,
                 theta_start=4*np.pi, theta_end=5*np.pi,
                 max_bend_angle=180.0):
        """
        初始化第四段运动学模型。
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
        
        self.l_wire_min = (self.a * np.sqrt(self.b**2 + 1) / self.b) * \
                          (np.exp(self.b * theta_end) - np.exp(self.b * theta_start))
        
        # 注意: 第四段的d_robot是最小的关节，所以我们用它来生成更大的关节
        l_center_segments = [20.36] # 第四段的初始l_center_segments
        for i in range(1, self.num_segments):
            l_center_segments.append(l_center_segments[-1] * self.beta)
        self.L_ideal_straight = sum(l_center_segments)

    def calculate_servo_steps(self, theta_bend_deg, phi_bend_deg):
        """
        根据期望的弯曲角度和方向，计算舵机步数。
        """
        if not (0 <= theta_bend_deg <= self.max_bend_angle):
            raise ValueError(f"弯曲角度 theta_bend_deg 必须在 0 到 {self.max_bend_angle} 度之间。")
        
        angle_ratio = theta_bend_deg / self.max_bend_angle
        dynamic_center_length = self.L_actual_straight + angle_ratio * (self.L_ideal_straight - self.L_actual_straight)
        max_possible_change = self.L_ideal_straight - self.l_wire_min
        delta_L_max = angle_ratio * max_possible_change
        phi_bend_rad = np.deg2rad(phi_bend_deg)
        
        target_lengths = []
        for phi_r in self.phi_ropes:
            length = dynamic_center_length - delta_L_max * np.cos(phi_r - phi_bend_rad)
            target_lengths.append(length)
            
        servo_steps = []
        for l_target in target_lengths:
            delta_L = self.L_actual_straight - l_target
            steps = (delta_L / self.spool_circumference) * self.steps_per_revolution
            servo_steps.append(int(steps))
            
        return servo_steps

# --- 验证模块 ---
if __name__ == '__main__':
    print("="*60)
    print("--- 正在验证 KinematicsSegment4 类的计算结果 ---")
    print("="*60)
    
    # 使用新类进行计算
    kinematics_s4 = KinematicsSegment4()
    
    # 示例1
    theta_1 = 180.0
    phi_1 = 0.0
    servo_S_1 = kinematics_s4.calculate_servo_steps(theta_1, phi_1)
   
    # 打印新类的计算结果
    print("--- 新 Class 结构的输出 ---")
    print(f"  驱动绳1 (S1) 步数: {servo_S_1[0]}")
    print(f"  驱动绳2 (S2) 步数: {servo_S_1[1]}")
    print(f"  驱动绳3 (S3) 步数: {servo_S_1[2]}")



