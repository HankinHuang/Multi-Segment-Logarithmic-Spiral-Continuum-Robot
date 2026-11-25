# location.py

import numpy as np

def cartesian_to_cylindrical(x, y, z):
    """
    【新增】辅助函数：将直角坐标 (x, y, z) 转换为圆柱坐标 (r, phi_deg, z)。
    返回:
    r: 径向距离 (到z轴的距离)
    phi_deg: 方位角 (单位: 度)
    z: 高度 (与直角坐标系中的z相同)
    """
    # 计算径向距离 r
    r = np.sqrt(x**2 + y**2)
    # 使用 arctan2 计算方位角 phi (弧度)，它可以正确处理所有象限
    phi_rad = np.arctan2(y, x)
    # 将弧度转换为度
    phi_deg = np.rad2deg(phi_rad)
    return r, phi_deg, z

def get_transformation_matrix(L, theta_rad, phi_rad):
    """
    根据给定的段参数计算齐次变换矩阵 T。(此函数无需修改)
    L: 段的弧长
    theta_rad: 弯曲角度 (弧度)
    phi_rad: 弯曲方向 (弧度)
    """
    if theta_rad == 0:
        # 纯平移 (伸直状态)
        T = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, L],
            [0, 0, 0, 1]
        ])
        return T
    
    # 弯曲状态
    c_phi = np.cos(phi_rad)
    s_phi = np.sin(phi_rad)
    c_theta = np.cos(theta_rad)
    s_theta = np.sin(theta_rad)
    
    # 弯曲半径
    R = L / theta_rad
    
    T = np.array([
        [c_phi**2 * (c_theta - 1) + 1, s_phi*c_phi*(c_theta-1), c_phi*s_theta, R*c_phi*(1-c_theta)],
        [s_phi*c_phi*(c_theta-1), c_phi**2*(1-c_theta) + c_theta, s_phi*s_theta, R*s_phi*(1-c_theta)],
        [-c_phi*s_theta, -s_phi*s_theta, c_theta, R*s_theta],
        [0, 0, 0, 1]
    ])
    
    return T

def calculate_forward_kinematics(pose):
    """
    计算整个机器人的正向运动学。
    pose: 一个包含4个(phi_deg, theta_deg)元组的列表。
    【修改】返回: 一个包含4个 (r, phi_deg, z) 圆柱坐标元组的列表。
    """
    segment_params = [
        {'L_min': 244.48, 'L_max': 268.53, 'theta_max_deg': 90.0},
        {'L_min': 230.54, 'L_max': 259.73, 'theta_max_deg': 120.0},
        {'L_min': 178.87, 'L_max': 215.03, 'theta_max_deg': 150.0},
        {'L_min': 114.47, 'L_max': 155.82, 'theta_max_deg': 180.0}
    ]
    
    T_global = np.identity(4)
    end_positions_cylindrical = [] # 【修改】存储圆柱坐标结果
    
    for i in range(4):
        params = segment_params[i]
        phi_deg, theta_deg = pose[i]
        
        # --- 1. 将输入转换为弧度 ---
        phi_rad = np.deg2rad(phi_deg)
        theta_rad = np.deg2rad(theta_deg)
        theta_max_rad = np.deg2rad(params['theta_max_deg'])
        
        # --- 2. 计算可变中心轴长度 L ---
        if theta_max_rad == 0:
            angle_ratio = 0
        else:
            angle_ratio = theta_rad / theta_max_rad
        
        L = params['L_min'] + angle_ratio * (params['L_max'] - params['L_min'])
        
        # --- 3. 计算该段的变换矩阵 T_i ---
        T_i = get_transformation_matrix(L, theta_rad, phi_rad)
        
        # --- 4. 更新全局位姿矩阵 ---
        T_global = T_global @ T_i
        
        # --- 5. 提取直角坐标 [x, y, z] ---
        cartesian_pos = T_global[0:3, 3]
        
        # --- 6. 【修改】将直角坐标转换为圆柱坐标并存储 ---
        cylindrical_pos = cartesian_to_cylindrical(cartesian_pos[0], cartesian_pos[1], cartesian_pos[2])
        end_positions_cylindrical.append(cylindrical_pos)
        
    return end_positions_cylindrical

# --- 【修改】验证模块以展示圆柱坐标 ---
if __name__ == '__main__':
    # 示例: 只有第二段弯曲，方向为 90 度 (沿 Y 轴), 弯曲 60 度
    # pose = [(phi_1, theta_1), (phi_2, theta_2), (phi_3, theta_3), (phi_4, theta_4)]
    target_pose = [
        (0, 0),      # 第一段
        (0, 0),      # 第二段
        (0, 0),      # 第三段
        (0, 90)       # 第四段
    ]
    
    positions = calculate_forward_kinematics(target_pose)
    
    print("="*60)
    print(f"--- 计算姿态: {target_pose} (输出为圆柱坐标) ---")
    print("="*60)
    
    # Print
    print(f"第一段末端位置 P1: [r={positions[0][0]:.2f}, phi={positions[0][1]:.2f}°, z={positions[0][2]:.2f}] mm")
    print(f"第二段末端位置 P2: [r={positions[1][0]:.2f}, phi={positions[1][1]:.2f}°, z={positions[1][2]:.2f}] mm")
    print(f"第三段末端位置 P3: [r={positions[2][0]:.2f}, phi={positions[2][1]:.2f}°, z={positions[2][2]:.2f}] mm")
    print(f"第四段末端位置 P4: [r={positions[3][0]:.2f}, phi={positions[3][1]:.2f}°, z={positions[3][2]:.2f}] mm")
