# -*- coding: utf-8 -*-
"""
二级齿轮传动参数化设计脚本 (以直径为核心) - Python版本
目标：在满足输出性能约束下，最小化总重量
方法：参数化扫描优化
更新：速度约束改为固定输出转速，反推电机转速需求
"""

import math
from dataclasses import dataclass
from typing import Optional

# ==================== 设定参数 (根据你的需求调节) ====================

# 电机参考参数
motor_D = 75              # 电机直径 (mm)，作为模组包络参考
output_speed = 260        # 输出转速要求：固定为 rpm
output_torque = 7         # 输出扭矩要求：大于 Nm
motor_torque_lower = 0.7  # 电机额定扭矩下限 Nm，按照下限计算
motor_torque_upper = 1.2  # 电机额定扭矩上限 Nm
motor_weight = 300        # 电机重量，克



# 优化模式选择
# 'weight'  = 重量最轻优化（默认）
# 'size'    = 纵向尺寸最短优化（总尺寸 = motor_D/2 + D1/2 + D2/2 + D3/2 + D4）
OPTIMIZATION_MODE = 'weight'  # 可选: 'weight' 或 'size'


# 速度约束
max_motor_speed = 3000    # 电机最大允许转速 rpm
# 电机类型	KV 值 (RPM/V)	24V 理论空载转速	实际安全上限
# 高扭矩型（机器人关节）	80~150	1900~3600 RPM	1500~2500 RPM
# 通用型（云台/航模）	150~250	3600~6000 RPM	2500~4000 RPM
# 高转速型	250~400	6000~9600 RPM	4000~7000 RPM


# ==================== 材料选择与强度约束 ====================

# 材质选择配置
# 'auto'    = 自动选择最优材质（基于强度计算）
# 'steel'   = 钢材 (20CrMnTi/40Cr等)
# 'peek'    = PEEK (聚醚醚酮)
# 'pom'     = POM (聚甲醛)
# 'nylon'   = 尼龙66
MATERIAL_MODE = 'auto'  # 可选: 'auto', 'steel', 'peek', 'pom', 'nylon'

# 材料强度数据库 (单位: MPa)
MATERIAL_DB = {
    'steel': {
        'name': '合金钢 (20CrMnTi)',
        'density': 7.85e-6,      # kg/mm³
        'sigma_b': 800,          # 抗拉强度 (MPa)
        'sigma_f': 600,          # 弯曲疲劳极限 (MPa)
        'sf_min_ratio': 0.3,     # 最小齿根厚/模数比
        'safety_factor': 2.0,    # 安全系数
        'color': '金属灰'
    },
    'peek': {
        'name': 'PEEK (聚醚醚酮)',
        'density': 1.32e-6,      # kg/mm³
        'sigma_b': 100,          # 抗拉强度 (MPa)
        'sigma_f': 90,           # 弯曲疲劳极限 (MPa)
        'sf_min_ratio': 0.8,     # 最小齿根厚/模数比 (塑料需要更厚)
        'safety_factor': 2.5,    # 安全系数
        'color': '米黄色'
    },
    'pom': {
        'name': 'POM (聚甲醛)',
        'density': 1.41e-6,      # kg/mm³
        'sigma_b': 70,           # 抗拉强度 (MPa)
        'sigma_f': 65,           # 弯曲疲劳极限 (MPa)
        'sf_min_ratio': 1.0,     # 最小齿根厚/模数比
        'safety_factor': 3.0,    # 安全系数
        'color': '白色/黑色'
    },
    'nylon': {
        'name': '尼龙66 (PA66)',
        'density': 1.15e-6,      # kg/mm³
        'sigma_b': 80,           # 抗拉强度 (MPa)
        'sigma_f': 70,           # 弯曲疲劳极限 (MPa)
        'sf_min_ratio': 0.9,     # 最小齿根厚/模数比
        'safety_factor': 2.8,    # 安全系数
        'color': '乳白色'
    }
}

# 材料参数 (当前选定材质, auto模式下会被覆盖)
rho_steel = 7.85e-6       # 钢材密度 (kg/mm^3)

# ==================== 优化参数设置 ====================

# 模数选择范围 (标准模数)，一级用 0.5 或 0.8，二级用 1.0 或 1.25
# 可选模数，第一系列（首选）： 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.8, 1.0, 1.25, 1.5, 2.0, 2.5, 3.0

m1_options = [0.3, 0.4, 0.5, 0.8, 1.0, 1.25, 1.5]      # 第一级模数选项 (高速级选小模数)
m2_options = [0.5, 0.8, 1.0, 1.25, 1.5, 2.0, 2.5]      # 第二级模数选项 (低速级选大模数)

# 直径扫描范围 (以直径为核心变量)
D1_min, D1_max = 20, 40     # 主动轮1直径范围 (mm)
D2_min, D2_max = 50, 100    # 从动轮2直径范围 (mm)
D3_min, D3_max = 18, 40     # 主动轮3直径范围 (mm)
D4_min, D4_max = 40, 80     # 从动轮4直径范围 (mm)

# ==================== 齿宽(轴向厚度)系数设置 ====================
# 齿宽 = 系数 × 模数，经验值：6-12
# 齿轮轴向厚度 = 齿宽系数 × 模数

# 是否优化齿宽系数
OPTIMIZE_WIDTH_FACTOR = True  # True = 扫描优化齿宽系数, False = 使用固定值

# 固定齿宽系数 (当 OPTIMIZE_WIDTH_FACTOR = False 时使用)
width_factor1_fixed = 8   # 第一级齿宽系数
width_factor2_fixed = 10  # 第二级齿宽系数 (低速级更宽)

# 齿宽系数扫描范围 (当 OPTIMIZE_WIDTH_FACTOR = True 时使用)
width_factor1_min, width_factor1_max = 6, 12   # 第一级齿宽系数范围
width_factor2_min, width_factor2_max = 8, 16   # 第二级齿宽系数范围 (低速级更宽)
width_factor_step = 1                          # 齿宽系数步长 (整数)
# 压力角
gear_pressure_angle = 20  # 压力角 20度
# 步长设置
step = 1  # 直径扫描步长 (mm)


# ==================== 颜色定义 ====================
class Color:
    """ANSI 颜色码"""
    RESET = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    
    # 前景色
    RED = '\033[31m'
    GREEN = '\033[32m'
    YELLOW = '\033[33m'
    BLUE = '\033[34m'
    MAGENTA = '\033[35m'
    CYAN = '\033[36m'
    WHITE = '\033[37m'
    
    # 亮前景色
    BRIGHT_RED = '\033[91m'
    BRIGHT_GREEN = '\033[92m'
    BRIGHT_YELLOW = '\033[93m'
    BRIGHT_BLUE = '\033[94m'
    BRIGHT_MAGENTA = '\033[95m'
    BRIGHT_CYAN = '\033[96m'
    BRIGHT_WHITE = '\033[97m'
    
    # 背景色
    BG_RED = '\033[41m'
    BG_GREEN = '\033[42m'
    BG_YELLOW = '\033[43m'
    BG_BLUE = '\033[44m'


def colorize(text, color):
    """为文本添加颜色"""
    return f"{color}{text}{Color.RESET}"


# ==================== 辅助函数 ====================
def iif(cond, a, b):
    """条件选择函数"""
    return a if cond else b


def calculate_bending_stress(m, z, width, torque, alpha_deg=20):
    """
    计算齿根弯曲应力 (MPa)
    使用简化公式: σ_F = (2 * T * Y_F * Y_S * Y_ε) / (b * m² * z)
    
    参数:
        m: 模数 (mm)
        z: 齿数
        width: 齿宽 (mm)
        torque: 传递扭矩 (N·mm)
        alpha_deg: 压力角 (度)
    
    返回:
        sigma_F: 弯曲应力 (MPa)
        Y_F: 齿形系数
    """
    if z < 12 or width <= 0 or m <= 0:
        return float('inf'), 0
    
    # 齿形系数 Y_F (简化公式, 适用于标准齿轮)
    # Y_F ≈ 2.1 + 3.5/z (经验公式)
    Y_F = 2.1 + 3.5 / z
    
    # 应力修正系数 Y_S ≈ 1.0 (简化)
    Y_S = 1.0
    
    # 重合度系数 Y_ε ≈ 0.7 (简化)
    Y_epsilon = 0.7
    
    # 扭矩转换为 N·mm
    T = torque * 1000  # N·mm
    
    # 弯曲应力计算 (MPa)
    sigma_F = (2 * T * Y_F * Y_S * Y_epsilon) / (width * m**2 * z)
    
    return sigma_F, Y_F


def select_material_by_strength(m1, m2, z1, z2, z3, z4, width1, width2, 
                                 torque1, torque2, material_db):
    """
    根据扭矩和齿轮参数自动选择合适材质
    
    返回:
        selected_key: 选中的材质key
        check_results: 各材质的校核结果
    """
    check_results = {}
    suitable_materials = []
    
    for key, props in material_db.items():
        # 计算第一级弯曲应力
        sigma_F1, YF1 = calculate_bending_stress(m1, z1, width1, torque1)
        sigma_F2, YF2 = calculate_bending_stress(m1, z2, width1, torque1 * z2 / z1)
        
        # 计算第二级弯曲应力
        sigma_F3, YF3 = calculate_bending_stress(m2, z3, width2, torque2)
        sigma_F4, YF4 = calculate_bending_stress(m2, z4, width2, torque2 * z4 / z3)
        
        # 最大弯曲应力
        max_sigma_F = max(sigma_F1, sigma_F2, sigma_F3, sigma_F4)
        
        # 许用弯曲应力
        sigma_F_allow = props['sigma_f'] / props['safety_factor']
        
        # 齿根厚检查
        alpha = 20 * math.pi / 180
        sf_factor = math.pi / 2 * math.cos(alpha) - 2 * 1.25 * math.tan(alpha)
        sf1 = m1 * sf_factor
        sf2 = m2 * sf_factor
        sf_ratio_ok = (sf1 / m1 >= props['sf_min_ratio']) and (sf2 / m2 >= props['sf_min_ratio'])
        
        # 强度检查
        strength_ok = max_sigma_F <= sigma_F_allow
        
        # 综合检查
        is_suitable = strength_ok and sf_ratio_ok
        
        check_results[key] = {
            'name': props['name'],
            'max_stress': max_sigma_F,
            'allow_stress': sigma_F_allow,
            'strength_ok': strength_ok,
            'sf_ratio_ok': sf_ratio_ok,
            'suitable': is_suitable,
            'safety_margin': sigma_F_allow / max_sigma_F if max_sigma_F > 0 else float('inf'),
            'density': props['density'],
            'color': props['color']
        }
        
        if is_suitable:
            suitable_materials.append((key, check_results[key]))
    
    # 选择最合适的材质 (优先顺序: PEEK > POM > 钢 > 尼龙, 基于重量和强度)
    if suitable_materials:
        # 按密度从小到大排序 (优先轻量材料)
        suitable_materials.sort(key=lambda x: x[1]['density'])
        selected_key = suitable_materials[0][0]
    else:
        # 如果没有合适的,选择强度最高的钢
        selected_key = 'steel'
    
    return selected_key, check_results


@dataclass
class DesignResult:
    """设计结果数据类"""
    m1: float = 0
    m2: float = 0
    z1: int = 0
    z2: int = 0
    z3: int = 0
    z4: int = 0
    D1: float = 0
    D2: float = 0
    D3: float = 0
    D4: float = 0
    width1: float = 0
    width2: float = 0
    wheelbase1: float = 0
    wheelbase2: float = 0
    i1: float = 0
    i2: float = 0
    Total_Ratio: float = 0
    output_speed: float = 0
    desired_motor_speed: float = 0
    output_torque: float = 0
    gear_weight_g: float = 0
    total_weight_g: float = 0
    total_size: float = 0  # 总纵向尺寸 (mm)
    # 材料相关字段
    material_key: str = ''
    material_name: str = ''
    material_density: float = 0
    bending_stress_1: float = 0
    bending_stress_2: float = 0
    bending_stress_3: float = 0
    bending_stress_4: float = 0
    max_bending_stress: float = 0
    allow_bending_stress: float = 0
    strength_check_pass: bool = False
    material_check_results: dict = None


# ==================== 优化计算 ====================
def optimize_gear_design():
    """齿轮优化设计主函数"""
    
    best_metric = float('inf')  # 根据优化模式，可以是重量或尺寸
    best_design: Optional[DesignResult] = None
    
    # 优化模式显示
    mode_display = "重量最轻" if OPTIMIZATION_MODE == 'weight' else "纵向尺寸最短"
    mode_color = Color.BRIGHT_GREEN if OPTIMIZATION_MODE == 'weight' else Color.BRIGHT_MAGENTA
    
    print(colorize('╔══════════════════════════════════════════════════════════╗', Color.BRIGHT_CYAN))
    print(colorize('║                  开始齿轮优化计算...                     ║', Color.BRIGHT_CYAN + Color.BOLD))
    print(colorize('╚══════════════════════════════════════════════════════════╝', Color.BRIGHT_CYAN))
    print(f"{colorize('优化模式:', Color.BRIGHT_YELLOW)} {colorize(mode_display, mode_color + Color.BOLD)}")
    print(f"{colorize('扫描范围:', Color.BRIGHT_YELLOW)} D1[{D1_min}-{D1_max}], D2[{D2_min}-{D2_max}], D3[{D3_min}-{D3_max}], D4[{D4_min}-{D4_max}]")
    if OPTIMIZE_WIDTH_FACTOR:
        print(f"{colorize('齿宽系数范围:', Color.BRIGHT_YELLOW)} 第一级[{width_factor1_min}-{width_factor1_max}], 第二级[{width_factor2_min}-{width_factor2_max}] (优化轴向厚度)")
    else:
        print(f"{colorize('齿宽系数:', Color.BRIGHT_YELLOW)} 第一级={width_factor1_fixed}, 第二级={width_factor2_fixed} (固定轴向厚度)")
    print(f"{colorize('约束条件:', Color.BRIGHT_YELLOW)} 输出转速={colorize(str(output_speed), Color.BRIGHT_GREEN)} rpm (固定), 输出扭矩>={colorize(str(output_torque), Color.BRIGHT_GREEN)} Nm, 电机转速<={colorize(str(max_motor_speed), Color.BRIGHT_GREEN)} rpm\n")
    
    # 齿宽系数选项
    if OPTIMIZE_WIDTH_FACTOR:
        width_factor1_options = range(width_factor1_min, width_factor1_max + 1, width_factor_step)
        width_factor2_options = range(width_factor2_min, width_factor2_max + 1, width_factor_step)
    else:
        width_factor1_options = [width_factor1_fixed]
        width_factor2_options = [width_factor2_fixed]
    
    # 嵌套循环优化搜索
    for m1 in m1_options:
        for m2 in m2_options:
            for width_factor1 in width_factor1_options:
                for width_factor2 in width_factor2_options:
                    for D1 in range(D1_min, D1_max + 1, step):
                        for D2 in range(D2_min, D2_max + 1, step):
                            for D3 in range(D3_min, D3_max + 1, step):
                                    for D4 in range(D4_min, D4_max + 1, step):
                                        
                                        # ----- 1. 计算并修正齿数 (齿数必须为整数) -----
                                        z1 = round(D1 / m1)
                                        z2 = round(D2 / m1)
                                        z3 = round(D3 / m2)
                                        z4 = round(D4 / m2)
                                        
                                        # 齿数约束：避免根切，最少齿数通常为17(标准齿轮)
                                        if z1 < 17 or z2 < 17 or z3 < 17 or z4 < 17:
                                            continue
                                        
                                        # ----- 2. 更新实际分度圆直径 -----
                                        D1_real = z1 * m1
                                        D2_real = z2 * m1
                                        D3_real = z3 * m2
                                        D4_real = z4 * m2
                                        
                                        # ----- 3. 计算传动比 -----
                                        i1 = z2 / z1
                                        i2 = z4 / z3
                                        Total_Ratio = i1 * i2
                                        
                                        # ----- 4. 计算轴距参数 -----
                                        wheelbase1 = (D1_real + D2_real) / 2  # 第一级中心距
                                        wheelbase2 = (D3_real + D4_real) / 2  # 第二级中心距
                                        
                                        # 轴距约束：确保物理上可装配
                                        if wheelbase1 <= 0 or wheelbase2 <= 0:
                                            continue
                                        
                                        # ----- 5. 计算齿宽(轴向厚度) -----
                                        width1 = width_factor1 * m1  # 第一级齿轮轴向厚度
                                        width2 = width_factor2 * m2  # 第二级齿轮轴向厚度
                                        
                                        # ----- 6. 性能计算 -----
                                        
                                        # 固定输出转速，反推所需电机转速
                                        actual_output_speed = output_speed  # 固定输出转速
                                        desired_motor_speed = actual_output_speed * Total_Ratio  # rpm
                                        
                                        # 电机转速约束：不能超过最大允许转速
                                        if desired_motor_speed > max_motor_speed:
                                            continue
                                        
                                        # 扭矩计算 (假设电机额定扭矩取下限值)
                                        motor_torque = motor_torque_lower
                                        actual_output_torque = motor_torque * Total_Ratio * 0.95 * 0.95  # 考虑两级传动效率各95%
                                        
                                        # ----- 7. 约束检查 -----
                                        # 输出扭矩约束
                                        if actual_output_torque < output_torque:
                                            continue
                                        
                                        # 包络尺寸约束：从动轮直径不超过电机直径的1倍
                                        if max(D2_real, D4_real) > motor_D:
                                            continue
                                        
                                        # 约束：一级主动轮(D1)要大于二级主动轮(D3)至少1mm间隙
                                        # 避免干涉，确保二级主动轮可以安装在中间轴上不与一级主动轮冲突
                                        if (D1_real - D3_real) < 1:  # 当间隙小于1mm时跳过
                                            continue

                                        
                                        # ----- 8. 重量计算 (简化为圆柱体模型) -----
                                        # 在auto模式下，先使用钢的密度估算，后续会更新为实际材质密度
                                        if MATERIAL_MODE == 'auto':
                                            temp_density = MATERIAL_DB['steel']['density']
                                        else:
                                            temp_density = MATERIAL_DB[MATERIAL_MODE]['density']
                                        
                                        # 齿轮体积计算 (分度圆直径 × 齿宽近似)
                                        vol1 = math.pi * (D1_real / 2) ** 2 * width1
                                        vol2 = math.pi * (D2_real / 2) ** 2 * width1
                                        vol3 = math.pi * (D3_real / 2) ** 2 * width2
                                        vol4 = math.pi * (D4_real / 2) ** 2 * width2
                                        
                                        # 修正：四个齿轮体积之和 × 密度，表示挖空体积占一半
                                        Total_Weight_kg = 0.5 * (vol1 + vol2 + vol3 + vol4) * temp_density
                                        Total_Weight_g = Total_Weight_kg * 1000
                                        
                                        # 总重量包含电机
                                        Total_System_Weight = Total_Weight_g + motor_weight
                                        
                                        # ----- 9. 纵向尺寸计算 -----
                                        # 总尺寸 = 电机半径 + 一级主动轮半径 + 一级从动轮半径 + 二级主动轮半径 + 二级从动轮半径
                                        # 简化：电机半径 + D1/2 + D2/2 + D3/2 + D4/2
                                        total_size = motor_D / 2 + D1_real / 2 + D2_real / 2 + D3_real / 2 + D4_real / 2
                                        
                                        # ----- 10. 材料强度校核 -----
                                        # 计算各级扭矩 (N·mm)
                                        T1 = motor_torque * 1000  # 第一级输入扭矩
                                        T2 = T1 * i1 * 0.95       # 第二级输入扭矩 (考虑一级效率)
                                        
                                        # 选择材质
                                        if MATERIAL_MODE == 'auto':
                                            selected_mat, mat_check_results = select_material_by_strength(
                                                m1, m2, z1, z2, z3, z4, width1, width2,
                                                T1, T2, MATERIAL_DB
                                            )
                                        else:
                                            selected_mat = MATERIAL_MODE
                                            # 仅校核指定材质
                                            _, mat_check_results = select_material_by_strength(
                                                m1, m2, z1, z2, z3, z4, width1, width2,
                                                T1, T2, {MATERIAL_MODE: MATERIAL_DB[MATERIAL_MODE]}
                                            )
                                        
                                        # 检查选定材质是否满足要求
                                        if selected_mat in mat_check_results:
                                            mat_result = mat_check_results[selected_mat]
                                            strength_pass = mat_result['suitable']
                                            
                                            # 如果不满足强度要求且是auto模式,尝试选择其他材质
                                            if not strength_pass and MATERIAL_MODE == 'auto':
                                                # 查找第一个满足要求的材质
                                                for key, result in mat_check_results.items():
                                                    if result['suitable']:
                                                        selected_mat = key
                                                        mat_result = result
                                                        strength_pass = True
                                                        break
                                            
                                            # 强度约束检查 (非auto模式下如果不满足则跳过)
                                            if not strength_pass and MATERIAL_MODE != 'auto':
                                                continue
                                        
                                        # ----- 11. 记录最优解 -----
                                        # 根据优化模式选择判断条件
                                        if OPTIMIZATION_MODE == 'weight':
                                            # 重量优化模式：选择重量最轻的
                                            current_metric = Total_Weight_kg
                                        else:
                                            # 尺寸优化模式：选择尺寸最小的
                                            current_metric = total_size
                                        
                                        if current_metric < best_metric:
                                            best_metric = current_metric
                                            
                                            # 获取选定材质的详细信息
                                            mat_props = MATERIAL_DB[selected_mat]
                                            
                                            # 使用实际材质密度重新计算重量
                                            actual_density = mat_props['density']
                                            Total_Weight_kg_actual = 0.5 * (vol1 + vol2 + vol3 + vol4) * actual_density
                                            Total_Weight_g_actual = Total_Weight_kg_actual * 1000
                                            Total_System_Weight_actual = Total_Weight_g_actual + motor_weight
                                            
                                            best_design = DesignResult(
                                                m1=m1,
                                                m2=m2,
                                                z1=z1,
                                                z2=z2,
                                                z3=z3,
                                                z4=z4,
                                                D1=D1_real,
                                                D2=D2_real,
                                                D3=D3_real,
                                                D4=D4_real,
                                                width1=width1,
                                                width2=width2,
                                                wheelbase1=wheelbase1,
                                                wheelbase2=wheelbase2,
                                                i1=i1,
                                                i2=i2,
                                                Total_Ratio=Total_Ratio,
                                                output_speed=actual_output_speed,
                                                desired_motor_speed=desired_motor_speed,
                                                output_torque=actual_output_torque,
                                                gear_weight_g=Total_Weight_g_actual,
                                                total_weight_g=Total_System_Weight_actual,
                                                total_size=total_size,
                                                material_key=selected_mat,
                                                material_name=mat_props['name'],
                                                material_density=mat_props['density'],
                                                max_bending_stress=mat_result.get('max_stress', 0),
                                                allow_bending_stress=mat_result.get('allow_stress', 0),
                                                strength_check_pass=mat_result.get('suitable', False),
                                                material_check_results=mat_check_results
                                            )
    
    return best_design, best_metric


# ==================== 输出结果 ====================
def print_results(best_design: Optional[DesignResult], best_metric: float):
    """打印优化结果（带颜色）"""
    
    print()
    print(colorize('╔══════════════════════════════════════════════════════════╗', Color.BRIGHT_CYAN))
    print(colorize('║            二级齿轮传动优化设计报告                      ║', Color.BRIGHT_CYAN + Color.BOLD))
    print(colorize('╚══════════════════════════════════════════════════════════╝', Color.BRIGHT_CYAN))
    print()
    
    if best_metric == float('inf') or best_design is None:
        print(colorize('【警告】未找到满足所有约束条件的解！请放宽约束或扩大搜索范围。', Color.BRIGHT_RED + Color.BOLD))
    else:
        # 根据优化模式显示不同的优化目标
        if OPTIMIZATION_MODE == 'weight':
            opt_target = "重量最轻设计"
            opt_color = Color.BRIGHT_GREEN
        else:
            opt_target = "纵向尺寸最短设计"
            opt_color = Color.BRIGHT_MAGENTA
        
        # 打印最优设计参数
        print(f"{colorize('▶', Color.BRIGHT_GREEN)} {colorize('【优化目标】', Color.BRIGHT_YELLOW)}{colorize(opt_target, opt_color + Color.BOLD)}")
        
        # 材质信息
        mat_color = Color.BRIGHT_GREEN if best_design.strength_check_pass else Color.BRIGHT_RED
        print(f"{colorize('▶', Color.BRIGHT_GREEN)} {colorize('【选定材质】', Color.BRIGHT_YELLOW)}{colorize(best_design.material_name, mat_color + Color.BOLD)} {colorize(f'(密度: {best_design.material_density:.2e} kg/mm³)', Color.BRIGHT_CYAN)}")
        
        # 材质校核结果汇总
        if best_design.material_check_results:
            print(f"{colorize('▶', Color.BRIGHT_GREEN)} {colorize('【材质校核】', Color.BRIGHT_YELLOW)}")
            for key, result in best_design.material_check_results.items():
                status_color = Color.BRIGHT_GREEN if result['suitable'] else Color.BRIGHT_RED
                status = colorize('✓ 适用', status_color) if result['suitable'] else colorize('✗ 不适用', status_color)
                print(f"   {result['name']}: {status} (应力: {result['max_stress']:.1f}/{result['allow_stress']:.1f} MPa, 安全裕度: {result['safety_margin']:.2f})")
        print()
        print()
        
        # 计算齿距和齿根相关参数
        p1 = math.pi * best_design.m1  # 第一级齿距
        p2 = math.pi * best_design.m2  # 第二级齿距
        ha1 = best_design.m1  # 第一级齿顶高
        ha2 = best_design.m2  # 第二级齿顶高
        hf1 = 1.25 * best_design.m1  # 第一级齿根高(标准齿轮)
        hf2 = 1.25 * best_design.m2  # 第二级齿根高(标准齿轮)
        h1 = ha1 + hf1  # 第一级总齿高 = 2.25*m1
        h2 = ha2 + hf2  # 第二级总齿高 = 2.25*m2
        
        # 齿根圆直径
        df1 = best_design.D1 - 2 * hf1  # 第一级主动轮齿根圆直径
        df2 = best_design.D2 - 2 * hf1  # 第一级从动轮齿根圆直径
        df3 = best_design.D3 - 2 * hf2  # 第二级主动轮齿根圆直径
        df4 = best_design.D4 - 2 * hf2  # 第二级从动轮齿根圆直径
        
        # 计算危险截面齿根宽(用于弯曲强度校核)
        alpha_deg = gear_pressure_angle  # 压力角 20度
        alpha = alpha_deg * math.pi / 180
        sf_factor = math.pi / 2 * math.cos(alpha) - 2 * 1.25 * math.tan(alpha)
        
        # 齿根宽(危险截面处)
        sf1 = best_design.m1 * sf_factor
        sf2 = best_design.m2 * sf_factor
        
        # 齿根厚与模数比
        sf_m_ratio1 = sf1 / best_design.m1
        sf_m_ratio2 = sf2 / best_design.m2
        
        # 计算齿根圆处齿厚(用于强度校核)
        def inv(x):
            """渐开线函数 inv(α) = tan(α) - α (α为弧度)"""
            return math.tan(x) - x
        
        def tooth_thickness_at_root(d, df, z, m, alpha_rad):
            """计算齿根圆处齿厚
            d: 分度圆直径
            df: 齿根圆直径  
            z: 齿数
            m: 模数
            alpha_rad: 压力角(弧度)
            """
            if df <= 0:
                return 0
            # 分度圆齿厚 (标准齿轮)
            s = math.pi * m / 2
            # 齿根圆压力角
            cos_alpha_f = (d * math.cos(alpha_rad)) / df
            if cos_alpha_f >= 1 or cos_alpha_f <= -1:
                return 0
            alpha_f = math.acos(cos_alpha_f)
            # 齿根圆齿厚
            s_f = df * (s / d + inv(alpha_rad) - inv(alpha_f))
            return max(0, s_f)
        
        # 第一级齿根圆齿厚
        sf_tooth_1 = tooth_thickness_at_root(best_design.D1, df1, best_design.z1, best_design.m1, alpha)
        sf_tooth_2 = tooth_thickness_at_root(best_design.D2, df2, best_design.z2, best_design.m1, alpha)
        # 第二级齿根圆齿厚
        sf_tooth_3 = tooth_thickness_at_root(best_design.D3, df3, best_design.z3, best_design.m2, alpha)
        sf_tooth_4 = tooth_thickness_at_root(best_design.D4, df4, best_design.z4, best_design.m2, alpha)
        
        # 齿轮参数
        print(colorize('┌────────────────── 齿轮参数 ──────────────────┐', Color.BRIGHT_BLUE))
        print(f"{colorize('│ 第一级 (高速级):', Color.BRIGHT_MAGENTA)}")
        print(f"│   模数 m1 = {colorize(f'{best_design.m1:.2f}', Color.BRIGHT_GREEN)} mm, 齿距 p1 = {colorize(f'{p1:.3f}', Color.BRIGHT_CYAN)} mm")
        print(f"│   齿顶高 = {colorize(f'{ha1:.2f}', Color.BRIGHT_CYAN)} mm, 齿根高 = {colorize(f'{hf1:.2f}', Color.BRIGHT_CYAN)} mm, 总齿高 = {colorize(f'{h1:.2f}', Color.BRIGHT_CYAN)} mm")
        print(f"│   齿宽系数 = {colorize(f'{best_design.width1/best_design.m1:.0f}', Color.BRIGHT_YELLOW)}, 轴向厚度(齿宽) = {colorize(f'{best_design.width1:.2f}', Color.BRIGHT_GREEN)} mm")
        print(f"│   危险截面齿根宽 sf1 = {colorize(f'{sf1:.3f}', Color.BRIGHT_YELLOW)} mm (sf/m = {colorize(f'{sf_m_ratio1:.3f}', Color.BRIGHT_YELLOW)})")
        print(f"│   主动轮: 齿数 z1 = {colorize(f'{best_design.z1}', Color.BRIGHT_CYAN)}, 分度圆直径 D1 = {colorize(f'{best_design.D1:.2f}', Color.BRIGHT_CYAN)} mm")
        print(f"│           齿根圆直径 = {colorize(f'{df1:.2f}', Color.BRIGHT_CYAN)} mm, 齿根圆处齿厚 = {colorize(f'{sf_tooth_1:.3f}', Color.BRIGHT_GREEN)} mm")
        print(f"│   从动轮: 齿数 z2 = {colorize(f'{best_design.z2}', Color.BRIGHT_CYAN)}, 分度圆直径 D2 = {colorize(f'{best_design.D2:.2f}', Color.BRIGHT_CYAN)} mm")
        print(f"│           齿根圆直径 = {colorize(f'{df2:.2f}', Color.BRIGHT_CYAN)} mm, 齿根圆处齿厚 = {colorize(f'{sf_tooth_2:.3f}', Color.BRIGHT_GREEN)} mm")
        print(f"│   传动比 i1 = {colorize(f'{best_design.i1:.3f}', Color.BRIGHT_GREEN)}")
        print(f"{colorize('│', Color.BRIGHT_BLUE)}")
        print(f"{colorize('│ 第二级 (低速级):', Color.BRIGHT_MAGENTA)}")
        print(f"│   模数 m2 = {colorize(f'{best_design.m2:.2f}', Color.BRIGHT_GREEN)} mm, 齿距 p2 = {colorize(f'{p2:.3f}', Color.BRIGHT_CYAN)} mm")
        print(f"│   齿顶高 = {colorize(f'{ha2:.2f}', Color.BRIGHT_CYAN)} mm, 齿根高 = {colorize(f'{hf2:.2f}', Color.BRIGHT_CYAN)} mm, 总齿高 = {colorize(f'{h2:.2f}', Color.BRIGHT_CYAN)} mm")
        print(f"│   齿宽系数 = {colorize(f'{best_design.width2/best_design.m2:.0f}', Color.BRIGHT_YELLOW)}, 轴向厚度(齿宽) = {colorize(f'{best_design.width2:.2f}', Color.BRIGHT_GREEN)} mm")
        print(f"│   危险截面齿根宽 sf2 = {colorize(f'{sf2:.3f}', Color.BRIGHT_YELLOW)} mm (sf/m = {colorize(f'{sf_m_ratio2:.3f}', Color.BRIGHT_YELLOW)})")
        print(f"│   主动轮: 齿数 z3 = {colorize(f'{best_design.z3}', Color.BRIGHT_CYAN)}, 分度圆直径 D3 = {colorize(f'{best_design.D3:.2f}', Color.BRIGHT_CYAN)} mm")
        print(f"│           齿根圆直径 = {colorize(f'{df3:.2f}', Color.BRIGHT_CYAN)} mm, 齿根圆处齿厚 = {colorize(f'{sf_tooth_3:.3f}', Color.BRIGHT_GREEN)} mm")
        print(f"│   从动轮: 齿数 z4 = {colorize(f'{best_design.z4}', Color.BRIGHT_CYAN)}, 分度圆直径 D4 = {colorize(f'{best_design.D4:.2f}', Color.BRIGHT_CYAN)} mm")
        print(f"│           齿根圆直径 = {colorize(f'{df4:.2f}', Color.BRIGHT_CYAN)} mm, 齿根圆处齿厚 = {colorize(f'{sf_tooth_4:.3f}', Color.BRIGHT_GREEN)} mm")
        print(f"│   传动比 i2 = {colorize(f'{best_design.i2:.3f}', Color.BRIGHT_GREEN)}")
        print(colorize('└──────────────────────────────────────────────┘', Color.BRIGHT_BLUE))
        print()
        
        # 总体性能
        print(colorize('┌────────────────── 总体性能 ──────────────────┐', Color.BRIGHT_BLUE))
        print(f"│ 总减速比: {colorize(f'{best_design.Total_Ratio:.3f}', Color.BRIGHT_GREEN + Color.BOLD)}")
        print(f"│ 实际齿数配比: ({colorize(f'{best_design.z1}:{best_design.z2}', Color.BRIGHT_CYAN)}) × ({colorize(f'{best_design.z3}:{best_design.z4}', Color.BRIGHT_CYAN)})")
        print(f"│ 输出转速: {colorize(f'{best_design.output_speed:.2f}', Color.BRIGHT_CYAN)} rpm (固定目标值)")
        
        satisfy_speed = '满足' if best_design.desired_motor_speed <= max_motor_speed else '不满足'
        speed_color = Color.BRIGHT_GREEN if satisfy_speed == '满足' else Color.BRIGHT_RED
        print(f"│ 期望电机转速: {colorize(f'{best_design.desired_motor_speed:.2f}', Color.BRIGHT_CYAN)} rpm (约束: <= {max_motor_speed} rpm) 【{colorize(satisfy_speed, speed_color + Color.BOLD)}】")
        
        satisfy_torque = '满足' if best_design.output_torque >= output_torque else '不满足'
        torque_color = Color.BRIGHT_GREEN if satisfy_torque == '满足' else Color.BRIGHT_RED
        print(f"│ 输出扭矩: {colorize(f'{best_design.output_torque:.3f}', Color.BRIGHT_CYAN)} Nm (要求 >= {output_torque} Nm) 【{colorize(satisfy_torque, torque_color + Color.BOLD)}】")
        print(colorize('└──────────────────────────────────────────────┘', Color.BRIGHT_BLUE))
        print()
        
        # 尺寸与重量
        print(colorize('┌────────────────── 尺寸与重量 ────────────────┐', Color.BRIGHT_BLUE))
        print(f"{colorize('│ 轴距参数:', Color.BRIGHT_MAGENTA)}")
        print(f"│   第一级中心距 (电机轴-中间轴): {colorize(f'{best_design.wheelbase1:.2f}', Color.BRIGHT_CYAN)} mm")
        print(f"│   第二级中心距 (中间轴-输出轴): {colorize(f'{best_design.wheelbase2:.2f}', Color.BRIGHT_CYAN)} mm")
        print(f"│ 齿轮轴向总厚度: {colorize(f'{best_design.width1 + best_design.width2:.2f}', Color.BRIGHT_CYAN)} mm (两级齿轮厚度之和)")
        print(f"│ 总体轴向尺寸: {colorize(f'{best_design.width1 + best_design.width2 + 15:.2f}', Color.BRIGHT_CYAN)} mm (齿轮厚度 + 轴承间隙15mm)")
        print(f"│ 最大径向尺寸: {colorize(f'{max(best_design.D2, best_design.D4):.2f}', Color.BRIGHT_CYAN)} mm (电机直径参考: {motor_D:.1f} mm)")
        print(f"{colorize('│', Color.BRIGHT_BLUE)}")
        print(f"{colorize('│ 纵向尺寸组成:', Color.BRIGHT_MAGENTA)}")
        print(f"│   电机半径: {colorize(f'{motor_D/2:.2f}', Color.BRIGHT_CYAN)} mm")
        print(f"│   D1/2 (一级主动轮半径): {colorize(f'{best_design.D1/2:.2f}', Color.BRIGHT_CYAN)} mm")
        print(f"│   D2/2 (一级从动轮半径): {colorize(f'{best_design.D2/2:.2f}', Color.BRIGHT_CYAN)} mm")
        print(f"│   D3/2 (二级主动轮半径): {colorize(f'{best_design.D3/2:.2f}', Color.BRIGHT_CYAN)} mm")
        print(f"│   D4/2 (二级从动轮半径): {colorize(f'{best_design.D4/2:.2f}', Color.BRIGHT_CYAN)} mm")
        # 根据优化模式高亮显示主要优化指标
        if OPTIMIZATION_MODE == 'weight':
            print(f"│ {colorize('总纵向尺寸:', Color.BRIGHT_YELLOW)} {colorize(f'{best_design.total_size:.2f}', Color.BRIGHT_CYAN)} mm")
            print(f"{colorize('│', Color.BRIGHT_BLUE)}")
            print(f"│ 齿轮组重量: {colorize(f'{best_design.gear_weight_g:.3f}', Color.BRIGHT_YELLOW)} g")
            print(f"│ 电机重量: {colorize(f'{motor_weight}', Color.BRIGHT_YELLOW)} g")
            print(f"│ {colorize('系统总重量:', Color.BRIGHT_YELLOW)} {colorize(f'{best_design.total_weight_g:.3f}', Color.BRIGHT_GREEN + Color.BOLD)} g ({colorize(f'{best_design.total_weight_g/1000:.3f}', Color.BRIGHT_GREEN)} kg) {colorize('← 优化指标', Color.BRIGHT_GREEN)}")
        else:
            print(f"│ {colorize('总纵向尺寸:', Color.BRIGHT_YELLOW)} {colorize(f'{best_design.total_size:.2f}', Color.BRIGHT_MAGENTA + Color.BOLD)} mm {colorize('← 优化指标', Color.BRIGHT_MAGENTA)}")
            print(f"{colorize('│', Color.BRIGHT_BLUE)}")
            print(f"│ 齿轮组重量: {colorize(f'{best_design.gear_weight_g:.3f}', Color.BRIGHT_YELLOW)} g")
            print(f"│ 电机重量: {colorize(f'{motor_weight}', Color.BRIGHT_YELLOW)} g")
            print(f"│ 系统总重量: {colorize(f'{best_design.total_weight_g:.3f}', Color.BRIGHT_GREEN)} g ({colorize(f'{best_design.total_weight_g/1000:.3f}', Color.BRIGHT_GREEN)} kg)")
        print(colorize('└──────────────────────────────────────────────┘', Color.BRIGHT_BLUE))
        print()
        
        # 材料强度校核详细信息
        print(colorize('┌────────────────── 材料强度校核 ──────────────┐', Color.BRIGHT_BLUE))
        print(f"{colorize('│ 选定材质:', Color.BRIGHT_MAGENTA)} {colorize(best_design.material_name, Color.BRIGHT_GREEN)}")
        print(f"│ 材质密度: {colorize(f'{best_design.material_density:.2e}', Color.BRIGHT_CYAN)} kg/mm³")
        print(f"│ 最大弯曲应力: {colorize(f'{best_design.max_bending_stress:.1f}', Color.BRIGHT_YELLOW)} MPa")
        print(f"│ 许用弯曲应力: {colorize(f'{best_design.allow_bending_stress:.1f}', Color.BRIGHT_GREEN)} MPa")
        stress_ratio = best_design.max_bending_stress / best_design.allow_bending_stress if best_design.allow_bending_stress > 0 else 0
        stress_status = "✓ 安全" if stress_ratio < 1.0 else "✗ 超限"
        stress_color = Color.BRIGHT_GREEN if stress_ratio < 1.0 else Color.BRIGHT_RED
        print(f"│ 应力比: {colorize(f'{stress_ratio:.2f}', Color.BRIGHT_CYAN)} {colorize(stress_status, stress_color)}")
        
        # 各材质校核对比
        if best_design.material_check_results:
            print(f"{colorize('│', Color.BRIGHT_BLUE)}")
            print(f"{colorize('│ 各材质校核对比:', Color.BRIGHT_MAGENTA)}")
            for key, result in best_design.material_check_results.items():
                marker = "►" if key == best_design.material_key else " "
                status = colorize('✓', Color.BRIGHT_GREEN) if result['suitable'] else colorize('✗', Color.BRIGHT_RED)
                weight_est = best_design.gear_weight_g * (result['density'] / best_design.material_density)
                print(f"│ {marker} {status} {result['name']}: 应力 {result['max_stress']:.1f}/{result['allow_stress']:.1f} MPa, 预估重量 {weight_est:.1f}g")
        print(colorize('└──────────────────────────────────────────────┘', Color.BRIGHT_BLUE))
        print()
        
        # 设计验证
        print(colorize('┌────────────────── 设计验证 ──────────────────┐', Color.BRIGHT_BLUE))
        
        # 验证轴距约束
        wb1_check = abs(best_design.wheelbase1 - (best_design.D1 + best_design.D2) / 2) < 0.01
        wb2_check = abs(best_design.wheelbase2 - (best_design.D3 + best_design.D4) / 2) < 0.01
        wb_result = iif(wb1_check and wb2_check, "通过", "失败")
        wb_color = Color.BRIGHT_GREEN if wb_result == "通过" else Color.BRIGHT_RED
        print(f"│ 轴距约束验证: {colorize(wb_result, wb_color + Color.BOLD)}")
        
        # 齿轮间隙验证
        clearance_check = (best_design.D1 - best_design.D3) >= 1
        clearance_value = best_design.D1 - best_design.D3
        clearance_result = iif(clearance_check, "通过", "失败")
        clearance_color = Color.BRIGHT_GREEN if clearance_result == "通过" else Color.BRIGHT_RED
        print(f"│ 齿轮间隙验证 (D1-D3 >= 1mm): {colorize(f'{clearance_value:.2f}', Color.BRIGHT_CYAN)} mm 【{colorize(clearance_result, clearance_color + Color.BOLD)}】")
        
        # 强度粗略估算 (简化)
        strength_factor = (best_design.m2 * best_design.width2) / (best_design.m1 * best_design.width1)
        strength_ok = strength_factor > 1.0
        strength_status = iif(strength_ok, "✓ 符合", "✗ 注意")
        strength_color = Color.BRIGHT_GREEN if strength_ok else Color.BRIGHT_YELLOW
        print(f"│ 低速级相对强度系数: {colorize(f'{strength_factor:.2f}', Color.BRIGHT_CYAN)} {colorize(strength_status, strength_color)} (>1.0表示低速级更强)")
        print(colorize('└──────────────────────────────────────────────┘', Color.BRIGHT_BLUE))
    
    print()
    print(colorize('╔══════════════════════════════════════════════════════════╗', Color.BRIGHT_GREEN))
    print(colorize('║                      设计完成                            ║', Color.BRIGHT_GREEN + Color.BOLD))
    print(colorize('╚══════════════════════════════════════════════════════════╝', Color.BRIGHT_GREEN))


# ==================== 主程序入口 ====================
if __name__ == '__main__':
    best_design, best_metric = optimize_gear_design()
    print_results(best_design, best_metric)
