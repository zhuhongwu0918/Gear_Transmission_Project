# 模拟主程序逻辑的调试脚本
import math
from typing import Optional, Tuple, Dict

# 参数配置 (与主程序一致)
MOTOR_TORQUE_LOWER = 0.8
OUTPUT_TORQUE = 4
MAX_MOTOR_SPEED = 4000
OUTPUT_SPEED = 250
MOTOR_D = 75
MIN_CLEARANCE = 1
MIN_TEETH = 17

M1_OPTIONS = [0.3, 0.4, 0.5, 0.8, 1.0, 1.25, 1.5]
M2_OPTIONS = [0.5, 0.8, 1.0, 1.25, 1.5, 2.0, 2.5]

D1_MIN, D1_MAX = 15, 30
D2_MIN, D2_MAX = 50, 120
D3_MIN, D3_MAX = 10, 40
D4_MIN, D4_MAX = 30, 80
STEP = 5

WIDTH_FACTOR1 = 15
WIDTH_FACTOR2 = 10

MATERIAL_MODE = 'auto'

MATERIAL_DB = {
    'steel': {
        'name': '合金钢 (20CrMnTi)',
        'density': 7.85e-6,
        'sigma_f': 600,
        'sf_min_ratio': 0.3,
        'safety_factor': 2.0,
    },
    'peek': {
        'name': 'PEEK',
        'density': 1.32e-6,
        'sigma_f': 90,
        'sf_min_ratio': 0.8,
        'safety_factor': 2.5,
    },
    'pom': {
        'name': 'POM',
        'density': 1.41e-6,
        'sigma_f': 65,
        'sf_min_ratio': 1.0,
        'safety_factor': 3.0,
    },
    'nylon': {
        'name': '尼龙66',
        'density': 1.15e-6,
        'sigma_f': 70,
        'sf_min_ratio': 0.9,
        'safety_factor': 2.8,
    }
}

def calculate_bending_stress(m, z, width, torque):
    if z < 12 or width <= 0 or m <= 0:
        return float('inf'), 0
    Y_F = 2.1 + 3.5 / z
    Y_S = 1.0
    Y_epsilon = 0.7
    T = torque * 1000
    sigma_F = (2 * T * Y_F * Y_S * Y_epsilon) / (width * m**2 * z)
    return sigma_F, Y_F

def select_material_by_strength(m1, m2, z1, z2, z3, z4, width1, width2, torque1, torque2):
    check_results = {}
    suitable_materials = []
    
    for key, props in MATERIAL_DB.items():
        sigma_F1, _ = calculate_bending_stress(m1, z1, width1, torque1)
        sigma_F2, _ = calculate_bending_stress(m1, z2, width1, torque1 * z2 / z1)
        sigma_F3, _ = calculate_bending_stress(m2, z3, width2, torque2)
        sigma_F4, _ = calculate_bending_stress(m2, z4, width2, torque2 * z4 / z3)
        
        max_sigma_F = max(sigma_F1, sigma_F2, sigma_F3, sigma_F4)
        sigma_F_allow = props['sigma_f'] / props['safety_factor']
        
        alpha = 20 * math.pi / 180
        sf_factor = math.pi / 2 * math.cos(alpha) - 2 * 1.25 * math.tan(alpha)
        sf1 = m1 * sf_factor
        sf2 = m2 * sf_factor
        sf_ratio_ok = (sf1 / m1 >= props['sf_min_ratio']) and (sf2 / m2 >= props['sf_min_ratio'])
        
        strength_ok = max_sigma_F <= sigma_F_allow
        is_suitable = strength_ok and sf_ratio_ok
        
        check_results[key] = {
            'suitable': is_suitable,
            'max_stress': max_sigma_F,
            'allow_stress': sigma_F_allow,
            'strength_ok': strength_ok,
            'sf_ratio_ok': sf_ratio_ok,
        }
        
        if is_suitable:
            suitable_materials.append((key, check_results[key]))
    
    if suitable_materials:
        suitable_materials.sort(key=lambda x: MATERIAL_DB[x[0]]['density'])
        selected_key = suitable_materials[0][0]
    else:
        selected_key = 'steel'
    
    return selected_key, check_results

def evaluate_design(m1, m2, wf1, wf2, D1, D2, D3, D4) -> Optional[Dict]:
    """评估单个设计组合 (与主程序逻辑一致)"""
    
    # 1. 计算齿数
    z1 = round(D1 / m1)
    z2 = round(D2 / m1)
    z3 = round(D3 / m2)
    z4 = round(D4 / m2)
    
    # 2. 齿数约束
    if z1 < MIN_TEETH or z2 < MIN_TEETH or z3 < MIN_TEETH or z4 < MIN_TEETH:
        return None, 'teeth'
    
    # 3. 实际直径
    D1_real = z1 * m1
    D2_real = z2 * m1
    D3_real = z3 * m2
    D4_real = z4 * m2
    
    # 4. 传动比
    i1 = z2 / z1
    i2 = z4 / z3
    Total_Ratio = i1 * i2
    
    # 5. 轴距
    wheelbase1 = (D1_real + D2_real) / 2
    wheelbase2 = (D3_real + D4_real) / 2
    if wheelbase1 <= 0 or wheelbase2 <= 0:
        return None, 'wheelbase'
    
    # 6. 齿宽
    width1 = wf1 * m1
    width2 = wf2 * m2
    
    # 7. 转速约束
    desired_motor_speed = OUTPUT_SPEED * Total_Ratio
    if desired_motor_speed > MAX_MOTOR_SPEED:
        return None, 'motor_speed'
    
    # 8. 扭矩约束
    actual_output_torque = MOTOR_TORQUE_LOWER * Total_Ratio * 0.95 * 0.95
    if actual_output_torque < OUTPUT_TORQUE:
        return None, 'output_torque'
    
    # 9. 包络约束
    if max(D2_real, D4_real) > MOTOR_D:
        return None, 'envelope'
    
    # 10. 间隙约束
    if (D1_real - D3_real) < MIN_CLEARANCE:
        return None, 'clearance'
    
    # 11. 材料强度校核
    # 修复：select_material_by_strength 期望 N·m，内部会转为 N·mm
    T1_motor = MOTOR_TORQUE_LOWER  # N·m
    T2_intermediate = T1_motor * i1 * 0.95  # N·m
    
    selected_mat, mat_check_results = select_material_by_strength(
        m1, m2, z1, z2, z3, z4, width1, width2, T1_motor, T2_intermediate
    )
    
    if selected_mat not in mat_check_results:
        return None, 'material_not_found'
    
    mat_result = mat_check_results[selected_mat]
    strength_pass = mat_result['suitable']
    
    if not strength_pass and MATERIAL_MODE == 'auto':
        for key, result in mat_check_results.items():
            if result['suitable']:
                selected_mat = key
                mat_result = result
                strength_pass = True
                break
    
    if not strength_pass:
        return None, 'strength'
    
    return {
        'm1': m1, 'm2': m2, 'z1': z1, 'z2': z2, 'z3': z3, 'z4': z4,
        'D1': D1_real, 'D2': D2_real, 'D3': D3_real, 'D4': D4_real,
        'ratio': Total_Ratio,
        'material': selected_mat,
        'max_stress': mat_result['max_stress'],
        'allow_stress': mat_result['allow_stress'],
    }, 'ok'

# 统计各约束的失败次数
print("="*60)
print("主程序逻辑模拟 - 约束诊断")
print("="*60)

fail_counts = {
    'teeth': 0,
    'wheelbase': 0,
    'motor_speed': 0,
    'output_torque': 0,
    'envelope': 0,
    'clearance': 0,
    'material_not_found': 0,
    'strength': 0,
    'ok': 0,
}

valid_count = 0
best_ratio_diff = float('inf')
best_design = None

# 只扫描部分组合以加快速度
for m1 in [0.5, 0.8, 1.0]:
    for m2 in [0.8, 1.0, 1.5]:
        for D1 in range(D1_MIN, D1_MAX + 1, 5):
            for D2 in range(D2_MIN, D2_MAX + 1, 5):
                for D3 in range(D3_MIN, D3_MAX + 1, 5):
                    for D4 in range(D4_MIN, D4_MAX + 1, 5):
                        result, reason = evaluate_design(m1, m2, WIDTH_FACTOR1, WIDTH_FACTOR2, D1, D2, D3, D4)
                        fail_counts[reason] += 1
                        if result:
                            valid_count += 1
                            # 找最接近目标传动比的设计
                            target_ratio = OUTPUT_TORQUE / MOTOR_TORQUE_LOWER / 0.95 / 0.95
                            ratio_diff = abs(result['ratio'] - target_ratio)
                            if ratio_diff < best_ratio_diff:
                                best_ratio_diff = ratio_diff
                                best_design = result

# 收集强度失败的案例
strength_failures = []

def evaluate_design_debug(m1, m2, wf1, wf2, D1, D2, D3, D4):
    """评估单个设计组合并返回详细信息"""
    z1 = round(D1 / m1)
    z2 = round(D2 / m1)
    z3 = round(D3 / m2)
    z4 = round(D4 / m2)
    
    if z1 < MIN_TEETH or z2 < MIN_TEETH or z3 < MIN_TEETH or z4 < MIN_TEETH:
        return None, 'teeth', None
    
    D1_real = z1 * m1
    D2_real = z2 * m1
    D3_real = z3 * m2
    D4_real = z4 * m2
    
    i1 = z2 / z1
    i2 = z4 / z3
    Total_Ratio = i1 * i2
    
    wheelbase1 = (D1_real + D2_real) / 2
    wheelbase2 = (D3_real + D4_real) / 2
    if wheelbase1 <= 0 or wheelbase2 <= 0:
        return None, 'wheelbase', None
    
    width1 = wf1 * m1
    width2 = wf2 * m2
    
    desired_motor_speed = OUTPUT_SPEED * Total_Ratio
    if desired_motor_speed > MAX_MOTOR_SPEED:
        return None, 'motor_speed', None
    
    actual_output_torque = MOTOR_TORQUE_LOWER * Total_Ratio * 0.95 * 0.95
    if actual_output_torque < OUTPUT_TORQUE:
        return None, 'output_torque', None
    
    if max(D2_real, D4_real) > MOTOR_D:
        return None, 'envelope', None
    
    if (D1_real - D3_real) < MIN_CLEARANCE:
        return None, 'clearance', None
    
    # 修复：select_material_by_strength 期望 N·m，内部会转为 N·mm
    T1_motor = MOTOR_TORQUE_LOWER  # N·m
    T2_intermediate = T1_motor * i1 * 0.95  # N·m
    
    selected_mat, mat_check_results = select_material_by_strength(
        m1, m2, z1, z2, z3, z4, width1, width2, T1_motor, T2_intermediate
    )
    
    mat_result = mat_check_results[selected_mat]
    strength_pass = mat_result['suitable']
    
    if not strength_pass and MATERIAL_MODE == 'auto':
        for key, result in mat_check_results.items():
            if result['suitable']:
                strength_pass = True
                break
    
    if not strength_pass:
        # 收集强度失败信息
        debug_info = {
            'params': (m1, m2, D1, D2, D3, D4),
            'gears': (z1, z2, z3, z4),
            'diameters': (D1_real, D2_real, D3_real, D4_real),
            'ratio': Total_Ratio,
            'widths': (width1, width2),
            'torques': (T1, T2),
            'mat_results': mat_check_results,
        }
        return None, 'strength', debug_info
    
    return {'material': selected_mat}, 'ok', None

# 重新运行收集失败案例
for m1 in [0.5, 0.8, 1.0]:
    for m2 in [0.8, 1.0, 1.5]:
        for D1 in range(D1_MIN, D1_MAX + 1, 5):
            for D2 in range(D2_MIN, D2_MAX + 1, 5):
                for D3 in range(D3_MIN, D3_MAX + 1, 5):
                    for D4 in range(D4_MIN, D4_MAX + 1, 5):
                        result, reason, debug_info = evaluate_design_debug(
                            m1, m2, WIDTH_FACTOR1, WIDTH_FACTOR2, D1, D2, D3, D4
                        )
                        if reason == 'strength' and len(strength_failures) < 3:
                            strength_failures.append(debug_info)

print(f"\n【约束失败统计】(扫描步长=5mm)")
for reason, count in fail_counts.items():
    if reason != 'ok':
        print(f"  {reason}: {count}")
print(f"  通过所有约束: {fail_counts['ok']}")

print(f"\n【强度失败案例分析】(前3个)")
for i, info in enumerate(strength_failures, 1):
    m1, m2, D1, D2, D3, D4 = info['params']
    z1, z2, z3, z4 = info['gears']
    D1r, D2r, D3r, D4r = info['diameters']
    width1, width2 = info['widths']
    T1, T2 = info['torques']
    
    print(f"\n案例{i}:")
    print(f"  参数: m1={m1}, m2={m2}, D1={D1}, D2={D2}, D3={D3}, D4={D4}")
    print(f"  齿数: z1={z1}, z2={z2}, z3={z3}, z4={z4}")
    print(f"  实际直径: D1={D1r}, D2={D2r}, D3={D3r}, D4={D4r}")
    print(f"  齿宽: width1={width1:.2f}, width2={width2:.2f}")
    print(f"  传动比: {info['ratio']:.2f}")
    print(f"  扭矩: T1={T1:.0f} N·mm, T2={T2:.0f} N·mm")
    print(f"  材料检查结果:")
    for mat, res in info['mat_results'].items():
        status = "✓" if res['suitable'] else "✗"
        print(f"    {status} {mat}: 应力={res['max_stress']:.1f}/{res['allow_stress']:.1f} MPa, "
              f"强度={'✓' if res['strength_ok'] else '✗'}, 齿根厚={'✓' if res['sf_ratio_ok'] else '✗'}")

if best_design:
    print(f"\n【最优设计】")
    print(f"  模数: m1={best_design['m1']}, m2={best_design['m2']}")
    print(f"  齿数: z1={best_design['z1']}, z2={best_design['z2']}, z3={best_design['z3']}, z4={best_design['z4']}")
    print(f"  直径: D1={best_design['D1']}, D2={best_design['D2']}, D3={best_design['D3']}, D4={best_design['D4']}")
    print(f"  传动比: {best_design['ratio']:.2f}")
    print(f"  材质: {best_design['material']}")
    print(f"  最大应力: {best_design['max_stress']:.1f} MPa (许用: {best_design['allow_stress']:.1f} MPa)")
else:
    print(f"\n【警告】没有找到可行设计！")
    
print("\n" + "="*60)
