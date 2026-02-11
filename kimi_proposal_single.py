# -*- coding: utf-8 -*-
"""
二级齿轮传动参数化设计脚本 - 单进程版本
用于避免多进程资源泄漏问题
"""
import math
import gc
import time
from dataclasses import dataclass
from typing import Optional, Dict

# ==================== 可调参数配置区 ====================

# ---- 1. 性能约束参数 ----
MOTOR_TORQUE_LOWER = 0.8
OUTPUT_TORQUE = 4
MAX_MOTOR_SPEED = 4000
OUTPUT_SPEED = 250

# ---- 2. 结构约束参数 ----
MOTOR_D = 75
MIN_CLEARANCE = 1
MIN_TEETH = 17

# ---- 3. 齿轮直径扫描范围 (mm) ----
D1_MIN, D1_MAX = 15, 30
D2_MIN, D2_MAX = 50, 120
D3_MIN, D3_MAX = 10, 40
D4_MIN, D4_MAX = 30, 80
STEP = 2  # 步长2mm平衡速度和精度

# ---- 4. 模数选项 ----
M1_OPTIONS = [0.5, 0.8, 1.0]
M2_OPTIONS = [0.8, 1.0, 1.5]

# ---- 5. 齿宽系数 ----
WIDTH_FACTOR1 = 15
WIDTH_FACTOR2 = 10

# ---- 6. 优化模式 ----
OPTIMIZATION_MODE = 'size'

# ---- 7. 其他参数 ----
MOTOR_WEIGHT = 300

# ==================== 材料数据库 ====================
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

# ==================== 核心函数 ====================

def calculate_bending_stress(m, z, width, torque):
    if z < 12 or width <= 0 or m <= 0:
        return float('inf'), 0
    Y_F = 2.1 + 3.5 / z
    T = torque * 1000  # N·mm
    sigma_F = (2 * T * Y_F * 0.7) / (width * m**2 * z)
    return sigma_F, Y_F

def select_material(m1, m2, z1, z2, z3, z4, width1, width2, T1, T2):
    check_results = {}
    suitable_materials = []
    
    for key, props in MATERIAL_DB.items():
        sigma_F1, _ = calculate_bending_stress(m1, z1, width1, T1)
        sigma_F2, _ = calculate_bending_stress(m1, z2, width1, T1 * z2 / z1)
        sigma_F3, _ = calculate_bending_stress(m2, z3, width2, T2)
        sigma_F4, _ = calculate_bending_stress(m2, z4, width2, T2 * z4 / z3)
        
        max_sigma_F = max(sigma_F1, sigma_F2, sigma_F3, sigma_F4)
        sigma_F_allow = props['sigma_f'] / props['safety_factor']
        
        alpha = 20 * math.pi / 180
        sf_factor = math.pi / 2 * math.cos(alpha) - 2 * 1.25 * math.tan(alpha)
        sf_ratio_ok = (sf_factor >= props['sf_min_ratio']) and (sf_factor >= props['sf_min_ratio'])
        
        strength_ok = max_sigma_F <= sigma_F_allow
        is_suitable = strength_ok and sf_ratio_ok
        
        check_results[key] = {
            'suitable': is_suitable,
            'max_stress': max_sigma_F,
            'allow_stress': sigma_F_allow,
        }
        
        if is_suitable:
            suitable_materials.append((key, props['density']))
    
    if suitable_materials:
        suitable_materials.sort(key=lambda x: x[1])
        selected_key = suitable_materials[0][0]
    else:
        selected_key = 'steel'
    
    return selected_key, check_results

def evaluate_design(m1, m2, D1, D2, D3, D4):
    z1, z2 = round(D1 / m1), round(D2 / m1)
    z3, z4 = round(D3 / m2), round(D4 / m2)
    
    if z1 < MIN_TEETH or z2 < MIN_TEETH or z3 < MIN_TEETH or z4 < MIN_TEETH:
        return None
    
    D1r, D2r = z1 * m1, z2 * m1
    D3r, D4r = z3 * m2, z4 * m2
    
    i1, i2 = z2 / z1, z4 / z3
    ratio = i1 * i2
    
    if OUTPUT_SPEED * ratio > MAX_MOTOR_SPEED:
        return None
    
    if MOTOR_TORQUE_LOWER * ratio * 0.9025 < OUTPUT_TORQUE:
        return None
    
    if max(D2r, D4r) > MOTOR_D:
        return None
    
    if (D1r - D3r) < MIN_CLEARANCE:
        return None
    
    width1 = WIDTH_FACTOR1 * m1
    width2 = WIDTH_FACTOR2 * m2
    
    T1 = MOTOR_TORQUE_LOWER
    T2 = T1 * i1 * 0.95
    
    selected_mat, mat_results = select_material(m1, m2, z1, z2, z3, z4, width1, width2, T1, T2)
    
    if not mat_results[selected_mat]['suitable']:
        for key, result in mat_results.items():
            if result['suitable']:
                selected_mat = key
                break
        else:
            return None
    
    # 计算重量
    density = MATERIAL_DB[selected_mat]['density']
    vol1 = math.pi * (D1r / 2) ** 2 * width1
    vol2 = math.pi * (D2r / 2) ** 2 * width1
    vol3 = math.pi * (D3r / 2) ** 2 * width2
    vol4 = math.pi * (D4r / 2) ** 2 * width2
    weight_g = 0.5 * (vol1 + vol2 + vol3 + vol4) * density * 1000
    
    # 计算纵向尺寸
    total_size = MOTOR_D / 2 + D1r / 2 + D2r / 2 + D3r / 2 + D4r / 2
    
    return {
        'm1': m1, 'm2': m2, 'z1': z1, 'z2': z2, 'z3': z3, 'z4': z4,
        'D1': D1r, 'D2': D2r, 'D3': D3r, 'D4': D4r,
        'ratio': ratio,
        'material': selected_mat,
        'weight_g': weight_g,
        'total_size': total_size,
        'output_torque': MOTOR_TORQUE_LOWER * ratio * 0.9025,
    }

# ==================== 主程序 ====================

def main():
    print("=" * 60)
    print("齿轮优化设计 - 单进程版本")
    print("=" * 60)
    
    total_tasks = len(M1_OPTIONS) * len(M2_OPTIONS) * \
                  len(range(D1_MIN, D1_MAX + 1, STEP)) * \
                  len(range(D2_MIN, D2_MAX + 1, STEP)) * \
                  len(range(D3_MIN, D3_MAX + 1, STEP)) * \
                  len(range(D4_MIN, D4_MAX + 1, STEP))
    
    print(f"搜索空间: {total_tasks:,} 种组合")
    print(f"步长: {STEP} mm")
    print()
    
    best_design = None
    best_metric = float('inf')
    completed = 0
    valid_count = 0
    start_time = time.time()
    
    for m1 in M1_OPTIONS:
        for m2 in M2_OPTIONS:
            for D1 in range(D1_MIN, D1_MAX + 1, STEP):
                for D2 in range(D2_MIN, D2_MAX + 1, STEP):
                    for D3 in range(D3_MIN, D3_MAX + 1, STEP):
                        for D4 in range(D4_MIN, D4_MAX + 1, STEP):
                            result = evaluate_design(m1, m2, D1, D2, D3, D4)
                            completed += 1
                            
                            if result:
                                valid_count += 1
                                metric = result['total_size'] if OPTIMIZATION_MODE == 'size' else result['weight_g']
                                if metric < best_metric:
                                    best_metric = metric
                                    best_design = result
                            
                            if completed % 10000 == 0:
                                progress = completed / total_tasks * 100
                                elapsed = time.time() - start_time
                                eta = elapsed / progress * (100 - progress) if progress > 0 else 0
                                print(f"\r进度: {completed}/{total_tasks} ({progress:.1f}%) | "
                                      f"可行解: {valid_count} | ETA: {eta:.0f}s", end='', flush=True)
    
    print(f"\n\n搜索完成! 总耗时: {time.time() - start_time:.1f}s")
    print(f"可行方案数: {valid_count}")
    
    if best_design:
        print("\n" + "=" * 60)
        print("最优设计方案")
        print("=" * 60)
        print(f"模数: m1={best_design['m1']}, m2={best_design['m2']}")
        print(f"齿数: z1={best_design['z1']}, z2={best_design['z2']}, z3={best_design['z3']}, z4={best_design['z4']}")
        print(f"直径: D1={best_design['D1']:.1f}, D2={best_design['D2']:.1f}, D3={best_design['D3']:.1f}, D4={best_design['D4']:.1f}")
        print(f"传动比: {best_design['ratio']:.2f}")
        print(f"输出扭矩: {best_design['output_torque']:.2f} Nm")
        print(f"材质: {MATERIAL_DB[best_design['material']]['name']}")
        print(f"齿轮重量: {best_design['weight_g']:.1f} g")
        print(f"总纵向尺寸: {best_design['total_size']:.1f} mm")
    else:
        print("\n未找到可行方案!")

if __name__ == '__main__':
    main()
