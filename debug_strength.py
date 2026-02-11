# 材料强度校核诊断工具
# 用于分析为什么材料强度约束过滤掉了所有方案

import math

# 材料数据库 (与主程序一致)
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

def calculate_bending_stress(m, z, width, torque, alpha_deg=20):
    """计算齿根弯曲应力 (MPa)"""
    if z < 12 or width <= 0 or m <= 0:
        return float('inf'), 0
    
    Y_F = 2.1 + 3.5 / z
    Y_S = 1.0
    Y_epsilon = 0.7
    T = torque * 1000  # N·mm
    
    sigma_F = (2 * T * Y_F * Y_S * Y_epsilon) / (width * m**2 * z)
    return sigma_F, Y_F

def check_material_strength(m1, m2, z1, z2, z3, z4, width1, width2, T1, T2, material_db):
    """检查材料强度，返回详细结果"""
    results = []
    
    # 计算各级扭矩
    i1 = z2 / z1
    i2 = z4 / z3
    T1_out = T1 * i1 * 0.95  # 第一级输出 = 第二级输入
    T2_out = T1_out * i2 * 0.95  # 第二级输出
    
    print(f"\n  扭矩传递:")
    print(f"    T1(电机): {T1:.1f} N·m")
    print(f"    T1_out/T2_in: {T1_out:.1f} N·m (×{i1:.2f}×0.95)")
    print(f"    T2_out: {T2_out:.1f} N·m (×{i2:.2f}×0.95)")
    print(f"    总传动比: {i1*i2:.2f}")
    
    for key, props in material_db.items():
        # 计算弯曲应力
        sigma_F1, _ = calculate_bending_stress(m1, z1, width1, T1)
        sigma_F2, _ = calculate_bending_stress(m1, z2, width1, T1 * i1)
        sigma_F3, _ = calculate_bending_stress(m2, z3, width2, T1_out)
        sigma_F4, _ = calculate_bending_stress(m2, z4, width2, T1_out * i2)
        
        max_sigma_F = max(sigma_F1, sigma_F2, sigma_F3, sigma_F4)
        sigma_F_allow = props['sigma_f'] / props['safety_factor']
        
        # 齿根厚检查
        alpha = 20 * math.pi / 180
        sf_factor = math.pi / 2 * math.cos(alpha) - 2 * 1.25 * math.tan(alpha)
        sf1 = m1 * sf_factor
        sf2 = m2 * sf_factor
        sf_ratio_ok = (sf1 / m1 >= props['sf_min_ratio']) and (sf2 / m2 >= props['sf_min_ratio'])
        
        strength_ok = max_sigma_F <= sigma_F_allow
        is_suitable = strength_ok and sf_ratio_ok
        
        results.append({
            'material': key,
            'name': props['name'],
            'max_stress': max_sigma_F,
            'allow_stress': sigma_F_allow,
            'strength_ok': strength_ok,
            'sf_ratio_ok': sf_ratio_ok,
            'suitable': is_suitable,
            'sf1/m1': sf1/m1,
            'sf2/m2': sf2/m2,
            'sf_min_ratio': props['sf_min_ratio']
        })
        
        status = "✓" if is_suitable else "✗"
        print(f"\n  {status} {props['name']}:")
        print(f"      最大应力: {max_sigma_F:.1f} MPa (许用: {sigma_F_allow:.1f} MPa) {'✓' if strength_ok else '✗'}")
        print(f"      应力详情: σF1={sigma_F1:.1f}, σF2={sigma_F2:.1f}, σF3={sigma_F3:.1f}, σF4={sigma_F4:.1f}")
        print(f"      齿根厚比: sf1/m1={sf1/m1:.3f}, sf2/m2={sf2/m2:.3f} (要求≥{props['sf_min_ratio']}) {'✓' if sf_ratio_ok else '✗'}")
    
    return any(r['suitable'] for r in results)

# 测试一个典型组合
print("="*60)
print("材料强度校核诊断")
print("="*60)
print("\n参数设置:")
print(f"  电机扭矩: 0.8 Nm")
print(f"  齿宽系数: 15 (第一级), 10 (第二级)")

# 测试几个典型组合 (确保 D1 > D3 + 1)
# 注意: z1*m1 > z3*m2 + 1 (间隙约束)
test_cases = [
    # m1, m2, z1, z2, z3, z4
    (0.5, 0.5, 40, 80, 20, 50),   # D1=20, D3=10, 间隙=10 ✓, 传动比约 10.0
    (0.5, 0.5, 36, 72, 24, 48),   # D1=18, D3=12, 间隙=6 ✓, 传动比约 8.0  
    (0.8, 1.0, 25, 50, 18, 45),   # D1=20, D3=18, 间隙=2 ✓, 传动比约 6.25
    (0.8, 1.0, 30, 60, 20, 50),   # D1=24, D3=20, 间隙=4 ✓, 传动比约 7.5
    (1.0, 1.5, 20, 50, 18, 36),   # D1=20, D3=27... 等等 z3*m2=27 > D1 ✗
    (1.0, 1.5, 30, 60, 18, 36),   # D1=30, D3=27, 间隙=3 ✓, 传动比约 6.67
]

for m1, m2, z1, z2, z3, z4 in test_cases:
    D1, D2 = z1*m1, z2*m1
    D3, D4 = z3*m2, z4*m2
    width1 = 15 * m1
    width2 = 10 * m2
    T1 = 0.8  # Nm
    
    i1, i2 = z2/z1, z4/z3
    ratio = i1 * i2
    
    print(f"\n{'='*60}")
    print(f"测试组合: m1={m1}, m2={m2}, z1={z1}, z2={z2}, z3={z3}, z4={z4}")
    print(f"  直径: D1={D1}, D2={D2}, D3={D3}, D4={D4}")
    print(f"  齿宽: width1={width1:.2f}, width2={width2:.2f}")
    print(f"  传动比: {ratio:.2f}")
    
    # 检查其他约束
    valid = True
    if z1 < 17 or z2 < 17 or z3 < 17 or z4 < 17:
        print("  ✗ 齿数约束不满足")
        valid = False
    if (D1 - D3) < 1:
        print(f"  ✗ 间隙约束不满足: D1-D3={D1-D3:.2f} < 1")
        valid = False
    if max(D2, D4) > 75:
        print(f"  ✗ 包络约束不满足: max(D2,D4)={max(D2,D4)} > 75")
        valid = False
    
    if valid:
        print("  ✓ 几何约束满足，检查材料强度...")
        any_suitable = check_material_strength(m1, m2, z1, z2, z3, z4, width1, width2, T1, None, MATERIAL_DB)
        if any_suitable:
            print("\n  >>> 存在满足强度要求的材料!")
        else:
            print("\n  >>> 没有材料满足强度要求!")

print("\n" + "="*60)
