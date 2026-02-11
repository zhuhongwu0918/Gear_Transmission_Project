# ==================== 可调参数配置区 ====================

# 齿轮直径扫描范围 (mm)
D1_MIN, D1_MAX = 15, 30      # 主动轮1直径范围 (一级主动轮)
D2_MIN, D2_MAX = 50, 120      # 从动轮2直径范围 (一级从动轮)
D3_MIN, D3_MAX = 10, 40      # 主动轮3直径范围 (二级主动轮)
D4_MIN, D4_MAX = 30, 80      # 从动轮4直径范围 (二级从动轮)
STEP = 5                     # 扫描步长 (mm)，调大可以加快计算

# 模数选项
M1_OPTIONS = [0.3, 0.4, 0.5, 0.8, 1.0, 1.25, 1.5]    # 第一级模数选项
M2_OPTIONS = [0.5, 0.8, 1.0, 1.25, 1.5, 2.0, 2.5]        # 第二级模数选项

# 性能约束参数
MOTOR_TORQUE_LOWER = 0.8     # 电机额定扭矩下限 (Nm)
OUTPUT_TORQUE = 4            # 输出扭矩要求 (Nm)
MAX_MOTOR_SPEED = 4000       # 电机最大允许转速 (rpm)
OUTPUT_SPEED = 250           # 输出转速要求 (rpm)

# 结构约束参数
MOTOR_D = 75                 # 电机直径 (mm)，用于包络约束
MIN_CLEARANCE = 1            # D1与D3之间最小间隙 (mm)
MIN_TEETH = 17               # 最小齿数 (避免根切)

# ==================== 分析计算 ====================

# 计算理论约束边界
total_efficiency = 0.95 * 0.95  # 两级传动效率
required_ratio = OUTPUT_TORQUE / MOTOR_TORQUE_LOWER / total_efficiency
max_allowed_ratio = MAX_MOTOR_SPEED / OUTPUT_SPEED

print("=" * 50)
print("齿轮传动比可行性分析")
print("=" * 50)
print(f"\n【当前配置】")
print(f"  D1范围: [{D1_MIN}, {D1_MAX}] mm")
print(f"  D2范围: [{D2_MIN}, {D2_MAX}] mm")
print(f"  D3范围: [{D3_MIN}, {D3_MAX}] mm")
print(f"  D4范围: [{D4_MIN}, {D4_MAX}] mm")
print(f"  扫描步长: {STEP} mm")
print(f"  电机扭矩: {MOTOR_TORQUE_LOWER} Nm")
print(f"  输出扭矩要求: {OUTPUT_TORQUE} Nm")
print(f"  电机最大转速: {MAX_MOTOR_SPEED} rpm")
print(f"  输出转速: {OUTPUT_SPEED} rpm")

print(f"\n【约束边界】")
print(f"  所需最小传动比: {required_ratio:.2f} (扭矩约束)")
print(f"  最大允许传动比: {max_allowed_ratio:.2f} (转速约束)")
print(f"  可行区间宽度: {((max_allowed_ratio/required_ratio - 1) * 100):.1f}%")

# 分析传动比范围
min_ratio = float('inf')
max_ratio = 0
valid_combos = 0
feasible_combos = 0  # 同时满足扭矩和转速约束的组合

for m1 in M1_OPTIONS:
    for m2 in M2_OPTIONS:
        for D1 in range(D1_MIN, D1_MAX + 1, STEP):
            for D2 in range(D2_MIN, D2_MAX + 1, STEP):
                for D3 in range(D3_MIN, D3_MAX + 1, STEP):
                    for D4 in range(D4_MIN, D4_MAX + 1, STEP):
                        # 计算齿数
                        z1 = round(D1 / m1)
                        z2 = round(D2 / m1)
                        z3 = round(D3 / m2)
                        z4 = round(D4 / m2)
                        
                        # 齿数约束
                        if z1 < MIN_TEETH or z2 < MIN_TEETH or z3 < MIN_TEETH or z4 < MIN_TEETH:
                            continue
                        
                        # 实际直径
                        D1r = z1 * m1
                        D2r = z2 * m1
                        D3r = z3 * m2
                        D4r = z4 * m2
                        
                        # 间隙约束
                        if (D1r - D3r) < MIN_CLEARANCE:
                            continue
                        
                        # 包络约束
                        if max(D2r, D4r) > MOTOR_D:
                            continue
                        
                        # 计算传动比
                        ratio = (z2 / z1) * (z4 / z3)
                        
                        min_ratio = min(min_ratio, ratio)
                        max_ratio = max(max_ratio, ratio)
                        valid_combos += 1
                        
                        # 检查是否满足扭矩和转速双约束
                        if required_ratio <= ratio <= max_allowed_ratio:
                            feasible_combos += 1

print(f"\n【计算结果】")
print(f"  有效组合数: {valid_combos}")
print(f"  可行组合数: {feasible_combos}")
print(f"  实际传动比范围: {min_ratio:.2f} ~ {max_ratio:.2f}")

if feasible_combos > 0:
    print(f"\n  ✓ 存在 {feasible_combos} 个满足所有约束的方案")
else:
    print(f"\n  ✗ 无可行方案！")
    
    # 诊断问题
    print(f"\n【问题诊断】")
    if max_ratio < required_ratio:
        print(f"  - 最大传动比({max_ratio:.2f}) < 所需传动比({required_ratio:.2f})")
        print(f"  - 建议: 增大D2或D4范围，或使用更小模数")
    elif min_ratio > max_allowed_ratio:
        print(f"  - 最小传动比({min_ratio:.2f}) > 允许最大值({max_allowed_ratio:.2f})")
        print(f"  - 建议: 减小D1或增大D3范围，或提高电机最大转速")
    else:
        print(f"  - 传动比范围覆盖目标区间，但可能其他约束(间隙/包络)导致无可行解")
        print(f"  - 建议: 放宽MIN_CLEARANCE或增大MOTOR_D")

print("\n" + "=" * 50)
