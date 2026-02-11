# Gear_Transmission_Project
二级齿轮传动系统参数优化。这是一个用于二级齿轮传动系统参数优化，项目针对给定电机规格和尺寸等约束，确定最佳齿轮参数。

## 当前程序中的约束条件如下：
一、硬约束（不满足则直接跳过该方案）

1. 齿数约束（防根切）齿数必须 ≥ 17（避免根切）
python
if z1 < 17 or z2 < 17 or z3 < 17 or z4 < 17:
    return None
2. 轴距有效性约束
python
if wheelbase1 <= 0 or wheelbase2 <= 0:
    return None

3. 电机转速约束
python
if desired_motor_speed > max_motor_speed:  # max_motor_speed = 3500 rpm
    return None

4. 输出扭矩约束
python
if actual_output_torque < output_torque:  # output_torque = 7 Nm
    return None

5. 包络尺寸约束（径向），齿轮直径都小于电机直径
python
if max(D2_real, D4_real) > motor_D:  # motor_D = 75 mm
    return None

6. 齿轮间隙约束（轴向装配）
python
if (D1_real - D3_real) < 1:
    return None
一级主动轮D1必须比二级主动轮D3大至少1mm（避免干涉）

二、材质约束（强度校核）
行为取决于 MATERIAL_MODE 设置：
模式:'auto';软约束 - 自动选择强度足够的材料。如果没有材料满足，则尝试所有材质，选择第一个suitable的；如果都没有则跳过该方案
模式:指定材质 ('steel'/'peek'/'pom'/'nylon');硬约束 - 只校核该材质，若不满足强度要求则直接跳过该方案
材质校核内容：
弯曲强度校核：max_sigma_F ≤ sigma_F_allow（许用应力）
齿根厚校核：sf/m ≥ sf_min_ratio（材料相关最小齿根厚比例）

## Debug传动比窗口过窄
用于校验齿轮直径是否合适，计算传动比窗口
依据，最小最小传动比（扭矩约束），以及最大允许传动比（转速约束），查看可接受范围是否过窄。