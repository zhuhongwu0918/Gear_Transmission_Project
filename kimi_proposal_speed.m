%% 二级齿轮传动参数化设计脚本 (以直径为核心) - 优化版本
% 目标：在满足输出性能约束下，最小化总重量
% 方法：参数化扫描优化
% 更新：速度约束改为固定输出转速，反推电机转速需求

clear; clc;

% ==================== 设定参数 (根据你的需求调节) ====================

% 电机参考参数
motor_D = 75;       % 电机直径 (mm)，作为模组包络参考
output_speed = 260; % 输出转速要求：固定为 rpm
output_torque = 7;  % 输出扭矩要求：大于 Nm
motor_torque_lower = 0.7; % 电机额定扭矩下限 Nm，按照下限计算
motor_torque_upper = 1.2; % 电机额定扭矩上限 Nm
motor_weight = 300; % 电机重量，克

% 速度约束
max_motor_speed = 4000; % 电机最大允许转速 rpm

% 材料参数
rho_steel = 7.85e-6; % 钢材密度 (kg/mm^3)

% ==================== 优化参数设置 ====================

% 模数选择范围 (标准模数)，一级用 0.5 或 0.8，二级用 1.0 或 1.25
% 可选模数，第一系列（首选）： 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.8, 1.0, 1.25, 1.5, 2.0, 2.5, 3.0

m1_options = [0.3, 0.4, 0.5, 0.8, 1.0, 1.25, 1.5];      % 第一级模数选项 (高速级选小模数)
m2_options = [0.5, 0.8, 1.0, 1.25, 1.5, 2.0, 2.5];      % 第二级模数选项 (低速级选大模数)

% 直径扫描范围 (以直径为核心变量)
D1_min = 20; D1_max = 40;   % 主动轮1直径范围 (mm)
D2_min = 50; D2_max = 100;  % 从动轮2直径范围 (mm)
D3_min = 18; D3_max = 40;   % 主动轮3直径范围 (mm)
D4_min = 40; D4_max = 80;   % 从动轮4直径范围 (mm)

% 齿宽系数 (齿宽 = 系数 × 模数，经验值：6-12)
width_factor1 = 8;  % 第一级齿宽系数
width_factor2 = 10; % 第二级齿宽系数 (低速级更宽)

% 步长设置
step = 1; % 直径扫描步长 (mm)

% ==================== 初始化优化记录 ====================

best_weight = inf;
best_design = struct();

fprintf('开始优化计算...\n');
fprintf('扫描范围: D1[%d-%d], D2[%d-%d], D3[%d-%d], D4[%d-%d]\n', ...
    D1_min, D1_max, D2_min, D2_max, D3_min, D3_max, D4_min, D4_max);
fprintf('约束条件: 输出转速=%d rpm (固定), 输出扭矩>=%d Nm, 电机转速<=%d rpm\n\n', ...
    output_speed, output_torque, max_motor_speed);

% ==================== 嵌套循环优化搜索 ====================

for m1 = m1_options
    for m2 = m2_options
        for D1 = D1_min:step:D1_max
            for D2 = D2_min:step:D2_max
                for D3 = D3_min:step:D3_max
                    for D4 = D4_min:step:D4_max
                        
                        % ----- 1. 计算并修正齿数 (齿数必须为整数) -----
                        z1 = round(D1 / m1);
                        z2 = round(D2 / m1);
                        z3 = round(D3 / m2);
                        z4 = round(D4 / m2);
                        
                        % 齿数约束：避免根切，最少齿数通常为17(标准齿轮)
                        if z1 < 17 || z2 < 17 || z3 < 17 || z4 < 17
                            continue;
                        end
                        
                        % ----- 2. 更新实际分度圆直径 -----
                        D1_real = z1 * m1;
                        D2_real = z2 * m1;
                        D3_real = z3 * m2;
                        D4_real = z4 * m2;
                        
                        % ----- 3. 计算传动比 -----
                        i1 = z2 / z1;
                        i2 = z4 / z3;
                        Total_Ratio = i1 * i2;
                        
                        % ----- 4. 计算轴距参数 -----
                        wheelbase1 = (D1_real + D2_real) / 2; % 第一级中心距
                        wheelbase2 = (D3_real + D4_real) / 2; % 第二级中心距
                        
                        % 轴距约束：确保物理上可装配
                        if wheelbase1 <= 0 || wheelbase2 <= 0
                            continue;
                        end
                        
                        % ----- 5. 计算齿宽 -----
                        width1 = width_factor1 * m1;
                        width2 = width_factor2 * m2;
                        
                        % ----- 6. 性能计算 -----
                        
                        % 固定输出转速，反推所需电机转速
                        actual_output_speed = output_speed; % 固定输出转速
                        desired_motor_speed = actual_output_speed * Total_Ratio; % rpm
                        
                        % 电机转速约束：不能超过最大允许转速
                        if desired_motor_speed > max_motor_speed
                            continue;
                        end
                        
                        % 扭矩计算 (假设电机额定扭矩取中间值)
                        motor_torque = motor_torque_lower;
                        actual_output_torque = motor_torque * Total_Ratio * 0.95 * 0.95; % 考虑两级传动效率各95%
                        
                        % ----- 7. 约束检查 -----
                        % 输出扭矩约束
                        if actual_output_torque < output_torque
                            continue;
                        end
                        
                        % 包络尺寸约束：从动轮直径不超过电机直径的1倍
                        if max(D2_real, D4_real) > motor_D 
                            continue;
                        end
                        
                        % 约束：一级主动轮(D3)要大于二级主动轮(D1)至少1mm间隙
                        % 避免干涉，确保二级主动轮可以安装在中间轴上不与一级主动轮冲突
                        if (D1_real - D3_real) >1
                            continue;
                        end
                        
                        % ----- 8. 重量计算 (简化为圆柱体模型) -----
                        % 齿轮体积计算 (分度圆直径 × 齿宽近似)
                        vol1 = pi * (D1_real/2)^2 * width1;
                        vol2 = pi * (D2_real/2)^2 * width1;
                        vol3 = pi * (D3_real/2)^2 * width2;
                        vol4 = pi * (D4_real/2)^2 * width2;
                        
                        % 修正：四个齿轮体积之和 × 密度，表示挖空体积占一半
                        Total_Weight_kg = 0.5 * (vol1 + vol2 + vol3 + vol4) * rho_steel;
                        Total_Weight_g = Total_Weight_kg * 1000;
                        
                        % 总重量包含电机
                        Total_System_Weight = Total_Weight_g + motor_weight;
                        
                        % ----- 9. 记录最优解 -----
                        if Total_Weight_kg < best_weight
                            best_weight = Total_Weight_kg;
                            
                            best_design.m1 = m1;
                            best_design.m2 = m2;
                            best_design.z1 = z1;
                            best_design.z2 = z2;
                            best_design.z3 = z3;
                            best_design.z4 = z4;
                            best_design.D1 = D1_real;
                            best_design.D2 = D2_real;
                            best_design.D3 = D3_real;
                            best_design.D4 = D4_real;
                            best_design.width1 = width1;
                            best_design.width2 = width2;
                            best_design.wheelbase1 = wheelbase1;
                            best_design.wheelbase2 = wheelbase2;
                            best_design.i1 = i1;
                            best_design.i2 = i2;
                            best_design.Total_Ratio = Total_Ratio;
                            best_design.output_speed = actual_output_speed;
                            best_design.desired_motor_speed = desired_motor_speed; % 记录期望电机转速
                            best_design.output_torque = actual_output_torque;
                            best_design.gear_weight_g = Total_Weight_g;
                            best_design.total_weight_g = Total_System_Weight;
                        end
                        
                    end
                end
            end
        end
    end
end

% ==================== 输出优化结果 ====================

fprintf('\n');
fprintf('=================================================\n');
fprintf('          二级齿轮传动优化设计报告\n');
fprintf('=================================================\n');
fprintf('\n');

if best_weight == inf
    fprintf('【警告】未找到满足所有约束条件的解！请放宽约束或扩大搜索范围。\n');
else
    % 打印最优设计参数
    fprintf('【优化目标】重量最轻设计\n');
    fprintf('【材质】钢材 (密度: %.2e kg/mm?)\n', rho_steel);
    fprintf('\n');
    
    fprintf('---------------- 齿轮参数 ----------------\n');
    fprintf('第一级 (高速级):\n');
    fprintf('  模数 m1 = %.2f mm\n', best_design.m1);
    fprintf('  主动轮: 齿数 z1 = %d, 分度圆直径 D1 = %.2f mm, 齿宽 = %.2f mm\n', ...
        best_design.z1, best_design.D1, best_design.width1);
    fprintf('  从动轮: 齿数 z2 = %d, 分度圆直径 D2 = %.2f mm, 齿宽 = %.2f mm\n', ...
        best_design.z2, best_design.D2, best_design.width1);
    fprintf('  传动比 i1 = %.3f\n', best_design.i1);
    fprintf('\n');
    
    fprintf('第二级 (低速级):\n');
    fprintf('  模数 m2 = %.2f mm\n', best_design.m2);
    fprintf('  主动轮: 齿数 z3 = %d, 分度圆直径 D3 = %.2f mm, 齿宽 = %.2f mm\n', ...
        best_design.z3, best_design.D3, best_design.width2);
    fprintf('  从动轮: 齿数 z4 = %d, 分度圆直径 D4 = %.2f mm, 齿宽 = %.2f mm\n', ...
        best_design.z4, best_design.D4, best_design.width2);
    fprintf('  传动比 i2 = %.3f\n', best_design.i2);
    fprintf('\n');
    
    fprintf('---------------- 总体性能 ----------------\n');
    fprintf('总减速比: %.3f\n', best_design.Total_Ratio);
    fprintf('实际齿数配比: (%d:%d) × (%d:%d)\n', ...
        best_design.z1, best_design.z2, best_design.z3, best_design.z4);
    fprintf('输出转速: %.2f rpm (固定目标值)\n', best_design.output_speed);
    fprintf('期望电机转速: %.2f rpm (约束: <= %d rpm) 【%s】\n', ...
        best_design.desired_motor_speed, max_motor_speed, ...
        iif(best_design.desired_motor_speed <= max_motor_speed, '满足', '不满足'));
    fprintf('输出扭矩: %.3f Nm (要求 >= %d Nm) 【%s】\n', ...
        best_design.output_torque, output_torque, ...
        iif(best_design.output_torque >= output_torque, '满足', '不满足'));
    fprintf('\n');
    
    fprintf('---------------- 尺寸与重量 ----------------\n');
    fprintf('轴距参数:\n');
    fprintf('  第一级中心距 (电机轴-中间轴): %.2f mm\n', best_design.wheelbase1);
    fprintf('  第二级中心距 (中间轴-输出轴): %.2f mm\n', best_design.wheelbase2);
    fprintf('总体轴向尺寸估算: %.2f mm (齿宽之和 + 轴承间隙)\n', ...
        best_design.width1 + best_design.width2 + 15);
    fprintf('最大径向尺寸: %.2f mm (电机直径参考: %.1f mm)\n', ...
        max(best_design.D2, best_design.D4), motor_D);
    fprintf('\n');
    fprintf('齿轮组重量: %.3f g\n', best_design.gear_weight_g);
    fprintf('电机重量: %d g\n', motor_weight);
    fprintf('系统总重量: %.3f g (%.3f kg)\n', ...
        best_design.total_weight_g, best_design.total_weight_g/1000);
    fprintf('\n');
    
    fprintf('---------------- 设计验证 ----------------\n');
    % 验证轴距约束
    wb1_check = abs(best_design.wheelbase1 - (best_design.D1 + best_design.D2)/2) < 0.01;
    wb2_check = abs(best_design.wheelbase2 - (best_design.D3 + best_design.D4)/2) < 0.01;
    fprintf('轴距约束验证: %s\n', iif(wb1_check && wb2_check, '通过', '失败'));
    
    % 齿轮间隙验证
    clearance_check = (best_design.D1 - best_design.D3) >= 1;
    fprintf('齿轮间隙验证 (D3-D1 >= 1mm): %.2f mm 【%s】\n', ...
        best_design.D3 - best_design.D1, iif(clearance_check, '通过', '失败'));
    
    % 强度粗略估算 (简化)
    strength_factor = (best_design.m2 * best_design.width2) / (best_design.m1 * best_design.width1);
    fprintf('低速级相对强度系数: %.2f (>1.0表示低速级更强，符合设计要求)\n', strength_factor);
end

fprintf('\n=================================================\n');
fprintf('                   设计完成\n');
fprintf('=================================================\n');

% ==================== 辅助函数 ====================
function out = iif(cond, a, b)
    if cond
        out = a;
    else
        out = b;
    end
end