# -*- coding: utf-8 -*-
"""
齿轮传动优化设计程序 - 模块化版本
使用 material_check 模块进行材料校核
"""

import math
import time
from dataclasses import dataclass
from typing import Optional, Dict, List

# 导入材料校核模块
from material_check import MaterialDatabase, StrengthChecker, MaterialCheckResult

# 导入报告生成模块
from report_generator import print_result


# ==================== 配置参数 ====================

@dataclass
class GearConfig:
    """齿轮设计配置"""
    # 性能约束
    motor_torque: float = 0.8      # 电机扭矩 (Nm)
    output_torque: float = 5.0     # 输出扭矩要求 (Nm)
    max_motor_speed: float = 4000  # 电机最大转速 (rpm)
    output_speed: float = 350      # 输出转速 (rpm)
    
    # 结构约束
    motor_diameter: float = 75     # 电机直径 (mm)
    min_clearance: float = 1.0     # D1-D3最小间隙 (mm)
    min_teeth: int = 17            # 最小齿数
    
    # 扫描范围
    d1_range: tuple = (15, 30)     # 一级主动轮直径范围
    d2_range: tuple = (50, 120)    # 一级从动轮直径范围
    d3_range: tuple = (10, 40)     # 二级主动轮直径范围
    d4_range: tuple = (30, 80)     # 二级从动轮直径范围
    step: int = 2                  # 扫描步长 (mm)
    
    # 模数选项
    m1_options: tuple = (1.0, 1.2) # 增大模数,增大齿根绝对厚度
    m2_options: tuple = (0.8, 1.0, 1.5)
    
    # 齿宽系数列表 (常用值: 6, 8, 10, 12, 15)，分别控制两级齿轮厚度
    width_factors1: tuple = (10, 12, 15)  # 第一级齿宽系数（可以更大，因为第一级扭矩小）
    width_factors2: tuple = (6, 8, 10)       # 第二级齿宽系数
    
    # 混合材质配置 - 可为每个齿轮指定不同材质
    # 可选: 'steel'(钢), 'peek'(PEEK), 'pom'(POM), 'nylon'(尼龙)
    # 默认: 全部用钢
    material_z1: str = 'steel'  # 第一级主动轮
    material_z2: str = 'steel'  # 第一级从动轮  <- 可改为 'peek' 使用塑料
    material_z3: str = 'steel'  # 第二级主动轮
    material_z4: str = 'steel'  # 第二级从动轮
    
    # 优化目标
    optimize_mode: str = 'size'    # 'size'或'weight'
    
    # 其他
    motor_weight: float = 300      # 电机重量 (g)
    pressure_angle: float = 20     # 压力角 (度)


@dataclass
class GearResult:
    """齿轮设计结果"""
    m1: float
    m2: float
    z1: int
    z2: int
    z3: int
    z4: int
    d1: float
    d2: float
    d3: float
    d4: float
    ratio: float
    material: str
    material_name: str
    max_stress: float
    allow_stress: float
    weight_g: float
    total_size: float
    output_torque: float
    width_factor1: float  # 第一级齿宽系数
    width_factor2: float  # 第二级齿宽系数
    stress_details: Dict[str, float] = None  # 各齿轮应力详情
    max_stress_gear: str = ""  # 最大应力所在齿轮
    gear_materials: Dict[str, str] = None  # 各齿轮材质键名
    gear_material_names: Dict[str, str] = None  # 各齿轮材质名称
    
    def __post_init__(self):
        if self.stress_details is None:
            self.stress_details = {}
        if self.gear_materials is None:
            self.gear_materials = {}
        if self.gear_material_names is None:
            self.gear_material_names = {}


# ==================== 核心优化器 ====================

class GearOptimizer:
    """齿轮优化器"""
    
    def __init__(self, config: GearConfig):
        """
        初始化优化器
        
        Args:
            config: 齿轮设计配置
        """
        self.config = config
        self.material_db = MaterialDatabase()
        self.strength_checker = StrengthChecker(config.pressure_angle)
        
    def evaluate(self, m1: float, m2: float, d1_in: int, d2_in: int, 
                 d3_in: int, d4_in: int, wf1: float, wf2: float) -> Optional[GearResult]:
        """
        评估单个设计组合
        
        Returns:
            GearResult if valid, None otherwise
        """
        cfg = self.config
        
        # 1. 计算齿数
        z1 = round(d1_in / m1)
        z2 = round(d2_in / m1)
        z3 = round(d3_in / m2)
        z4 = round(d4_in / m2)
        
        # 2. 齿数约束
        if min(z1, z2, z3, z4) < cfg.min_teeth:
            return None
        
        # 3. 实际直径
        d1 = z1 * m1
        d2 = z2 * m1
        d3 = z3 * m2
        d4 = z4 * m2
        
        # 4. 传动比
        i1 = z2 / z1
        i2 = z4 / z3
        ratio = i1 * i2
        
        # 5. 转速约束
        if cfg.output_speed * ratio > cfg.max_motor_speed:
            return None
        
        # 6. 扭矩约束
        actual_torque = cfg.motor_torque * ratio * 0.9025  # 0.95^2
        if actual_torque < cfg.output_torque:
            return None
        
        # 7. 包络约束
        if max(d2, d4) > cfg.motor_diameter:
            return None
        
        # 8. 间隙约束
        if (d1 - d3) < cfg.min_clearance:
            return None
        
        # 9. 材料校核（支持混合材质）
        width1 = wf1 * m1
        width2 = wf2 * m2
        torque1 = cfg.motor_torque
        torque2 = torque1 * i1 * 0.95
        
        # 构建材质配置
        material_config = {
            'z1': cfg.material_z1,
            'z2': cfg.material_z2,
            'z3': cfg.material_z3,
            'z4': cfg.material_z4
        }
        
        # 检查是否使用混合材质
        unique_materials = set(material_config.values())
        if len(unique_materials) > 1:
            # 使用混合材质校核
            result = self.strength_checker.check_mixed_materials(
                self.material_db, m1, m2, z1, z2, z3, z4,
                width1, width2, torque1, torque2, material_config
            )
        else:
            # 使用单一材质校核（原有逻辑）
            selected_key, results = self.strength_checker.select_best_material(
                self.material_db, m1, m2, z1, z2, z3, z4,
                width1, width2, torque1, torque2, mode='auto'
            )
            result = results[selected_key]
        
        # 检查是否有材料满足
        if not result.suitable:
            return None  # 没有材料满足
        
        # 找出最大应力所在齿轮
        max_stress_gear = max(result.stress_details.items(), key=lambda x: x[1])[0]
        
        # 10. 计算重量（各齿轮按实际材质密度计算）
        if result.gear_materials:
            # 混合材质：分别计算各齿轮重量
            weight_g = 0
            gears = [
                (d1, width1, 'z1'), (d2, width1, 'z2'),
                (d3, width2, 'z3'), (d4, width2, 'z4')
            ]
            for d, width, gear_key in gears:
                mat_key = result.gear_materials.get(gear_key, 'steel')
                material = self.material_db.get(mat_key) or self.material_db['steel']
                vol = math.pi * (d/2)**2 * width
                weight_g += 0.5 * vol * material.density * 1000
        else:
            # 单一材质
            vol = (
                math.pi * (d1/2)**2 * width1 +
                math.pi * (d2/2)**2 * width1 +
                math.pi * (d3/2)**2 * width2 +
                math.pi * (d4/2)**2 * width2
            )
            weight_g = 0.5 * vol * result.density * 1000
        
        # 11. 纵向尺寸
        total_size = cfg.motor_diameter/2 + d1/2 + d2/2 + d3/2 + d4/2
        
        return GearResult(
            m1=m1, m2=m2, z1=z1, z2=z2, z3=z3, z4=z4,
            d1=d1, d2=d2, d3=d3, d4=d4,
            ratio=ratio,
            material='mixed' if len(unique_materials) > 1 else list(unique_materials)[0],
            material_name=result.name,
            max_stress=result.max_stress,
            allow_stress=result.allow_stress,
            weight_g=weight_g,
            total_size=total_size,
            output_torque=actual_torque,
            width_factor1=wf1,
            width_factor2=wf2,
            stress_details=result.stress_details,
            max_stress_gear=max_stress_gear,
            gear_materials=result.gear_materials,
            gear_material_names=result.gear_material_names
        )
    
    def optimize(self, progress_interval: int = 10000) -> tuple:
        """
        执行优化搜索
        
        Args:
            progress_interval: 进度报告间隔
            
        Returns:
            (best_result, valid_count, total_count)
        """
        cfg = self.config
        
        # 生成搜索空间
        d1_list = list(range(cfg.d1_range[0], cfg.d1_range[1] + 1, cfg.step))
        d2_list = list(range(cfg.d2_range[0], cfg.d2_range[1] + 1, cfg.step))
        d3_list = list(range(cfg.d3_range[0], cfg.d3_range[1] + 1, cfg.step))
        d4_list = list(range(cfg.d4_range[0], cfg.d4_range[1] + 1, cfg.step))
        
        total_count = (
            len(cfg.m1_options) * len(cfg.m2_options) *
            len(d1_list) * len(d2_list) * len(d3_list) * len(d4_list) *
            len(cfg.width_factors1) * len(cfg.width_factors2)
        )
        
        print(f"搜索空间: {total_count:,} 种组合")
        print(f"步长: {cfg.step} mm")
        print(f"第一级齿宽系数: {cfg.width_factors1}")
        print(f"第二级齿宽系数: {cfg.width_factors2}")
        print(f"优化目标: {'纵向尺寸最短' if cfg.optimize_mode == 'size' else '重量最轻'}")
        print()
        
        best_result = None
        best_metric = float('inf')
        valid_count = 0
        completed = 0
        start_time = time.time()
        
        # 搜索
        for m1 in cfg.m1_options:
            for m2 in cfg.m2_options:
                for d1 in d1_list:
                    for d2 in d2_list:
                        for d3 in d3_list:
                            for d4 in d4_list:
                                for wf1 in cfg.width_factors1:
                                    for wf2 in cfg.width_factors2:
                                        result = self.evaluate(m1, m2, d1, d2, d3, d4, wf1, wf2)
                                        completed += 1
                                        
                                        if result:
                                            valid_count += 1
                                            metric = (
                                                result.total_size 
                                                if cfg.optimize_mode == 'size' 
                                                else result.weight_g
                                            )
                                            if metric < best_metric:
                                                best_metric = metric
                                                best_result = result
                                        
                                        # 进度报告
                                        if completed % progress_interval == 0:
                                            progress = completed / total_count * 100
                                            elapsed = time.time() - start_time
                                            eta = (elapsed / progress * (100 - progress) 
                                                   if progress > 0 else 0)
                                            print(f"\r进度: {completed}/{total_count} "
                                                  f"({progress:.1f}%) | "
                                                  f"可行: {valid_count} | "
                                                  f"ETA: {eta:.0f}s", end='', flush=True)
        
        print()  # 换行
        return best_result, valid_count, total_count


# ==================== 主程序 ====================

def main():
    """主函数"""
    print("=" * 60)
    print("齿轮传动优化设计程序")
    print("=" * 60)
    
    # 创建配置
    config = GearConfig()
    
    # 创建优化器并运行
    optimizer = GearOptimizer(config)
    
    start_time = time.time()
    best_result, valid_count, total_count = optimizer.optimize()
    elapsed = time.time() - start_time
    
    # 输出结果
    print(f"\n搜索完成!")
    print(f"  总耗时: {elapsed:.1f} 秒")
    print(f"  可行方案: {valid_count}/{total_count}")
    
    print_result(best_result, config)


if __name__ == '__main__':
    main()
